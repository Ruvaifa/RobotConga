"""ROS 2 Follower Node for Robot Conga sequential path following.

Tracks s_i = s_L - i * delta_s on the leader's published path with the frame-rotated
Eq. (8) controller (RotatedCongaController).

Subscribes:
  - pose (see WorldPoseSource): odom + static TF, or a world-frame PoseStamped topic
  - /conga/path (nav_msgs/Path): leader waypoint window, s in pose.position.z
  - /conga/leader_state (geometry_msgs/Vector3Stamped): x = s_L, y = u_L, z = omega_L
  - scan (sensor_msgs/LaserScan): front e-stop
Publishes:
  - cmd_vel (Twist or TwistStamped, parameter cmd_vel_type)
  - conga/pose, conga/reference (PoseStamped, world frame; reference z = s_i) for logging

Safety, checked every cycle before the controller runs (any failure -> zero velocity):
  - watchdog: pose or leader state older than watchdog_timeout (0.2 s)
  - e-stop: obstacle closer than estop_distance (0.3 m) in front, or no scan for scan_timeout
  - s_i outside the received path window
The singularity guard is a last resort: if Eq. (8) is singular even in the rotated frame,
the robot turns in place toward theta* (singularity_fallback) instead of freezing.
"""

import math
from typing import Optional

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped, Vector3Stamped
    from nav_msgs.msg import Path as NavPath
    from sensor_msgs.msg import LaserScan

    from robot_conga.ros_common import (
        MAX_SAFE_SPEED,
        WorldPoseSource,
        make_cmd_vel_publisher,
        spin,
        to_pose_stamped,
    )
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object  # type: ignore

from robot_conga.controller import ControllerSingularityError, RotatedCongaController, singularity_fallback
from robot_conga.transport import SampledPath, front_min_range


class FollowerNode(Node):
    def __init__(self) -> None:
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 (rclpy, nav_msgs, geometry_msgs, sensor_msgs, tf2_ros) is required.")
        super().__init__("follower_node")

        params = {
            "follower_index": 1,
            "delta_s": 1.0,
            "lambda1": 4.5,
            "lambda2": 7.5,
            "lambda3": 2.5,
            "max_linear_velocity": MAX_SAFE_SPEED,
            "max_angular_velocity": 1.5,
            "singularity_threshold": 1e-3,
            "cmd_vel_type": "TwistStamped",
            "rate": 20.0,
            "watchdog_timeout": 0.2,
            "path_smoothing_std": 0.01,
            "use_estop": True,
            "estop_distance": 0.3,
            "estop_half_angle_deg": 30.0,
            "scan_front_angle_deg": -90.0,  # TB4 RPLIDAR is yawed +90 deg: robot front = -90 deg in scan frame
            "scan_timeout": 0.5,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in params}
        self.p = p

        self.idx = int(p["follower_index"])
        self.offset = self.idx * float(p["delta_s"])
        self.controller = RotatedCongaController(
            lambda1=float(p["lambda1"]),
            lambda2=float(p["lambda2"]),
            lambda3=float(p["lambda3"]),
            singularity_threshold=float(p["singularity_threshold"]),
            max_linear_velocity=min(float(p["max_linear_velocity"]), MAX_SAFE_SPEED),
            max_angular_velocity=float(p["max_angular_velocity"]),
        )

        self.pose_source = WorldPoseSource(self)
        self.world_frame = self.pose_source.world_frame
        self.path: Optional[SampledPath] = None
        self.leader_s = 0.0
        self.leader_u = 0.0
        self.leader_rx: Optional[Time] = None
        self.front_range = math.inf
        self.scan_rx: Optional[Time] = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(NavPath, "/conga/path", self.on_path, latched)
        self.create_subscription(Vector3Stamped, "/conga/leader_state", self.on_leader_state, 10)
        if p["use_estop"]:
            self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)

        self.publish_cmd = make_cmd_vel_publisher(self, p["cmd_vel_type"])
        self.pose_pub = self.create_publisher(PoseStamped, "conga/pose", 10)
        self.ref_pub = self.create_publisher(PoseStamped, "conga/reference", 10)
        self.create_timer(1.0 / float(p["rate"]), self.control_loop)
        self.get_logger().info(
            f"Follower {self.idx}: offset {self.offset:.2f} m, cmd_vel type {p['cmd_vel_type']}, "
            f"pose source {self.pose_source.mode}."
        )

    def _age(self, stamp: Optional[Time]) -> float:
        return math.inf if stamp is None else (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def on_path(self, msg: NavPath) -> None:
        pts = [(ps.pose.position.z, ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) < 4:
            return
        s, x, y = zip(*pts)
        try:
            self.path = SampledPath(s, x, y, smoothing_std=float(self.p["path_smoothing_std"]))
        except Exception as e:  # keep the previous path rather than dropping out
            self.get_logger().warn(f"Path fit failed, keeping previous path: {e}", throttle_duration_sec=2.0)

    def on_leader_state(self, msg: Vector3Stamped) -> None:
        self.leader_s = float(msg.vector.x)
        self.leader_u = max(0.0, float(msg.vector.y))
        self.leader_rx = self.get_clock().now()

    def on_scan(self, msg: LaserScan) -> None:
        self.front_range = front_min_range(
            msg.ranges, msg.angle_min, msg.angle_increment,
            math.radians(float(self.p["scan_front_angle_deg"])),
            math.radians(float(self.p["estop_half_angle_deg"])),
            msg.range_min, msg.range_max,
        )
        self.scan_rx = self.get_clock().now()

    def stop(self, reason: str) -> None:
        self.publish_cmd(0.0, 0.0)
        self.get_logger().warn(f"Stopped: {reason}", throttle_duration_sec=2.0)

    def control_loop(self) -> None:
        timeout = float(self.p["watchdog_timeout"])
        pose = self.pose_source.latest
        if self.pose_source.age() > timeout:
            return self.stop(f"pose older than {timeout} s")
        if self._age(self.leader_rx) > timeout:
            return self.stop(f"leader state older than {timeout} s")
        if self.p["use_estop"]:
            if self._age(self.scan_rx) > float(self.p["scan_timeout"]):
                return self.stop("no lidar scan")
            if self.front_range < float(self.p["estop_distance"]):
                return self.stop(f"obstacle {self.front_range:.2f} m ahead")
        if self.path is None:
            return self.stop("no path yet")

        target_s = self.leader_s - self.offset
        ref = self.path.reference(target_s, self.leader_u)
        if ref is None:
            return self.stop(f"s_i = {target_s:.2f} m is outside the path window (starts {self.path.s0:.2f} m)")

        try:
            cmd = self.controller.compute(pose, ref)
        except ControllerSingularityError as e:
            self.get_logger().warn(f"Singular even in rotated frame ({e}); turning toward theta*.",
                                   throttle_duration_sec=1.0)
            cmd = singularity_fallback(pose, ref, max_angular_velocity=float(self.p["max_angular_velocity"]))
        self.publish_cmd(cmd.linear, cmd.angular)
        self.pose_pub.publish(to_pose_stamped(self, pose, self.world_frame))
        self.ref_pub.publish(to_pose_stamped(self, ref.pose, self.world_frame, z=ref.s))

    def on_shutdown(self) -> None:
        self.publish_cmd(0.0, 0.0)


def main(args=None) -> None:
    if not HAS_ROS2:
        raise RuntimeError("ROS 2 is required to run FollowerNode.")
    spin(FollowerNode)


if __name__ == "__main__":
    main()
