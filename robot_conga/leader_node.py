"""ROS 2 Leader Node for Robot Conga sequential path following.

The leader is driven by an operator (teleop). This node records its world-frame path and
publishes, for the followers:
  - /conga/path (nav_msgs/Path, transient-local): the recent window of waypoints in the world
    frame, each with its arc length s in pose.position.z (see robot_conga.transport).
  - /conga/leader_state (geometry_msgs/Vector3Stamped): x = s_L (m), y = u_L (m/s, smoothed
    ds_L/dt), z = omega_L (rad/s, smoothed dtheta/dt). s_L is on the same waypoint/s
    representation the followers use.
and for logging: conga/pose and conga/reference (PoseStamped, world frame, z = s_L).
"""

import math
from typing import Optional

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from geometry_msgs.msg import PoseStamped, Vector3Stamped
    from nav_msgs.msg import Path as NavPath

    from robot_conga.ros_common import WorldPoseSource, spin, to_pose_stamped
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object  # type: ignore

from robot_conga.geometry import wrap_to_pi
from robot_conga.models import Pose
from robot_conga.transport import LeaderPathRecorder


class LeaderNode(Node):
    def __init__(self) -> None:
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 (rclpy, nav_msgs, geometry_msgs, tf2_ros) is required to run LeaderNode.")
        super().__init__("leader_node")

        self.declare_parameter("num_robots", 3)
        self.declare_parameter("delta_s", 1.0)
        self.declare_parameter("min_dist", 0.05)
        self.declare_parameter("path_window", 0.0)  # 0 -> (N-1)*d + 2 m
        self.declare_parameter("max_path_points", 400)
        self.declare_parameter("startup_prefix", True)
        self.declare_parameter("rate", 20.0)

        n = int(self.get_parameter("num_robots").value)
        d = float(self.get_parameter("delta_s").value)
        window = float(self.get_parameter("path_window").value) or (n - 1) * d + 2.0
        self.recorder = LeaderPathRecorder(
            min_dist=float(self.get_parameter("min_dist").value),
            window=window,
            max_points=int(self.get_parameter("max_path_points").value),
            prefix_length=(n - 1) * d if self.get_parameter("startup_prefix").value else 0.0,
        )

        self.pose_source = WorldPoseSource(self)
        self.world_frame = self.pose_source.world_frame
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(NavPath, "/conga/path", latched)
        self.state_pub = self.create_publisher(Vector3Stamped, "/conga/leader_state", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "conga/pose", 10)
        self.ref_pub = self.create_publisher(PoseStamped, "conga/reference", 10)

        self.prev: Optional[tuple] = None  # (time, s_L, theta)
        self.u_l = 0.0
        self.w_l = 0.0
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.tick)
        self.get_logger().info(f"Leader: {n} robots, d = {d:.2f} m, path window {window:.1f} m.")

    def tick(self) -> None:
        pose = self.pose_source.latest
        if pose is None or self.pose_source.age() > 0.5:
            return  # followers' watchdogs stop them when leader_state goes stale
        if self.recorder.add(pose.x, pose.y, pose.theta):
            self.publish_path()
        s_l = self.recorder.leader_s(pose.x, pose.y)

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self.prev is not None and t > self.prev[0]:
            dt = t - self.prev[0]
            a = min(1.0, dt / 0.25)  # ~0.25 s low-pass on the finite differences
            self.u_l += a * ((s_l - self.prev[1]) / dt - self.u_l)
            self.w_l += a * (wrap_to_pi(pose.theta - self.prev[2]) / dt - self.w_l)
        self.prev = (t, s_l, pose.theta)

        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.world_frame
        msg.vector.x, msg.vector.y, msg.vector.z = float(s_l), float(max(self.u_l, 0.0)), float(self.w_l)
        self.state_pub.publish(msg)
        self.pose_pub.publish(to_pose_stamped(self, pose, self.world_frame))
        self.ref_pub.publish(to_pose_stamped(self, pose, self.world_frame, z=s_l))

    def publish_path(self) -> None:
        msg = NavPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        for s, x, y in self.recorder.points:
            msg.poses.append(to_pose_stamped(self, Pose(x, y, 0.0), self.world_frame, z=s))
        self.path_pub.publish(msg)


def main(args=None) -> None:
    if not HAS_ROS2:
        raise RuntimeError("ROS 2 is required to run LeaderNode.")
    spin(LeaderNode)


if __name__ == "__main__":
    main()
