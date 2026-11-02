"""ROS 2 Follower Node for Robot Conga sequential path following."""

import math
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path as NavPath
    from geometry_msgs.msg import Twist as RosTwist, Vector3
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object  # type: ignore

from robot_conga.bspline_path import PathManager
from robot_conga.controller import CongaController, ControllerSingularityError
from robot_conga.geometry import wrap_to_pi
from robot_conga.models import Pose, Twist


class FollowerNode(Node):
    """ROS 2 node for a follower robot in the Robot Conga line.

    Configured for 4-robot conga line (1 leader + 3 followers) with default 1.0 m spacing.
    Tracks spatial arc-length offset:
        s_i = s_L - i * delta_s

    Subscribes to:
      - odom (nav_msgs/Odometry): Follower odometry
      - /conga/path (nav_msgs/Path): Shared leader trajectory waypoints
      - /conga/leader_state (geometry_msgs/Vector3): Leader progress (x=s_L, y=u_L, z=omega_L)

    Publishes to:
      - cmd_vel (geometry_msgs/Twist): Commanded unicycle velocity inputs (u, omega)
    """

    def __init__(self) -> None:
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 (rclpy, nav_msgs, geometry_msgs) is required to run FollowerNode.")
        super().__init__('follower_node')

        # Parameters
        self.declare_parameter('follower_index', 1)
        self.declare_parameter('delta_s', 1.0)
        self.declare_parameter('lambda1', 4.5)
        self.declare_parameter('lambda2', 7.5)
        self.declare_parameter('lambda3', 2.5)
        self.declare_parameter('max_linear_velocity', 0.30)
        self.declare_parameter('max_angular_velocity', 1.50)
        self.declare_parameter('singularity_threshold', 1e-3)

        self.idx = int(self.get_parameter('follower_index').value)
        self.delta_s = float(self.get_parameter('delta_s').value)
        l1 = float(self.get_parameter('lambda1').value)
        l2 = float(self.get_parameter('lambda2').value)
        l3 = float(self.get_parameter('lambda3').value)
        v_max = float(self.get_parameter('max_linear_velocity').value)
        w_max = float(self.get_parameter('max_angular_velocity').value)
        sing_thresh = float(self.get_parameter('singularity_threshold').value)

        self.target_offset = self.idx * self.delta_s

        # Initialize RobotConga controller with Paper Eq. 8 and singularity guard
        self.controller = CongaController(
            lambda1=l1,
            lambda2=l2,
            lambda3=l3,
            singularity_threshold=sing_thresh,
            max_linear_velocity=v_max,
            max_angular_velocity=w_max,
        )
        self.path_manager = PathManager()

        # State variables
        self.current_pose: Optional[Pose] = None
        self.leader_s: float = 0.0
        self.leader_u: float = 0.0
        self.num_path_points: int = 0

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10,
        )
        self.path_sub = self.create_subscription(
            NavPath,
            '/conga/path',
            self.path_callback,
            10,
        )
        self.leader_state_sub = self.create_subscription(
            Vector3,
            '/conga/leader_state',
            self.leader_state_callback,
            10,
        )

        # Publisher
        self.cmd_pub = self.create_publisher(RosTwist, 'cmd_vel', 10)

        # 50 Hz control loop timer
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(
            f"Robot Conga Follower {self.idx} initialized with spatial offset {self.target_offset:.2f} m."
        )

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = wrap_to_pi(math.atan2(siny_cosp, cosy_cosp))
        self.current_pose = Pose(x=float(p.x), y=float(p.y), theta=theta)

    def path_callback(self, msg: NavPath) -> None:
        pts = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        if len(pts) != self.num_path_points and len(pts) >= 4:
            if self.path_manager.update_path(pts):
                self.num_path_points = len(pts)

    def leader_state_callback(self, msg: Vector3) -> None:
        self.leader_s = float(msg.x)
        self.leader_u = float(msg.y)

    def control_loop(self) -> None:
        if self.current_pose is None or self.num_path_points < 4:
            return

        target_s = self.leader_s - self.target_offset

        # Follower waits until the leader has advanced past its spatial offset
        if target_s <= 0.0:
            self.publish_cmd(0.0, 0.0)
            return

        ref_state = self.path_manager.get_reference_state(target_s, self.leader_u)
        if ref_state is None:
            return

        try:
            cmd = self.controller.compute(self.current_pose, ref_state)
            self.publish_cmd(cmd.linear, cmd.angular)
        except ControllerSingularityError as e:
            self.get_logger().warn(f"Singularity detected: {e}. Halting linear motion for safety.")
            self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, u: float, omega: float) -> None:
        msg = RosTwist()
        msg.linear.x = float(u)
        msg.angular.z = float(omega)
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    if not HAS_ROS2:
        raise RuntimeError("ROS 2 (rclpy, nav_msgs, geometry_msgs) is required to run FollowerNode.")
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

