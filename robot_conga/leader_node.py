"""ROS 2 Leader Node for Robot Conga sequential path following."""

import math
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path as NavPath
    from geometry_msgs.msg import PoseStamped, Vector3
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object  # type: ignore


class LeaderNode(Node):
    """ROS 2 node tracking leader progress and publishing path and state for followers.

    The leader tracks its own continuous arc length s_L from odometry and publishes:
      - /conga/path (nav_msgs/Path): Cumulative sequence of waypoints along the trajectory.
      - /conga/leader_state (geometry_msgs/Vector3):
          x = leader arc length s_L (m)
          y = leader forward linear speed u_L (m/s)
          z = leader angular velocity omega_L (rad/s)
    """

    def __init__(self) -> None:
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 (rclpy, nav_msgs, geometry_msgs) is required to run LeaderNode.")
        super().__init__('leader_node')

        # Parameters
        self.declare_parameter('min_dist', 0.05)  # min distance (m) to record new waypoint
        self.min_dist = float(self.get_parameter('min_dist').value)

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10,
        )

        # Publishers
        self.path_pub = self.create_publisher(NavPath, '/conga/path', 10)
        self.state_pub = self.create_publisher(Vector3, '/conga/leader_state', 10)

        # Internal state
        self.path_msg = NavPath()
        self.path_msg.header.frame_id = 'odom'
        self.last_pose = None
        self.arc_length: float = 0.0
        self.current_v: float = 0.0
        self.current_w: float = 0.0

        self.get_logger().info("Robot Conga LeaderNode initialized.")

    def odom_callback(self, msg: Odometry) -> None:
        current_pose = msg.pose.pose
        self.current_v = float(msg.twist.twist.linear.x)
        self.current_w = float(msg.twist.twist.angular.z)

        if self.last_pose is None:
            self.last_pose = current_pose
            self.add_pose_to_path(current_pose)
            return

        dx = float(current_pose.position.x - self.last_pose.position.x)
        dy = float(current_pose.position.y - self.last_pose.position.y)
        dist = math.sqrt(dx * dx + dy * dy)

        if dist >= self.min_dist:
            self.arc_length += dist
            self.last_pose = current_pose
            self.add_pose_to_path(current_pose)
            self.path_pub.publish(self.path_msg)

        # Publish state (x = s_L, y = u_L, z = omega_L)
        state_msg = Vector3()
        state_msg.x = float(self.arc_length)
        state_msg.y = float(self.current_v)
        state_msg.z = float(self.current_w)
        self.state_pub.publish(state_msg)

    def add_pose_to_path(self, pose) -> None:
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'odom'
        ps.pose = pose
        self.path_msg.poses.append(ps)


def main(args=None) -> None:
    if not HAS_ROS2:
        raise RuntimeError("ROS 2 (rclpy, nav_msgs, geometry_msgs) is required to run LeaderNode.")
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

