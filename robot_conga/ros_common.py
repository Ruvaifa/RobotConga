"""Shared ROS 2 helpers: world-frame pose source, cmd_vel publisher (Twist or TwistStamped)."""

import math
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener

from robot_conga.geometry import wrap_to_pi
from robot_conga.models import Pose

MAX_SAFE_SPEED = 0.25  # m/s, hard cap for every robot in this project


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_yaw(q, yaw: float) -> None:
    q.x, q.y, q.z, q.w = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def compose(a: Pose, b: Pose) -> Pose:
    """2D pose composition a * b (b expressed in a's frame)."""
    c, s = math.cos(a.theta), math.sin(a.theta)
    return Pose(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap_to_pi(a.theta + b.theta))


def inverse(a: Pose) -> Pose:
    c, s = math.cos(a.theta), math.sin(a.theta)
    return Pose(-c * a.x - s * a.y, s * a.x - c * a.y, wrap_to_pi(-a.theta))


def to_pose_stamped(node: Node, pose: Pose, frame_id: str, z: float = 0.0) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose.x, pose.y, float(z)
    set_yaw(msg.pose.orientation, pose.theta)
    return msg


class WorldPoseSource:
    """Robot pose in the shared world frame, from one of two sources (parameter `pose_source`):

    - "odom":  the robot's odometry, mapped into the world by a static TF world -> <ns>/odom
               (publish it from the hand-measured start pose; see conga_hardware_launch.py).
               With zero_odom_at_start, odometry is re-zeroed at the first message, so the
               static TF only has to equal the robot's measured pose when the nodes start.
    - "topic": a geometry_msgs/PoseStamped topic (e.g. an overhead-camera tracker). If its
               frame_id is not the world frame it is transformed with tf2.

    `latest` is the newest world pose; `stamp` is when it arrived (node clock) for watchdogs.
    """

    def __init__(self, node: Node) -> None:
        self.node = node
        ns = node.get_namespace().strip("/")
        node.declare_parameter("pose_source", "odom")
        node.declare_parameter("world_frame", "world")
        node.declare_parameter("odom_frame", f"{ns}/odom" if ns else "odom")
        node.declare_parameter("pose_topic", "world_pose")
        node.declare_parameter("zero_odom_at_start", True)
        self.mode = node.get_parameter("pose_source").value
        self.world_frame = node.get_parameter("world_frame").value
        self.odom_frame = node.get_parameter("odom_frame").value
        self.zero_odom = bool(node.get_parameter("zero_odom_at_start").value)

        self.latest: Optional[Pose] = None
        self.stamp: Optional[Time] = None
        self._odom0: Optional[Pose] = None
        self._tf_cache = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        if self.mode == "odom":
            node.create_subscription(Odometry, "odom", self._on_odom, qos_profile_sensor_data)
        elif self.mode == "topic":
            topic = node.get_parameter("pose_topic").value
            node.create_subscription(PoseStamped, topic, self._on_pose, qos_profile_sensor_data)
        else:
            raise ValueError(f"pose_source must be 'odom' or 'topic', got {self.mode!r}")

    def _world_from(self, frame: str) -> Optional[Pose]:
        """Transform world <- frame. Only static transforms are expected, so cache them."""
        if frame == self.world_frame:
            return Pose(0.0, 0.0, 0.0)
        if frame not in self._tf_cache:
            try:
                tf = self.tf_buffer.lookup_transform(self.world_frame, frame, Time()).transform
            except TransformException as e:
                self.node.get_logger().warn(f"No TF {self.world_frame} <- {frame} yet: {e}", throttle_duration_sec=2.0)
                return None
            self._tf_cache[frame] = Pose(tf.translation.x, tf.translation.y, yaw_from_quaternion(tf.rotation))
        return self._tf_cache[frame]

    def _store(self, frame: str, local: Pose) -> None:
        world_t_frame = self._world_from(frame)
        if world_t_frame is not None:
            self.latest = compose(world_t_frame, local)
            self.stamp = self.node.get_clock().now()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        odom = Pose(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        if self.zero_odom:
            if self._odom0 is None:
                self._odom0 = odom
            odom = compose(inverse(self._odom0), odom)
        self._store(self.odom_frame, odom)

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose
        self._store(msg.header.frame_id, Pose(p.position.x, p.position.y, yaw_from_quaternion(p.orientation)))

    def age(self) -> float:
        """Seconds since the last pose arrived (inf if none yet)."""
        if self.stamp is None:
            return math.inf
        return (self.node.get_clock().now() - self.stamp).nanoseconds * 1e-9


def cmd_vel_type(name: str):
    if name not in ("Twist", "TwistStamped"):
        raise ValueError(f"cmd_vel_type must be 'Twist' or 'TwistStamped', got {name!r}")
    return Twist if name == "Twist" else TwistStamped


def make_cmd_vel_publisher(node: Node, type_name: str, topic: str = "cmd_vel") -> Callable[[float, float], None]:
    """Return publish(u, omega) for the configured cmd_vel type, hard-capped at MAX_SAFE_SPEED."""
    msg_type = cmd_vel_type(type_name)
    pub = node.create_publisher(msg_type, topic, 10)

    def publish(u: float, omega: float) -> None:
        msg = msg_type()
        twist = msg if msg_type is Twist else msg.twist
        if msg_type is TwistStamped:
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
        twist.linear.x = max(-MAX_SAFE_SPEED, min(MAX_SAFE_SPEED, float(u)))
        twist.angular.z = float(omega)
        pub.publish(msg)

    return publish


def twist_from_cmd_msg(msg) -> "tuple[float, float]":
    twist = msg.twist if isinstance(msg, TwistStamped) else msg
    return float(twist.linear.x), float(twist.angular.z)


def spin(node_factory) -> None:
    rclpy.init()
    node = node_factory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "on_shutdown") and rclpy.ok():
            node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
