"""ROS 2 data logger for Robot Conga runs: one CSV row per tick with every robot's world pose,
reference pose, s_i, commanded twist and the adjacent inter-robot distances.

Reads /<prefix><i>/conga/pose, /<prefix><i>/conga/reference (z = s_i) and /<prefix><i>/cmd_vel.
Analyse the CSV with:  python scripts/analyze_log.py conga_log.csv --spacing 1.0
"""

import csv
import math
import os
import time

try:
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

    from robot_conga.ros_common import cmd_vel_type, spin, twist_from_cmd_msg, yaw_from_quaternion
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object  # type: ignore

from robot_conga.models import Pose, Twist
from robot_conga.transport import csv_header, csv_row


class LoggerNode(Node):
    def __init__(self) -> None:
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 is required to run LoggerNode.")
        super().__init__("conga_logger")
        self.declare_parameter("num_robots", 3)
        self.declare_parameter("namespace_prefix", "tb4_")
        self.declare_parameter("cmd_vel_type", "Twist")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("output", "")  # default: conga_log_<unix time>.csv in cwd
        n = int(self.get_parameter("num_robots").value)
        prefix = self.get_parameter("namespace_prefix").value
        out = self.get_parameter("output").value or f"conga_log_{int(time.time())}.csv"

        self.n = n
        self.pose = [None] * n
        self.ref = [None] * n
        self.s = [math.nan] * n
        self.cmd = [Twist(0.0, 0.0)] * n
        msg_type = cmd_vel_type(self.get_parameter("cmd_vel_type").value)
        for i in range(n):
            ns = f"/{prefix}{i}"
            self.create_subscription(PoseStamped, f"{ns}/conga/pose", lambda m, i=i: self._set_pose(i, m), 10)
            self.create_subscription(PoseStamped, f"{ns}/conga/reference", lambda m, i=i: self._set_ref(i, m), 10)
            self.create_subscription(msg_type, f"{ns}/cmd_vel", lambda m, i=i: self._set_cmd(i, m), 10)

        self.file = open(out, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(csv_header(n))
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.tick)
        self.get_logger().info(f"Logging {n} robots to {os.path.abspath(out)}")

    @staticmethod
    def _to_pose(msg: PoseStamped) -> Pose:
        p = msg.pose
        return Pose(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _set_pose(self, i: int, msg: PoseStamped) -> None:
        self.pose[i] = self._to_pose(msg)

    def _set_ref(self, i: int, msg: PoseStamped) -> None:
        self.ref[i] = self._to_pose(msg)
        self.s[i] = msg.pose.position.z

    def _set_cmd(self, i: int, msg) -> None:
        self.cmd[i] = Twist(*twist_from_cmd_msg(msg))

    def tick(self) -> None:
        if any(p is None for p in self.pose) or any(r is None for r in self.ref):
            return  # wait until every robot has reported once
        t = self.get_clock().now().nanoseconds * 1e-9
        self.writer.writerow(csv_row(t, self.pose, self.ref, self.s, self.cmd))
        self.file.flush()

    def on_shutdown(self) -> None:
        self.file.close()


def main(args=None) -> None:
    if not HAS_ROS2:
        raise RuntimeError("ROS 2 is required to run LoggerNode.")
    spin(LoggerNode)


if __name__ == "__main__":
    main()
