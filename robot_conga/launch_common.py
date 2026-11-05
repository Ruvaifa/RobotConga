"""Launch helpers shared by the gz simulation and real-hardware launch files."""

import math
from typing import List, Tuple

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

LAUNCH_ARGS = [
    DeclareLaunchArgument("num_robots", default_value="3", description="Robots in the line: tb4_0 (leader) .. tb4_{N-1}"),
    DeclareLaunchArgument("delta_s", default_value="1.0", description="Inter-robot spacing along the path (m)"),
    DeclareLaunchArgument("cmd_vel_type", default_value="TwistStamped", choices=["Twist", "TwistStamped"],
                          description="Check with: ros2 topic info -v /tb4_1/cmd_vel"),
    DeclareLaunchArgument("pose_source", default_value="odom", choices=["odom", "topic"],
                          description="odom + static world->tb4_i/odom TF, or a world-frame PoseStamped topic"),
    DeclareLaunchArgument("start_poses", default_value="",
                          description="World start poses 'x,y,yaw_deg;x,y,yaw_deg;...' for tb4_0..; "
                                      "default: lined up along -x at spacing delta_s"),
    DeclareLaunchArgument("use_estop", default_value="true", choices=["true", "false"]),
    DeclareLaunchArgument("log", default_value="true", choices=["true", "false"], description="Run the CSV logger"),
    DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
]


def start_poses(spec: str, num_robots: int, delta_s: float) -> List[Tuple[float, float, float]]:
    """Parse 'x,y,yaw_deg;...' into (x, y, yaw_rad); default is a line behind the leader along -x."""
    if not spec.strip():
        return [(-i * delta_s, 0.0, 0.0) for i in range(num_robots)]
    poses = []
    for item in spec.split(";"):
        x, y, yaw_deg = (float(v) for v in item.split(","))
        poses.append((x, y, math.radians(yaw_deg)))
    if len(poses) != num_robots:
        raise ValueError(f"start_poses has {len(poses)} entries, expected {num_robots}")
    return poses


def conga_nodes(context) -> list:
    """Leader + followers + static TFs (+ logger) from the resolved launch arguments."""
    arg = lambda name: context.launch_configurations[name]  # noqa: E731
    n = int(arg("num_robots"))
    d = float(arg("delta_s"))
    poses = start_poses(arg("start_poses"), n, d)
    sim_time = arg("use_sim_time") == "true"
    common = {
        "num_robots": n,
        "delta_s": d,
        "cmd_vel_type": arg("cmd_vel_type"),
        "pose_source": arg("pose_source"),
        "use_sim_time": sim_time,
    }
    actions = []
    for i, (x, y, yaw) in enumerate(poses):
        ns = f"tb4_{i}"
        if arg("pose_source") == "odom":
            actions.append(Node(
                package="tf2_ros", executable="static_transform_publisher", name=f"world_to_{ns}_odom",
                arguments=["--x", str(x), "--y", str(y), "--yaw", str(yaw),
                           "--frame-id", "world", "--child-frame-id", f"{ns}/odom"],
                parameters=[{"use_sim_time": sim_time}],
            ))
        if i == 0:
            actions.append(Node(package="robot_conga", executable="leader_node", name="leader_node",
                                namespace=ns, parameters=[common], output="screen"))
        else:
            actions.append(Node(
                package="robot_conga", executable="follower_node", name="follower_node", namespace=ns,
                parameters=[{**common, "follower_index": i, "use_estop": arg("use_estop") == "true"}],
                output="screen",
            ))
    if arg("log") == "true":
        actions.append(Node(package="robot_conga", executable="logger_node", name="conga_logger",
                            parameters=[{"num_robots": n, "cmd_vel_type": arg("cmd_vel_type"),
                                         "use_sim_time": sim_time}],
                            output="screen"))
    return actions
