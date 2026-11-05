"""Robot Conga in Gazebo (gz) with TurtleBot 4 on ROS 2 Jazzy.

Starts the turtlebot4_gz_bringup world, spawns tb4_0 .. tb4_{N-1} at their start poses,
and runs the same Conga nodes as conga_hardware_launch.py (with use_sim_time).

  ros2 launch robot_conga conga_launch.py num_robots:=3 delta_s:=1.0

Drive the leader:  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \\
                       -r cmd_vel:=/tb4_0/cmd_vel -p stamped:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from robot_conga.launch_common import LAUNCH_ARGS, conga_nodes, start_poses

GZ_LAUNCH = FindPackageShare("turtlebot4_gz_bringup")


def spawn_robots(context):
    arg = lambda name: context.launch_configurations[name]  # noqa: E731
    n, d = int(arg("num_robots")), float(arg("delta_s"))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([GZ_LAUNCH, "launch", "turtlebot4_spawn.launch.py"])),
            launch_arguments={"namespace": f"tb4_{i}", "x": str(x), "y": str(y), "z": "0.0",
                              "yaw": str(yaw), "model": arg("model")}.items(),
        )
        for i, (x, y, yaw) in enumerate(start_poses(arg("start_poses"), n, d))
    ]


def generate_launch_description():
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([GZ_LAUNCH, "launch", "sim.launch.py"])),
        launch_arguments={"world": LaunchConfiguration("world"), "model": LaunchConfiguration("model")}.items(),
    )
    return LaunchDescription(LAUNCH_ARGS + [
        DeclareLaunchArgument("world", default_value="warehouse"),
        DeclareLaunchArgument("model", default_value="lite", choices=["standard", "lite"]),
        SetLaunchConfiguration("use_sim_time", "true"),
        world,
        OpaqueFunction(function=spawn_robots),
        OpaqueFunction(function=conga_nodes),
    ])
