"""Run Robot Conga on real TurtleBot 4s (ROS 2 Jazzy). No simulation is started.

Prerequisites: robots tb4_0 .. tb4_{N-1} are up (namespaced), same ROS_DOMAIN_ID, clocks
synced (chrony). Place them, measure each start pose in your world frame, then launch:

  ros2 launch robot_conga conga_hardware_launch.py num_robots:=3 delta_s:=1.0 \\
      start_poses:='0,0,0;-1,0,0;-2,0,0' cmd_vel_type:=TwistStamped

tb4_0 is the leader: drive it with teleop on /tb4_0/cmd_vel. Followers start on the path
(startup prefix), so line them up behind the leader along its heading, d apart.
The robots must not move between placement and launch: odometry is zeroed at node start.
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from robot_conga.launch_common import LAUNCH_ARGS, conga_nodes


def generate_launch_description():
    return LaunchDescription(LAUNCH_ARGS + [OpaqueFunction(function=conga_nodes)])
