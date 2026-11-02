"""Launch multi-agent Robot Conga sequential path following in Ignition Gazebo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments
    # Robot Conga paper experiment specifies 4 robots (1 leader + 3 followers) at 1.0 m spacing
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='4',
        description='Total number of robots in the conga line (1 leader + followers)',
    )
    delta_s_arg = DeclareLaunchArgument(
        'delta_s',
        default_value='1.0',
        description='Desired inter-robot spatial spacing along path (meters)',
    )
    ld.add_action(num_robots_arg)
    ld.add_action(delta_s_arg)

    tb4_ign_dir = FindPackageShare('turtlebot4_ignition_bringup')

    # Launch Ignition Gazebo environment
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_ign_dir, 'launch', 'ignition.launch.py'])
        )
    )
    ld.add_action(ign_gazebo)

    # 4 Robots: 1 Leader (robot0) + 3 Followers (robot1, robot2, robot3)
    num_robots = 4
    delta_s = 1.0

    for i in range(num_robots):
        namespace = f'robot{i}'

        # Spawn TurtleBot4 instances lined up behind each other (x = 0.0, -1.0, -2.0, -3.0)
        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([tb4_ign_dir, 'launch', 'robot_spawn.launch.py'])
            ),
            launch_arguments={
                'namespace': namespace,
                'x': str(-i * delta_s),
                'y': '0.0',
                'z': '0.1',
            }.items(),
        )
        ld.add_action(spawn_robot)

        # Launch Leader node for robot0, Follower nodes for robot1..robot3
        if i == 0:
            leader_node = Node(
                package='robot_conga',
                executable='leader_node',
                name='leader_node',
                namespace=namespace,
                output='screen',
                remappings=[
                    ('/conga/path', '/conga/path'),
                    ('/conga/leader_state', '/conga/leader_state'),
                ],
            )
            ld.add_action(leader_node)
        else:
            follower_node = Node(
                package='robot_conga',
                executable='follower_node',
                name=f'follower_node_{i}',
                namespace=namespace,
                parameters=[{
                    'follower_index': i,
                    'delta_s': delta_s,
                    'lambda1': 4.5,
                    'lambda2': 7.5,
                    'lambda3': 2.5,
                    'max_linear_velocity': 0.30,
                    'max_angular_velocity': 1.50,
                    'singularity_threshold': 1e-3,
                }],
                output='screen',
                remappings=[
                    ('/conga/path', '/conga/path'),
                    ('/conga/leader_state', '/conga/leader_state'),
                ],
            )
            ld.add_action(follower_node)

    return ld

