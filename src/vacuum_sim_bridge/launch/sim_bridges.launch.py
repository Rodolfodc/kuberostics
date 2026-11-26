"""Starts the four CoppeliaSim bridges for the combined model. Same executable and node names as the stock
IA368II bridges (my_robot_pkg), so start either this launch or the stock nodes, never both. Needs the simulator
running with the GUI (the wheel signals are consumed by a script that needs it) and ZMQ on port 23000.

Arguments: odom_source (ground_truth|wheels), params_file (YAML with the robot profile overrides)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    odom_source = LaunchConfiguration('odom_source')
    params_file = LaunchConfiguration('params_file')

    def bridge(executable, extra=None, remappings=None):
        return Node(package='vacuum_sim_bridge', executable=executable, name=executable, output='screen',
                    parameters=[params_file] + (extra or []), remappings=remappings or [])

    return LaunchDescription([
        DeclareLaunchArgument('odom_source', default_value='ground_truth'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([FindPackageShare('vacuum_sim_bridge'), 'config',
                                                                  'sim_bridge.yaml']),
                              description='YAML with the robot profile (default: the documented defaults)'),
        bridge('my_robot_scan_node'),
        bridge('my_robot_vision_node'),
        bridge('my_robot_cmd_vel_node', remappings=[('/cmd_vel', '/myRobot/cmd_vel')]),
        bridge('my_robot_odometry_node', extra=[{'odom_source': odom_source}]),
    ])
