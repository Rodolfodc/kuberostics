"""map_tf.launch.py: publishes the `map -> my_robot_odom` transform (spec 003-R05, contract section `tf`).

    ros2 launch vacuum_bringup map_tf.launch.py [map_source:=static|slam]

Only the sim bridge publishes `my_robot_odom -> base_footprint` (plus the static base_link/laser/camera frames),
so without this launch nothing connects `map` to the robot.

  - static (default): identity static transform map -> my_robot_odom. The map frame is then a dead-reckoning-only
    frame: it drifts with the odometry, nothing corrects it. This is the documented fallback while slam_toolbox
    is not integrated.
  - slam: placeholder. slam_toolbox is NOT integrated; the launch fails at startup with a clear message instead of
    silently running without a map frame.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAP_FRAME = 'map'
ODOM_FRAME = 'my_robot_odom'      # the 2025S2 name, also used by nav2_params.yaml (contract `tf` section)


def static_map_tf_node():
    # named flags: the positional form is deprecated, and the flags exist on both Jazzy and Lyrical
    return Node(package='tf2_ros', executable='static_transform_publisher', name='map_tf_static', output='screen',
                arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                           '--frame-id', MAP_FRAME, '--child-frame-id', ODOM_FRAME])


def _select_source(context):
    source = LaunchConfiguration('map_source').perform(context)
    if source == 'static':
        return [static_map_tf_node()]
    if source == 'slam':
        raise RuntimeError('map_source:=slam is not available: slam_toolbox is not integrated yet. '
                           'Use map_source:=static (identity map -> my_robot_odom, dead-reckoning only).')
    raise RuntimeError(f"map_source must be 'static' or 'slam', got '{source}'")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_source', default_value='static',
                              description="static: identity map -> my_robot_odom (dead reckoning only); "
                                          "slam: placeholder, fails (slam_toolbox not integrated)"),
        OpaqueFunction(function=_select_source),
    ])
