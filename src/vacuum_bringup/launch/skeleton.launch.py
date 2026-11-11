"""skeleton.launch.py (T013, spec 001): brings up the whole skeleton with one command (001-R01).

    ros2 launch vacuum_bringup skeleton.launch.py [with_ia368ii:=false] [with_wheel_bridge:=false]
                                                  [with_teleop:=false]

Blocks, each built by its own function:
  - IA368II nodes + their lifecycle manager   (with_ia368ii, default true: 001-R01 asks for them)
  - our nodes + lifecycle_manager_core        (always)
  - twist_mux                                 (always; the ONLY publisher of /myRobot/cmd_vel, Art. VI)
  - 2025S2 wheel bridge                       (with_ia368ii and with_wheel_bridge; needs CoppeliaSim, D-06)
  - teleop_twist_keyboard                     (with_teleop; needs a terminal on stdin)

The IA368II packages are not in rosdep (they come from `vcs import`, Art. II), so everything that references
them is conditional and resolved lazily: this file imports and builds without them, and the launch_testing
tests of T014 run with `with_ia368ii:=false` or against the installed checkout.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CMD_VEL = '/myRobot/cmd_vel'

# (executable, node name). The IA368II launch names the myrobot_server node `myrobot`, and that is the
# name its lifecycle_manager.yaml lists in node_names.
IA368II_NODES = (
    ('myrobot_server', 'myrobot'),
    ('behaviour_server', 'behaviour_server'),
    ('cleaning_server', 'cleaning_server'),
    ('autodocking_server', 'autodocking_server'),
    ('charging_server', 'charging_server'),
    ('charged_server', 'charged_server'),
)

# Remappings that adapt IA368II without touching it (Art. II). Both nodes publish /myRobot/cmd_vel by default.
IA368II_REMAPS = {
    'cleaning_server': [(CMD_VEL, '/myRobot/legacy/cleaning_cmd_vel')],    # D-04: sink, nobody subscribes
    'autodocking_server': [(CMD_VEL, '/myRobot/mux/docking')],             # D-05
}

# (package, executable): our nodes, all LifecycleNodes under lifecycle_manager_core.
CORE_NODES = (
    ('vacuum_safety', 'stop_guard'),
    ('vacuum_stubs', 'coverage_server'),
    ('vacuum_stubs', 'escape_server'),
    ('vacuum_stubs', 'stuck_detector'),
    ('vacuum_supervisor', 'supervisor'),
)


def launch_arguments():
    return [
        DeclareLaunchArgument('with_ia368ii', default_value='true',
                              description='Start the IA368II nodes and their lifecycle manager'),
        DeclareLaunchArgument('with_wheel_bridge', default_value='true',
                              description='Start the 2025S2 wheel bridge (needs CoppeliaSim); '
                                          'ignored when with_ia368ii is false'),
        DeclareLaunchArgument('with_teleop', default_value='true',
                              description='Start teleop_twist_keyboard (needs a terminal)'),
    ]


def ia368ii_nodes(condition):
    nodes = []
    for executable, name in IA368II_NODES:
        nodes.append(Node(package='myrobot_bringup', executable=executable, name=name, output='screen',
                          remappings=IA368II_REMAPS.get(executable, []), condition=condition))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
        output='screen', condition=condition,
        parameters=[PathJoinSubstitution([FindPackageShare('myrobot_bringup'), 'config',
                                          'lifecycle_manager.yaml'])]))
    return nodes


def core_nodes():
    nodes = [Node(package=package, executable=executable, name=executable, output='screen')
             for package, executable in CORE_NODES]
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_core',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('vacuum_bringup'), 'config',
                                          'lifecycle_core.yaml'])]))
    return nodes


def twist_mux_node():
    # use_stamped: false lives in twist_mux.yaml (D-13). While lock_stop is on the mux publishes nothing,
    # stop_guard feeds the `stop` input instead (D-22).
    return Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('vacuum_bringup'), 'config', 'twist_mux.yaml'])],
        remappings=[('cmd_vel_out', CMD_VEL)])


def wheel_bridge_node(condition):
    # D-06: reused unchanged; it subscribes to /cmd_vel, remapped to the mux output.
    return Node(package='my_robot_pkg', executable='my_robot_cmd_vel_node', name='my_robot_cmd_vel_node',
                output='screen', remappings=[('/cmd_vel', CMD_VEL)], condition=condition)


def teleop_node(condition):
    # D-13: stamped is set explicitly; its default has changed between teleop_twist_keyboard releases.
    return Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard', name='teleop_keyboard',
                output='screen', parameters=[{'stamped': False}],
                remappings=[('cmd_vel', '/myRobot/mux/teleop')], condition=condition)


def generate_launch_description():
    with_ia368ii = IfCondition(LaunchConfiguration('with_ia368ii'))
    # IfCondition has no `and`: evaluate both flags in one expression
    with_bridge = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('with_ia368ii'), "' == 'true' and '",
         LaunchConfiguration('with_wheel_bridge'), "' == 'true'"]))
    with_teleop = IfCondition(LaunchConfiguration('with_teleop'))
    return LaunchDescription([
        *launch_arguments(),
        *ia368ii_nodes(with_ia368ii),
        *core_nodes(),
        twist_mux_node(),
        wheel_bridge_node(with_bridge),
        teleop_node(with_teleop),
    ])
