"""skeleton.launch.py (spec 001): brings up the whole skeleton with one command (001-R01).

    ros2 launch vacuum_bringup skeleton.launch.py [with_ia368ii:=false] [with_wheel_bridge:=false]
                                                  [with_teleop:=false] [with_sequencer:=true]
                                                  [nav:=true] [real_coverage:=true] [sim_slowdown:=11]
                                                  [wheel_bridge:=vacuum] [map_tf:=true] [map_source:=static]

Blocks, each built by its own function:
  - IA368II nodes + their lifecycle manager   (with_ia368ii, default true: 001-R01 asks for them)
  - our nodes + lifecycle_manager_core        (always)
  - twist_mux                                 (always; the ONLY publisher of /myRobot/cmd_vel, Art. VI)
  - wheel bridge                              (needs CoppeliaSim): wheel_bridge:=stock (default) is the 2025S2 node
                                                of my_robot_pkg (D-06; with_ia368ii and with_wheel_bridge), wheel_bridge:=vacuum
                                                is the same-name replacement of vacuum_sim_bridge for the combined model
                                                (D-27; with_wheel_bridge only, it does not need the IA368II checkout)
  - nav.launch.py                            (nav, default false): Nav2 under lifecycle_manager_nav; sim_slowdown is
                                                forwarded to it and to the real coverage launch (real_coverage)
  - map_tf.launch.py                         (map_tf, default false): static map -> my_robot_odom (dead reckoning only);
                                                map_source:=slam is a placeholder that fails (slam_toolbox not integrated)
  - teleop_twist_keyboard                     (with_teleop; needs a terminal on stdin)
  - lifecycle_sequencer + nav manager (with_sequencer, default false): the managers that have nodes start in
    order core -> nav (005-R35; optional is not launched while it has no nodes, see sequencer_nodes()); lifecycle_manager_core then runs with autostart false

The IA368II packages are not in rosdep (they come from `vcs import`, Art. II), so everything that references
them is conditional and resolved lazily: this file imports and builds without them, and the launch_testing
the live-graph tests run with `with_ia368ii:=false` or against the installed checkout.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
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
        DeclareLaunchArgument('wheel_bridge', default_value='stock', choices=['stock', 'vacuum'],
                              description='Which wheel bridge with_wheel_bridge starts: the 2025S2 one (stock) or '
                                          'vacuum_sim_bridge (vacuum, D-27); only one runs, they share the node name'),
        DeclareLaunchArgument('with_teleop', default_value='true',
                              description='Start teleop_twist_keyboard (needs a terminal)'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Include nav.launch.py (Nav2 under lifecycle_manager_nav, 005-R35); needs '
                                          '/map and the map -> odom TF from outside'),
        DeclareLaunchArgument('map_tf', default_value='false',
                              description='Include map_tf.launch.py (map -> my_robot_odom, what nav needs)'),
        DeclareLaunchArgument('map_source', default_value='static', choices=['static', 'slam'],
                              description='Source of map -> my_robot_odom when map_tf is true (slam: not integrated)'),
        DeclareLaunchArgument('real_coverage', default_value='false',
                              description='Run the real coverage_server of vacuum_coverage instead of the stub'),
        DeclareLaunchArgument('sim_slowdown', default_value='1.0',
                              description='Factor > 0 forwarded to nav and coverage launches: scales their '
                                          'wall-clock timeouts for a slow simulator (1.0 = real time)'),
        DeclareLaunchArgument('with_sequencer', default_value='false',
                              description='Start lifecycle_sequencer (plus lifecycle_manager_nav with nav:=true) '
                                          'and let the sequencer (not autostart) start the managers'),
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


def core_nodes(stub_coverage):
    nodes = [Node(package=package, executable=executable, name=executable, output='screen',
                  condition=stub_coverage if executable == 'coverage_server' else None)
             for package, executable in CORE_NODES]
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_core',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('vacuum_bringup'), 'config',
                                          'lifecycle_core.yaml']),
                    # the YAML says autostart true (stand-alone use); with the sequencer it must be false
                    {'autostart': ParameterValue(PythonExpression(
                        ["'", LaunchConfiguration('with_sequencer'), "' != 'true'"]), value_type=bool)}]))
    return nodes


def sequencer_nodes(nav_condition, no_nav_condition):
    """The sequencer and the managers it starts (autostart false, in their YAML).

    A Nav2 lifecycle_manager with `node_names: []` ABORTS at startup (exit -6: the empty YAML array has no
    type), so a manager without nodes is never launched and never listed: lifecycle_manager_optional (cloud and
    MQTT nodes, spec 009) is not launched until it has nodes, and lifecycle_manager_nav only with nav:=true. The
    sequencer's `managers` parameter is overridden to match (ManagerSequencer also skips a live manager that
    reports no nodes, but that cannot happen with a Nav2 manager)."""
    def config(name):
        return PathJoinSubstitution([FindPackageShare('vacuum_bringup'), 'config', name])

    def sequencer(managers, cond):
        return Node(package='vacuum_bringup', executable='lifecycle_sequencer', name='lifecycle_sequencer',
                    output='screen', parameters=[config('lifecycle_sequencer.yaml'), {'managers': managers}],
                    condition=cond)
    return [Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_nav',
                 output='screen', parameters=[config('lifecycle_nav.yaml')], condition=nav_condition),
            sequencer(['lifecycle_manager_core', 'lifecycle_manager_nav'], nav_condition),
            sequencer(['lifecycle_manager_core'], no_nav_condition)]


def nav_include(condition):
    """Nav2 (005-R35): controller/planner/bt_navigator from nav.launch.py. With the sequencer the nav manager of
    sequencer_nodes() (config/lifecycle_nav.yaml, same node list) starts them, so the include starts none."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('vacuum_bringup'), 'launch',
                                                            'nav.launch.py'])),
        launch_arguments={
            'with_manager': PythonExpression(["'", LaunchConfiguration('with_sequencer'), "' != 'true'"]),
            'autostart': 'true',
            'sim_slowdown': LaunchConfiguration('sim_slowdown'),
        }.items(), condition=condition)


def coverage_include(condition):
    """The real coverage_server in place of the stub (same node name, so lifecycle_manager_core starts it)."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('vacuum_coverage'), 'launch',
                                                            'coverage.launch.py'])),
        launch_arguments={'autostart': 'false', 'sim_slowdown': LaunchConfiguration('sim_slowdown')}.items(),
        condition=condition)


def map_tf_include(condition):
    source = os.path.join(get_package_share_directory('vacuum_bringup'), 'launch', 'map_tf.launch.py')
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(source),
                                    launch_arguments={'map_source': LaunchConfiguration('map_source')}.items(),
                                    condition=condition)


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


def vacuum_wheel_bridge_node(condition):
    # D-27: same executable and node name as the stock bridge, so only one of the two may run.
    return Node(package='vacuum_sim_bridge', executable='my_robot_cmd_vel_node', name='my_robot_cmd_vel_node',
                output='screen', remappings=[('/cmd_vel', CMD_VEL)], condition=condition,
                parameters=[PathJoinSubstitution([FindPackageShare('vacuum_sim_bridge'), 'config',
                                                  'sim_bridge.yaml'])])


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
         LaunchConfiguration('with_wheel_bridge'), "' == 'true' and '",
         LaunchConfiguration('wheel_bridge'), "' == 'stock'"]))
    with_vacuum_bridge = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('with_wheel_bridge'), "' == 'true' and '",
         LaunchConfiguration('wheel_bridge'), "' == 'vacuum'"]))
    with_teleop = IfCondition(LaunchConfiguration('with_teleop'))
    with_sequencer = IfCondition(LaunchConfiguration('with_sequencer'))
    with_nav = IfCondition(LaunchConfiguration('nav'))
    seq_and_nav = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('with_sequencer'), "' == 'true' and '", LaunchConfiguration('nav'), "' == 'true'"]))
    seq_no_nav = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('with_sequencer'), "' == 'true' and '", LaunchConfiguration('nav'), "' != 'true'"]))
    real_coverage = IfCondition(LaunchConfiguration('real_coverage'))
    stub_coverage = UnlessCondition(LaunchConfiguration('real_coverage'))
    return LaunchDescription([
        *launch_arguments(),
        *ia368ii_nodes(with_ia368ii),
        *core_nodes(stub_coverage),
        *sequencer_nodes(seq_and_nav, seq_no_nav),
        nav_include(with_nav),
        coverage_include(real_coverage),
        map_tf_include(IfCondition(LaunchConfiguration('map_tf'))),
        twist_mux_node(),
        wheel_bridge_node(with_bridge),
        vacuum_wheel_bridge_node(with_vacuum_bridge),
        teleop_node(with_teleop),
    ])
