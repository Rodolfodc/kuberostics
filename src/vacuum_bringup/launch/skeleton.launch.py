"""skeleton.launch.py (spec 001): brings up the whole skeleton with one command (001-R01).

    ros2 launch vacuum_bringup skeleton.launch.py [with_ia368ii:=false] [with_wheel_bridge:=false]
                                                  [with_teleop:=false] [with_sequencer:=true]
                                                  [wheel_bridge:=vacuum] [map_tf:=true] [map_source:=static|slam]
                                                  [real_nodes:=true]

Blocks, each built by its own function:
  - IA368II nodes + their lifecycle manager   (with_ia368ii, default true: 001-R01 asks for them)
  - our nodes + lifecycle_manager_core        (always)
  - twist_mux                                 (always; the ONLY publisher of /myRobot/cmd_vel, Art. VI)
  - wheel bridge                              (needs CoppeliaSim): wheel_bridge:=stock (default) is the 2025S2 node
                                                of my_robot_pkg (D-06; with_ia368ii and with_wheel_bridge), wheel_bridge:=vacuum
                                                is the same-name replacement of vacuum_sim_bridge for the combined model
                                                (D-27; with_wheel_bridge only, it does not need the IA368II checkout)
  - map -> my_robot_odom transform            (map_tf, default false; map_tf.launch.py, map_source static|slam)
  - real_nodes (default false): swaps the stubs for the real nodes that exist (today only coverage_server of
    vacuum_coverage; escape_server and stuck_detector have no real counterpart yet and stay the vacuum_stubs ones)
  - teleop_twist_keyboard                     (with_teleop; needs a terminal on stdin)
  - lifecycle_sequencer + nav/optional managers (with_sequencer, default false): the three managers start
    in order core -> nav -> optional (005-R35); lifecycle_manager_core then runs with autostart false

The IA368II packages are not in rosdep (they come from `vcs import`, Art. II), so everything that references
them is conditional and resolved lazily: this file imports and builds without them, and the launch_testing
the live-graph tests run with `with_ia368ii:=false` or against the installed checkout.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        DeclareLaunchArgument('map_tf', default_value='false',
                              description='Include map_tf.launch.py (map -> my_robot_odom)'),
        DeclareLaunchArgument('map_source', default_value='static', choices=['static', 'slam'],
                              description='Source of map -> my_robot_odom when map_tf is true (slam: not integrated)'),
        DeclareLaunchArgument('real_nodes', default_value='false',
                              description='Replace the stubs by the real nodes that exist (coverage_server)'),
        DeclareLaunchArgument('with_sequencer', default_value='false',
                              description='Start lifecycle_sequencer plus lifecycle_manager_nav and '
                                          '_optional, and let the sequencer (not autostart) start the managers'),
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


def coverage_server_nodes():
    """The stub (vacuum_stubs) or the real node (vacuum_coverage): same executable and node name, one runs."""
    real = IfCondition(LaunchConfiguration('real_nodes'))
    stub = UnlessCondition(LaunchConfiguration('real_nodes'))
    return [Node(package=package, executable='coverage_server', name='coverage_server', output='screen',
                 condition=condition)
            for package, condition in (('vacuum_stubs', stub), ('vacuum_coverage', real))]


def map_tf_include():
    source = os.path.join(get_package_share_directory('vacuum_bringup'), 'launch', 'map_tf.launch.py')
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(source),
                                    launch_arguments={'map_source': LaunchConfiguration('map_source')}.items(),
                                    condition=IfCondition(LaunchConfiguration('map_tf')))


def core_nodes():
    nodes = [Node(package=package, executable=executable, name=executable, output='screen')
             for package, executable in CORE_NODES]
    nodes.extend(coverage_server_nodes())
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_core',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('vacuum_bringup'), 'config',
                                          'lifecycle_core.yaml']),
                    # the YAML says autostart true (stand-alone use); with the sequencer it must be false
                    {'autostart': ParameterValue(PythonExpression(
                        ["'", LaunchConfiguration('with_sequencer'), "' != 'true'"]), value_type=bool)}]))
    return nodes


def sequencer_nodes(condition):
    """The other two managers (autostart false, in their YAML) and the sequencer that starts all three."""
    def config(name):
        return PathJoinSubstitution([FindPackageShare('vacuum_bringup'), 'config', name])
    nodes = [Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name=manager,
                  output='screen', parameters=[config(f'{manager.replace("lifecycle_manager", "lifecycle")}.yaml')],
                  condition=condition)
             for manager in ('lifecycle_manager_nav', 'lifecycle_manager_optional')]
    nodes.append(Node(package='vacuum_bringup', executable='lifecycle_sequencer', name='lifecycle_sequencer',
                      output='screen', parameters=[config('lifecycle_sequencer.yaml')], condition=condition))
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
    return LaunchDescription([
        *launch_arguments(),
        *ia368ii_nodes(with_ia368ii),
        *core_nodes(),
        *sequencer_nodes(with_sequencer),
        map_tf_include(),
        twist_mux_node(),
        wheel_bridge_node(with_bridge),
        vacuum_wheel_bridge_node(with_vacuum_bridge),
        teleop_node(with_teleop),
    ])
