"""Nav2 of spec 005: planner_server, controller_server, bt_navigator and lifecycle_manager_nav (005-R19, R23, R29, R34).

Configuration only (Art. II): the stock Nav2 executables with config/nav2_params.yaml and config/nav_bt.xml.
* controller_server `cmd_vel` is remapped to /myRobot/mux/nav (priority 60 in twist_mux.yaml): the only Nav2 node
  that publishes velocity. behavior_server, velocity_smoother and collision_monitor are NOT launched.
* The map (/map, transient local) and the TF map -> my_robot_odom come from outside (map_server / SLAM, or the
  fake robot of the tests). /scan and /myrobot/odom come from the sim bridges (D-07).
* Arguments: `autostart` (default true: the spike mode; with lifecycle_sequencer pass false, the sequencer then
  starts the manager), `with_manager` (default true; false when skeleton.launch.py with_sequencer:=true already
  runs a lifecycle_manager_nav, whose node_names must then list the three nodes), `params_file`, `bt_xml`,
  `sim_slowdown` (default 1.0 = real time: the files are used untouched). For a slow simulator (about 0.09x real
  time with the laser model; use_sim_time stays false) `sim_slowdown:=<factor>` writes an overlay copy of the params
  and of the BT with the wall-clock timeouts multiplied by the factor (vacuum_bringup/slowdown.py:
  movement_time_allowance, costmap_update_timeout of controller and planner, the BT Timeout of 005-R34).

Differences found between Nav2 1.3.14 (Jazzy) and 1.5.2 (Lyrical), absorbed by the single YAML (Nav2 ignores
parameters it does not declare, except where noted below). Method: run the stack in both containers, compare
`ros2 param list` of every node with the YAML keys; only `path_handler_plugins` and `PathHandler.*` are not
declared on Jazzy:
* bt_navigator: Lyrical aborts on_configure if the old `error_code_names` is set (FATAL, "MUST be removed"), so the
  YAML omits it; both distributions declare `error_code_name_prefixes`.
* controller_server gained `path_handler_plugins` on Lyrical (Jazzy ignores it) and, with it, a required plugin; the available class
  is `nav2_controller::FeasiblePathHandler` (there is no SimplePathHandler: the node dies in on_configure otherwise).
* `enable_stamped_cmd_vel` is declared on both (verified with `ros2 param list`), so setting it false is real on
  both, not merely harmless on Jazzy.
* Lyrical ships more stock BTs (route graph, bounds check); plugin names used here (NavfnPlanner,
  RegulatedPurePursuitController, SimpleGoalChecker, SimpleProgressChecker, ObstacleLayer, InflationLayer,
  StaticLayer) are identical on both. The BT `Timeout` decorator is BehaviorTree.CPP built in on both.
"""
import atexit
import os
import shutil
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from vacuum_bringup.slowdown import count_bt_timeouts, parse_factor, scale_bt_xml, scale_nav_params, scaled_keys

NAV_CMD_VEL = '/myRobot/mux/nav'
NAV_NODES = ['controller_server', 'planner_server', 'bt_navigator']


def slowed_files(params_path, bt_path, factor, directory=None, log=None):
    """(params_path, bt_path) to launch with: the inputs for factor 1.0, else overlay copies with scaled timeouts.

    The overlay directory is created here when `directory` is None and removed at exit of the launch process (the
    nodes read the files at start and on configure, before the launch ends). `log(message)` receives a warning when
    a custom params_file or BT has nothing to scale (default: the launch logger)."""
    factor = parse_factor(factor)
    if factor == 1.0:
        return params_path, bt_path
    log = log or get_logger('vacuum_bringup.nav').warning
    if directory is None:
        directory = tempfile.mkdtemp(prefix='nav_slowdown_')
        atexit.register(shutil.rmtree, directory, ignore_errors=True)
    os.makedirs(directory, exist_ok=True)
    with open(params_path) as f:
        loaded = yaml.safe_load(f)
    if not scaled_keys(loaded):
        log(f'sim_slowdown={factor}: params_file {params_path} has nothing to scale (none of the timeouts of '
            'vacuum_bringup/slowdown.py NAV_TIMEOUT_PARAMS is set); its values are used as they are')
    scaled = scale_nav_params(loaded, factor)
    out_params = os.path.join(directory, 'nav2_params_slow.yaml')
    with open(out_params, 'w') as f:
        yaml.safe_dump(scaled, f, sort_keys=False)
    with open(bt_path) as f:
        text = f.read()
    if count_bt_timeouts(text) == 0:
        log(f'sim_slowdown={factor}: bt_xml {bt_path} has no Timeout decorator to scale; it is used as it is')
    out_bt = os.path.join(directory, 'nav_bt_slow.xml')
    with open(out_bt, 'w') as f:
        f.write(scale_bt_xml(text, factor))
    return out_params, out_bt


def _nodes(context, *args, **kwargs):
    params, bt_xml = slowed_files(LaunchConfiguration('params_file').perform(context),
                                  LaunchConfiguration('bt_xml').perform(context),
                                  LaunchConfiguration('sim_slowdown').perform(context))
    # later dictionaries override the YAML: the BT path is the installed (or scaled) file
    bt_param = {'default_nav_to_pose_bt_xml': bt_xml}
    return [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=[params], remappings=[('cmd_vel', NAV_CMD_VEL)]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params, bt_param]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_nav',
             output='screen', condition=IfCondition(LaunchConfiguration('with_manager')),
             parameters=[{'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                          'bond_timeout': 0.0,
                          'node_names': NAV_NODES}]),
    ]


def generate_launch_description():
    share = FindPackageShare('vacuum_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('bt_xml', default_value=PathJoinSubstitution([share, 'config', 'nav_bt.xml'])),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('with_manager', default_value='true'),
        DeclareLaunchArgument('sim_slowdown', default_value='1.0',
                              description='factor > 0 that scales the wall-clock timeouts (1.0 = real time)'),
        OpaqueFunction(function=_nodes),
    ])
