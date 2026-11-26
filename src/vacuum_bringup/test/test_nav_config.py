"""Config tests of the Nav2 files (spec 005, T510): no ROS graph, no Nav2 install needed (YAML, XML, launch file
built but never launched). Behaviour on a running Nav2 is test_nav_graph.py and test_nav_timeout.py.
"""
import importlib.util
import pathlib
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parent.parent
NAV_GOAL_TIMEOUT_S = 180.0            # contract parameter nav_goal_timeout_s (005-R34), a BT value, not a node parameter
RECOVERY_NODES = {'RecoveryNode', 'Spin', 'BackUp', 'Wait', 'ClearEntireCostmap', 'ClearCostmapExceptRegion',
                  'ClearCostmapAroundRobot', 'DriveOnHeading', 'AssistedTeleop'}


@pytest.fixture(scope='module')
def params():
    return yaml.safe_load((PKG / 'config' / 'nav2_params.yaml').read_text())


@pytest.fixture(scope='module')
def tree():
    return ET.parse(PKG / 'config' / 'nav_bt.xml').getroot()


def test_005_R23_every_velocity_publisher_declares_enable_stamped_cmd_vel_false(params):
    """D-13. Only controller_server publishes velocity: behavior_server, smoother and collision monitor are absent."""
    assert params['controller_server']['ros__parameters']['enable_stamped_cmd_vel'] is False
    for name in ('behavior_server', 'velocity_smoother', 'collision_monitor'):
        assert name not in params


def test_005_R23_odometry_comes_from_the_bridge_topic(params):
    assert params['controller_server']['ros__parameters']['odom_topic'] == '/myrobot/odom'
    assert params['bt_navigator']['ros__parameters']['odom_topic'] == '/myrobot/odom'


def test_005_R19_plugins_are_the_ones_of_the_plan(params):
    c = params['controller_server']['ros__parameters']
    assert c['FollowPath']['plugin'] == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert params['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
    assert params['bt_navigator']['ros__parameters']['navigators'] == ['navigate_to_pose']


def test_005_R19_speed_and_footprint_follow_the_robot_facts(params):
    """Footprint radius 0.18 m about the model origin; below the robot caps 0.5 m/s and 90 deg/s, and below the
    0.30 m/s limit of spec 009a."""
    import math
    follow = params['controller_server']['ros__parameters']['FollowPath']
    assert 0.0 < follow['desired_linear_vel'] <= 0.30
    assert 0.0 < follow['rotate_to_heading_angular_vel'] <= math.radians(90.0)
    assert params['local_costmap']['local_costmap']['ros__parameters']['robot_radius'] == 0.18
    assert params['global_costmap']['global_costmap']['ros__parameters']['robot_radius'] == 0.18


def test_005_R19_costmaps_use_the_robot_frames_and_the_scan_topic(params):
    local = params['local_costmap']['local_costmap']['ros__parameters']
    glob = params['global_costmap']['global_costmap']['ros__parameters']
    assert local['global_frame'] == 'my_robot_odom' and glob['global_frame'] == 'map'
    assert local['robot_base_frame'] == glob['robot_base_frame'] == 'base_link'
    for costmap in (local, glob):
        assert costmap['obstacle_layer']['scan']['topic'] == '/scan'
        assert costmap['obstacle_layer']['scan']['sensor_frame'] == 'laser_link'


def test_005_R34_the_goal_is_bounded_by_a_timeout_decorator_equal_to_nav_goal_timeout_s(tree):
    timeouts = list(tree.iter('Timeout'))
    assert len(timeouts) == 1
    assert float(timeouts[0].attrib['msec']) == NAV_GOAL_TIMEOUT_S * 1000.0
    # the Timeout wraps the whole navigation, not just one child
    main = next(bt for bt in tree.iter('BehaviorTree') if bt.attrib['ID'] == tree.attrib['main_tree_to_execute'])
    assert [child.tag for child in main] == ['Timeout']


def test_005_R19_the_tree_has_no_recovery_nodes_and_computes_and_follows(tree):
    tags = {element.tag for element in tree.iter()}
    assert tags.isdisjoint(RECOVERY_NODES)
    assert {'ComputePathToPose', 'FollowPath'} <= tags
    assert tree.attrib['BTCPP_format'] == '4'


def launch_name(node):
    """Node name before execution: the public `node_name` property needs an executed action, so fall back to the
    stored substitutions."""
    from launch import LaunchContext
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    raw = getattr(node, '_Node__node_name', None)
    return perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(raw)) if raw is not None \
        else node.node_name


def expanded_nodes(module, **arguments):
    """The Node actions that nav.launch.py's OpaqueFunction produces for the given launch arguments."""
    from launch import LaunchContext
    from launch_ros.actions import Node
    context = LaunchContext()
    context.launch_configurations.update({
        'params_file': str(PKG / 'config' / 'nav2_params.yaml'), 'bt_xml': str(PKG / 'config' / 'nav_bt.xml'),
        'autostart': 'true', 'with_manager': 'true', 'sim_slowdown': '1.0'})
    context.launch_configurations.update(arguments)
    return [e for e in module._nodes(context) if isinstance(e, Node)]


def test_005_R19_nav_launch_starts_exactly_the_three_nav2_nodes_and_their_manager():
    """That bt_navigator really reads nav_bt.xml through the bt_xml argument is proven live by test_nav_timeout.py."""
    spec = importlib.util.spec_from_file_location('nav_launch', PKG / 'launch' / 'nav.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from launch_ros.actions import Node
    nodes = {launch_name(e): e for e in expanded_nodes(module)}
    assert set(nodes) == {'controller_server', 'planner_server', 'bt_navigator', 'lifecycle_manager_nav'}


def test_005_R23_controller_server_cmd_vel_is_remapped_to_mux_nav():
    spec = importlib.util.spec_from_file_location('nav_launch2', PKG / 'launch' / 'nav.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from launch import LaunchContext
    from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
    from launch_ros.actions import Node
    controller = next(e for e in expanded_nodes(module) if launch_name(e) == 'controller_server')
    remaps = [(perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(a)),
               perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(b)))
              for a, b in (getattr(controller, '_Node__remappings', None) or controller.expanded_remapping_rules or [])]
    assert ('cmd_vel', '/myRobot/mux/nav') in remaps
