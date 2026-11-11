"""T013: structure tests of launch/skeleton.launch.py (001-R01, 001-R02, 001-R12, 001-R13).

The launch description is built, never launched: no ROS graph, no IA368II checkout and no simulator are needed.
What really runs (is_active, mux behaviour) is verified by launch_testing in T014.
"""
import importlib.util
import pathlib

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node

HERE = pathlib.Path(__file__).resolve().parent
PKG = HERE.parent
LAUNCH = PKG / 'launch' / 'skeleton.launch.py'
CORE_CONFIG = PKG / 'config' / 'lifecycle_core.yaml'
CMD_VEL = '/myRobot/cmd_vel'


@pytest.fixture(scope='module')
def skeleton():
    spec = importlib.util.spec_from_file_location('skeleton_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def entities(skeleton):
    return list(skeleton.generate_launch_description().entities)


def text(substitutions, context=None):
    return perform_substitutions(context or LaunchContext(), normalize_to_list_of_substitutions(substitutions))


def nodes(entities):
    return [e for e in entities if isinstance(e, Node)]


def by_executable(entities, executable):
    found = [n for n in nodes(entities) if n.node_executable == executable]
    assert len(found) == 1, f'expected exactly one {executable}, found {len(found)}'
    return found[0]


def private(node, attribute):
    """The ONLY place that touches launch_ros private state. There is no public getter before the action
    executes; if a launch_ros upgrade renames these, fix this helper and nothing else."""
    mangled = '_Node__' + attribute
    if not hasattr(node, mangled):
        pytest.fail(f'launch_ros.actions.Node no longer has {mangled}: update private() in this test '
                    f'(launch_ros changed its internals)')
    return getattr(node, mangled)


def remaps(node):
    """(from, to) pairs as strings."""
    return [(text(a), text(b)) for a, b in private(node, 'remappings') or []]


def name_of(node):
    """Node name before execution (`node_name` raises until the action runs)."""
    return text(private(node, 'node_name'))


def parameters(node):
    return private(node, 'parameters')


def is_conditional(node):
    return node.condition is not None


def test_launch_arguments_are_declared(entities):
    """001-R01: optional blocks are switched by arguments, so the file works without IA368II or simulator."""
    declared = {e.name for e in entities if isinstance(e, DeclareLaunchArgument)}
    assert {'with_ia368ii', 'with_wheel_bridge', 'with_teleop'} <= declared


IA368II_EXECUTABLES = ['myrobot_server', 'behaviour_server', 'cleaning_server', 'autodocking_server',
                       'charging_server', 'charged_server']


@pytest.mark.parametrize('executable', IA368II_EXECUTABLES)
def test_ia368ii_node_present_and_conditional(entities, executable):
    """001-R01: the six IA368II nodes come from myrobot_bringup and only start with `with_ia368ii`."""
    node = by_executable(entities, executable)
    assert node.node_package == 'myrobot_bringup'
    assert is_conditional(node)


def test_ia368ii_node_names_match_its_lifecycle_config(entities):
    """001-R02: the IA368II manager lists `myrobot` (not myrobot_server) among its node_names."""
    assert name_of(by_executable(entities, 'myrobot_server')) == 'myrobot'
    for executable in IA368II_EXECUTABLES[1:]:
        assert name_of(by_executable(entities, executable)) == executable


def test_cleaning_server_goes_to_the_sink(entities):
    """D-04: the legacy cleaning_server never reaches the mux."""
    assert (CMD_VEL, '/myRobot/legacy/cleaning_cmd_vel') in remaps(by_executable(entities, 'cleaning_server'))


def test_autodocking_server_goes_to_the_docking_input(entities):
    """D-05."""
    assert (CMD_VEL, '/myRobot/mux/docking') in remaps(by_executable(entities, 'autodocking_server'))


def test_ia368ii_lifecycle_manager(entities):
    """001-R02: the IA368II manager keeps its own name and uses the IA368II config."""
    managers = [n for n in nodes(entities) if name_of(n) == 'lifecycle_manager']
    assert len(managers) == 1
    assert managers[0].node_package == 'nav2_lifecycle_manager'
    assert is_conditional(managers[0])


def test_core_lifecycle_manager(entities):
    """001-R02: lifecycle_manager_core is not conditional and reads config/lifecycle_core.yaml."""
    node = next(n for n in nodes(entities) if name_of(n) == 'lifecycle_manager_core')
    assert node.node_package == 'nav2_lifecycle_manager'
    assert node.node_executable == 'lifecycle_manager'
    assert not is_conditional(node)
    assert any('lifecycle_core.yaml' in str(p) for p in parameters(node))


def test_core_config_lists_our_nodes():
    """001-R02."""
    params = yaml.safe_load(CORE_CONFIG.read_text())['lifecycle_manager_core']['ros__parameters']
    names = params['node_names']
    assert sorted(names) == sorted(
        ['supervisor', 'stop_guard', 'coverage_server', 'escape_server', 'stuck_detector'])
    # activation order: the safety net first, the supervisor (which sends goals) last
    assert names[0] == 'stop_guard'
    assert names[-1] == 'supervisor'
    assert params['autostart'] is True
    assert params['bond_timeout'] == 0.0     # our nodes do not create bonds


@pytest.mark.parametrize('package,executable', [
    ('vacuum_supervisor', 'supervisor'), ('vacuum_safety', 'stop_guard'),
    ('vacuum_stubs', 'coverage_server'), ('vacuum_stubs', 'escape_server'),
    ('vacuum_stubs', 'stuck_detector')])
def test_our_nodes_always_start(entities, package, executable):
    """001-R01, 001-R12: stop_guard in particular must never be optional (D-22)."""
    node = by_executable(entities, executable)
    assert node.node_package == package
    assert not is_conditional(node)


def test_twist_mux_is_the_only_publisher_of_cmd_vel(entities):
    """Art. VI, D-03, 001-R10: only twist_mux has /myRobot/cmd_vel as an output.

    This checks remap targets only. The real proof is the T005 contract test (max_publishers on the live
    graph) and the T014 graph check.
    """
    mux = by_executable(entities, 'twist_mux')
    assert not is_conditional(mux)
    assert ('cmd_vel_out', CMD_VEL) in remaps(mux)
    assert any('twist_mux.yaml' in str(p) for p in parameters(mux))
    for node in nodes(entities):
        if node is mux:
            continue
        for _, target in remaps(node):
            if target == CMD_VEL:
                # only the wheel bridge may name it, and it is a subscriber (D-06)
                assert node.node_executable == 'my_robot_cmd_vel_node'


def test_every_ia368ii_publisher_of_cmd_vel_is_remapped_away(entities):
    """Art. VI: cleaning_server and autodocking_server publish /myRobot/cmd_vel by default."""
    for executable in ('cleaning_server', 'autodocking_server'):
        sources = [src for src, dst in remaps(by_executable(entities, executable)) if dst != CMD_VEL]
        assert CMD_VEL in sources


def test_wheel_bridge(entities):
    """D-06, 001-R13."""
    node = by_executable(entities, 'my_robot_cmd_vel_node')
    assert node.node_package == 'my_robot_pkg'
    assert ('/cmd_vel', CMD_VEL) in remaps(node)
    assert is_conditional(node)


def test_teleop(entities):
    """001-R11: teleop goes to the mux teleop input, unstamped (D-13)."""
    node = by_executable(entities, 'teleop_twist_keyboard')
    assert node.node_package == 'teleop_twist_keyboard'
    assert ('cmd_vel', '/myRobot/mux/teleop') in remaps(node)
    assert any(isinstance(p, dict) and any(text(list(k)) == 'stamped' and v is False for k, v in p.items())
               for p in parameters(node))
    assert is_conditional(node)


def test_launch_file_is_installed():
    """setup.py installs launch/ and config/ (T013)."""
    source = (PKG / 'setup.py').read_text()
    assert "'launch'" in source and "'config'" in source
    assert "launch/*.py" in source and "config/*.yaml" in source


@pytest.mark.parametrize('ia368ii,bridge,expected', [
    ('true', 'true', True), ('true', 'false', False), ('false', 'true', False), ('false', 'false', False)])
def test_wheel_bridge_condition(entities, ia368ii, bridge, expected):
    """001-R13: the bridge starts only when with_ia368ii and with_wheel_bridge are both true."""
    context = LaunchContext()
    context.launch_configurations['with_ia368ii'] = ia368ii
    context.launch_configurations['with_wheel_bridge'] = bridge
    condition = by_executable(entities, 'my_robot_cmd_vel_node').condition
    assert condition.evaluate(context) is expected
