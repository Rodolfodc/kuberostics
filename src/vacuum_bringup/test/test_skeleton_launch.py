"""Structure tests of launch/skeleton.launch.py (001-R01, 001-R02, 001-R12, 001-R13).

The launch description is built, never launched: no ROS graph, no IA368II checkout and no simulator are needed.
What really runs (is_active, mux behaviour) is verified by the live-graph launch_testing tests (test_graph_*.py).
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


def by_executable(entities, executable, package=None):
    found = [n for n in nodes(entities) if n.node_executable == executable
             and (package is None or n.node_package == package)]
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
    assert {'with_ia368ii', 'with_wheel_bridge', 'with_teleop', 'wheel_bridge'} <= declared


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
    if executable in ('coverage_server', 'escape_server', 'stuck_detector'):
        # a stub is replaced only by the real node (real_coverage / real_nodes), never dropped
        assert node.condition is not None
    else:
        assert not is_conditional(node)


def test_twist_mux_is_the_only_publisher_of_cmd_vel(entities):
    """Art. VI, D-03, 001-R10: only twist_mux has /myRobot/cmd_vel as an output.

    This checks remap targets only. The real proof is the contract test (max_publishers on the live
    graph) and the live-graph checks.
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
    node = by_executable(entities, 'my_robot_cmd_vel_node', 'my_robot_pkg')
    assert ('/cmd_vel', CMD_VEL) in remaps(node)
    assert is_conditional(node)


def test_vacuum_wheel_bridge(entities):
    """D-27, 001-R23: the same-name replacement of vacuum_sim_bridge, remapped to the mux output like the stock one."""
    node = by_executable(entities, 'my_robot_cmd_vel_node', 'vacuum_sim_bridge')
    assert name_of(node) == name_of(by_executable(entities, 'my_robot_cmd_vel_node', 'my_robot_pkg'))
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
    """setup.py installs launch/ and config/."""
    source = (PKG / 'setup.py').read_text()
    assert "'launch'" in source and "'config'" in source
    assert "launch/*.py" in source and "config/*.yaml" in source


@pytest.mark.parametrize('ia368ii,bridge,impl,stock,vacuum', [
    ('true', 'true', 'stock', True, False), ('true', 'false', 'stock', False, False),
    ('false', 'true', 'stock', False, False), ('false', 'false', 'stock', False, False),
    ('true', 'true', 'vacuum', False, True), ('false', 'true', 'vacuum', False, True),
    ('true', 'false', 'vacuum', False, False)])
def test_wheel_bridge_condition(entities, ia368ii, bridge, impl, stock, vacuum):
    """001-R13, D-27: the stock bridge starts only when with_ia368ii and with_wheel_bridge are true and wheel_bridge is
    stock; ours when with_wheel_bridge is true and wheel_bridge is vacuum. Never both."""
    context = LaunchContext()
    context.launch_configurations['with_ia368ii'] = ia368ii
    context.launch_configurations['with_wheel_bridge'] = bridge
    context.launch_configurations['wheel_bridge'] = impl
    assert by_executable(entities, 'my_robot_cmd_vel_node', 'my_robot_pkg').condition.evaluate(context) is stock
    assert by_executable(entities, 'my_robot_cmd_vel_node', 'vacuum_sim_bridge').condition.evaluate(context) is vacuum


def evaluate(condition, **arguments):
    from launch.actions import SetLaunchConfiguration
    context = LaunchContext()
    for key, value in arguments.items():
        SetLaunchConfiguration(key, value).visit(context)
    return condition.evaluate(context)


def declared_default(entities, name):
    return next(e for e in entities if isinstance(e, DeclareLaunchArgument) and e.name == name).default_value


def test_map_tf_default_false_and_included_behind_the_flag(entities):
    """map -> my_robot_odom is opt-in (existing graphs are unchanged)."""
    from launch.actions import IncludeLaunchDescription
    assert text(declared_default(entities, 'map_tf')) == 'false'
    assert text(declared_default(entities, 'map_source')) == 'static'
    include = next(e for e in entities if isinstance(e, IncludeLaunchDescription)
                   and any(k == 'map_source' for k, _ in e.launch_arguments))
    assert evaluate(include.condition, map_tf='true') is True
    assert evaluate(include.condition, map_tf='false') is False


# ---- real_nodes (001-R01, 005-R28, 005-R33): the real escape_server and stuck_detector in place of the stubs
def _context(**configs):
    context = LaunchContext()
    context.launch_configurations.update({'real_coverage': 'false', 'real_nodes': 'false', 'sim_slowdown': '1.0',
                                          'with_sequencer': 'false', **configs})
    return context


def _includes(entities, fragment):
    from launch.actions import IncludeLaunchDescription
    return [e for e in entities if isinstance(e, IncludeLaunchDescription)
            and fragment in text(e._IncludeLaunchDescription__launch_description_source.location)]


def test_001_R01_real_nodes_is_declared_and_off_by_default(entities):
    declared = {e.name: text(e.default_value) for e in entities if isinstance(e, DeclareLaunchArgument)}
    assert declared['real_nodes'] == 'false'


@pytest.mark.parametrize('real_nodes,real_coverage,stub_runs,real_runs', [
    ('false', 'false', True, False),
    ('true', 'false', False, True),
    ('false', 'true', True, False),
    ('true', 'true', False, True)])
def test_005_R33_real_nodes_swaps_exactly_one_variant_of_escape_and_stuck(
        entities, real_nodes, real_coverage, stub_runs, real_runs):
    context = _context(real_nodes=real_nodes, real_coverage=real_coverage)
    for executable in ('escape_server', 'stuck_detector'):
        stub = by_executable(entities, executable, 'vacuum_stubs')
        assert stub.condition.evaluate(context) is stub_runs, executable
    (include,) = _includes(entities, 'stuck_escape.launch.py')
    assert include.condition.evaluate(context) is real_runs
    args = {text(k, context): text(v, context) for k, v in include.launch_arguments}
    assert args['autostart'].lower() == 'false'      # lifecycle_manager_core / the sequencer start them


@pytest.mark.parametrize('real_nodes,real_coverage,real_runs', [
    ('false', 'false', False), ('true', 'false', True), ('false', 'true', True), ('true', 'true', True)])
def test_005_R35_real_nodes_implies_real_coverage(entities, real_nodes, real_coverage, real_runs):
    context = _context(real_nodes=real_nodes, real_coverage=real_coverage)
    stub = by_executable(entities, 'coverage_server', 'vacuum_stubs')
    (include,) = _includes(entities, 'coverage.launch.py')
    assert include.condition.evaluate(context) is real_runs
    assert stub.condition.evaluate(context) is (not real_runs)


def test_001_R12_stubs_and_real_nodes_share_the_names_the_core_manager_lists(entities):
    """The manager list is the same for both variants, so the real launch must use the stub names."""
    from ament_index_python.packages import get_package_share_directory
    spec = importlib.util.spec_from_file_location(
        'stuck_escape', pathlib.Path(get_package_share_directory('vacuum_coverage')) / 'launch'
        / 'stuck_escape.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    real = {name_of(a) for a in module.generate_launch_description().entities
            if type(a).__name__ == 'LifecycleNode'}
    stubs = {name_of(by_executable(entities, e, 'vacuum_stubs')) for e in ('escape_server', 'stuck_detector')}
    listed = set(yaml.safe_load(CORE_CONFIG.read_text())['lifecycle_manager_core']['ros__parameters']['node_names'])
    assert real == stubs == {'escape_server', 'stuck_detector'}
    assert stubs <= listed
