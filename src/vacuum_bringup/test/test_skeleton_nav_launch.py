"""Structure tests of the Nav2 wiring in skeleton.launch.py (005-R35, 005-R19, 005-R34, 005-R39). The launch description is
built, never launched; the live graph is test_graph_nav_sequenced.py."""
import importlib.util
import pathlib

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

PKG = pathlib.Path(__file__).resolve().parent.parent


def load(name):
    spec = importlib.util.spec_from_file_location(name, PKG / 'launch' / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def entities():
    return list(load('skeleton').generate_launch_description().entities)


def text(value, context=None):
    return perform_substitutions(context or LaunchContext(), normalize_to_list_of_substitutions(value))


def declared(entities):
    return {e.name: text(e.default_value) for e in entities if isinstance(e, DeclareLaunchArgument)}


def include_of(entities, fragment):
    found = [e for e in entities if isinstance(e, IncludeLaunchDescription)
             and fragment in text(e._IncludeLaunchDescription__launch_description_source.location)]
    assert len(found) == 1, f'expected one include of {fragment}, found {len(found)}'
    return found[0]


def args_of(include, **configs):
    context = LaunchContext()
    context.launch_configurations.update({'with_sequencer': 'false', 'sim_slowdown': '1.0', **configs})
    return {text(k, context): text(v, context) for k, v in include.launch_arguments}, context


def test_005_R35_lifecycle_nav_yaml_lists_the_nav2_nodes_of_nav_launch():
    names = yaml.safe_load((PKG / 'config' / 'lifecycle_nav.yaml').read_text())[
        'lifecycle_manager_nav']['ros__parameters']['node_names']
    assert names == load('nav').NAV_NODES


def test_005_R35_nav_is_off_by_default(entities):
    assert declared(entities)['nav'] == 'false'
    include = include_of(entities, 'nav.launch.py')
    context = LaunchContext()
    context.launch_configurations['nav'] = 'false'
    assert include.condition.evaluate(context) is False
    context.launch_configurations['nav'] = 'true'
    assert include.condition.evaluate(context) is True


def test_005_R35_with_the_sequencer_nav_launch_does_not_start_its_own_manager(entities):
    include = include_of(entities, 'nav.launch.py')
    args, _ = args_of(include, with_sequencer='true')
    assert args['with_manager'] == 'False' or args['with_manager'] == 'false'
    args, _ = args_of(include, with_sequencer='false')
    assert args['with_manager'].lower() == 'true'


def test_005_R34_sim_slowdown_is_forwarded_to_nav_and_coverage(entities):
    assert declared(entities)['sim_slowdown'] == '1.0'
    for fragment in ('nav.launch.py', 'coverage.launch.py'):
        args, _ = args_of(include_of(entities, fragment), sim_slowdown='7')
        assert args['sim_slowdown'] == '7', fragment


def test_005_R35_real_coverage_replaces_the_stub_and_is_off_by_default(entities):
    assert declared(entities)['real_coverage'] == 'false'
    include = include_of(entities, 'coverage.launch.py')
    args, _ = args_of(include)
    assert args['autostart'].lower() == 'false'         # lifecycle_manager_core starts it
    context = LaunchContext()
    context.launch_configurations['real_coverage'] = 'false'
    assert include.condition.evaluate(context) is False
    from launch_ros.actions import Node
    stub = [e for e in entities if isinstance(e, Node) and e.node_executable == 'coverage_server']
    assert len(stub) == 1
    assert stub[0].condition.evaluate(context) is True
    context.launch_configurations['real_coverage'] = 'true'
    assert stub[0].condition.evaluate(context) is False
    assert include.condition.evaluate(context) is True


def test_005_R35_no_manager_without_nodes_is_launched_or_listed(entities):
    """A Nav2 lifecycle_manager with node_names [] aborts, so the sequencer lists only managers that have nodes."""
    from launch_ros.actions import Node
    managers = [e for e in entities if isinstance(e, Node) and e.node_executable == 'lifecycle_manager'
                and 'lifecycle_manager_nav' in text(e._Node__node_name)]
    assert len(managers) == 1
    sequencers = [e for e in entities if isinstance(e, Node) and e.node_executable == 'lifecycle_sequencer']
    assert len(sequencers) == 2
    listed = {}
    for with_nav in ('true', 'false'):
        context = LaunchContext()
        context.launch_configurations.update({'with_sequencer': 'true', 'nav': with_nav})
        active = [s for s in sequencers if s.condition.evaluate(context)]
        assert len(active) == 1
        (value,) = [v for k, v in active[0]._Node__parameters[-1].items() if text(list(k)) == 'managers']
        listed[with_nav] = [text(item).split()[0] for item in value]
    assert listed['true'] == ['lifecycle_manager_core', 'lifecycle_manager_nav']
    assert listed['false'] == ['lifecycle_manager_core']
    assert managers[0].condition.evaluate(_ctx('true', 'true')) and not managers[0].condition.evaluate(_ctx('true', 'false'))
    assert not any('optional' in text(e._Node__node_name) for e in entities if isinstance(e, Node))


def _ctx(with_sequencer, nav):
    context = LaunchContext()
    context.launch_configurations.update({'with_sequencer': with_sequencer, 'nav': nav})
    return context
