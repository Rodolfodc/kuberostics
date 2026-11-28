"""Structure tests of launch/map_tf.launch.py (spec 003-R05; contract `tf`: map -> my_robot_odom).

The description is built, never launched. The live TF chain is test_graph_map_tf.py.
"""
import importlib.util
import pathlib

import pytest
from launch import LaunchContext
from launch.actions import OpaqueFunction, SetLaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

LAUNCH = pathlib.Path(__file__).resolve().parent.parent / 'launch' / 'map_tf.launch.py'


@pytest.fixture(scope='module')
def map_tf():
    spec = importlib.util.spec_from_file_location('map_tf_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_with(source):
    context = LaunchContext()
    if source is not None:
        SetLaunchConfiguration('map_source', source).visit(context)
    return context


def test_static_is_an_identity_map_to_odom_transform(map_tf):
    """003-R05: map -> my_robot_odom, identity, from tf2_ros static_transform_publisher."""
    (node,) = map_tf._select_source(context_with('static'))
    assert isinstance(node, Node) and node.node_package == 'tf2_ros'
    args = [perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(a))
            for a in node._Node__arguments]
    flags = dict(zip(args[::2], args[1::2]))
    assert flags['--frame-id'] == 'map' and flags['--child-frame-id'] == 'my_robot_odom'
    assert all(float(flags[k]) == 0.0 for k in ('--x', '--y', '--z', '--roll', '--pitch', '--yaw'))


def test_slam_fails_with_a_clear_message(map_tf):
    """slam_toolbox is not integrated: a clear error, not a silent missing frame."""
    with pytest.raises(RuntimeError, match='slam_toolbox is not integrated'):
        map_tf._select_source(context_with('slam'))


def test_unknown_source_is_rejected(map_tf):
    with pytest.raises(RuntimeError, match='map_source must be'):
        map_tf._select_source(context_with('gps'))


def test_default_source_is_static(map_tf):
    description = map_tf.generate_launch_description()
    declared = [e for e in description.entities if getattr(e, 'name', None) == 'map_source']
    assert perform_substitutions(LaunchContext(), declared[0].default_value) == 'static'
    assert any(isinstance(e, OpaqueFunction) for e in description.entities)
