"""Slow-simulator scaling of the coverage_server wall-clock timeouts (pure helper and launch argument).

Requirement: 005-R20 (waypoint_timeout_s; coverage_timeout_s and resume_window_s scale with it). The live observation behind it: at 0.09x real
time a 120 s waypoint_timeout_s expired on 4 of 8 waypoints and degraded the covered ratio to 0.932.
"""
import dataclasses
import importlib.util
import pathlib

import pytest

from vacuum_coverage.core.coverage_run import CoverageConfig
from vacuum_coverage.core.slowdown import WALL_CLOCK_PARAMS, parse_factor, scaled_timeout_params

LAUNCH = pathlib.Path(__file__).resolve().parent.parent / 'launch' / 'coverage.launch.py'


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf', 'x', None])
def test_005_R20_invalid_factors_are_rejected(value):
    with pytest.raises(ValueError):
        parse_factor(value)


def test_005_R20_factor_one_overrides_nothing():
    assert scaled_timeout_params('1.0') == {}
    assert scaled_timeout_params(1) == {}


def test_005_R20_factor_scales_the_three_wall_clock_timeouts_from_the_config_defaults():
    got = scaled_timeout_params('11')
    assert set(got) == set(WALL_CLOCK_PARAMS)
    assert got['waypoint_timeout_s'] == 120.0 * 11
    assert got['coverage_timeout_s'] == 1800.0 * 11
    assert got['resume_window_s'] == 120.0 * 11


def test_005_R20_the_scaled_values_still_validate():
    cfg = CoverageConfig(**scaled_timeout_params('0.5'))
    cfg.validate()
    assert cfg.waypoint_timeout_s == 60.0


def test_005_R20_node_defaults_agree_with_the_config_dataclass():
    """The launch scales CoverageConfig's defaults, so the node's own defaults must be the same numbers."""
    pytest.importorskip('rclpy')
    from vacuum_coverage.coverage_server_node import CONFIG_PARAMS
    defaults = {f.name: f.default for f in dataclasses.fields(CoverageConfig)}
    for name in WALL_CLOCK_PARAMS:
        assert CONFIG_PARAMS[name] == defaults[name]


def load_launch():
    spec = importlib.util.spec_from_file_location('coverage_launch_slow', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def node_parameters(factor):
    from launch import LaunchContext
    from launch_ros.actions import LifecycleNode
    context = LaunchContext()
    context.launch_configurations.update({'autostart': 'true', 'sim_slowdown': factor})
    node = next(a for a in load_launch()._actions(context) if isinstance(a, LifecycleNode))
    from launch.utilities import perform_substitutions
    raw = getattr(node, '_Node__parameters', None) or []
    return [{perform_substitutions(context, list(k)) if isinstance(k, tuple) else str(k): v for k, v in d.items()}
            if isinstance(d, dict) else d for d in raw]


def test_005_R20_launch_passes_no_override_by_default():
    assert node_parameters('1.0') == []


def test_005_R20_launch_passes_scaled_timeouts_for_a_slow_simulator():
    params = node_parameters('11')
    assert len(params) == 1
    assert params[0]['waypoint_timeout_s'] == 1320.0


def test_005_R20_launch_declares_sim_slowdown_with_default_one():
    from launch.actions import DeclareLaunchArgument
    declared = {e.name: e for e in load_launch().generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)}
    assert declared['sim_slowdown'].default_value[0].text == '1.0'
