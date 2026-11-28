"""Slow-simulator scaling of the wall-clock timeouts of coverage_server (pure Python, no ROS imports).

See vacuum_bringup/slowdown.py for the reason (simulator at about 0.09x real time, use_sim_time false). The launch
argument `sim_slowdown:=<factor>` (default 1.0: no parameter is overridden, the node defaults apply) multiplies
`waypoint_timeout_s`, `coverage_timeout_s` and `resume_window_s` by the factor. The defaults come from
CoverageConfig, the single source of truth, so nothing is duplicated.
"""
import dataclasses
import math
from typing import Any, Dict

from .coverage_run import CoverageConfig

WALL_CLOCK_PARAMS = ('coverage_timeout_s', 'waypoint_timeout_s', 'resume_window_s')


def parse_factor(value: Any) -> float:
    """`sim_slowdown` as a float; must be finite and > 0 (1.0 = real time)."""
    try:
        factor = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'sim_slowdown must be a number, got {value!r}') from None
    if not math.isfinite(factor) or factor <= 0.0:
        raise ValueError(f'sim_slowdown must be finite and > 0, got {value!r}')
    return factor


def scaled_timeout_params(factor: Any) -> Dict[str, float]:
    """Parameter overrides for coverage_server: {} for factor 1.0, else every wall-clock timeout times `factor`."""
    factor = parse_factor(factor)
    if factor == 1.0:
        return {}
    defaults = {f.name: f.default for f in dataclasses.fields(CoverageConfig)}
    return {name: float(defaults[name]) * factor for name in WALL_CLOCK_PARAMS}
