"""Slow-simulator scaling of the wall-clock timeouts of the Nav2 configuration (pure Python, no ROS imports).

The simulator with the laser model can run at about 0.09x real time (1.5 to 1.9 steps/s). Every timeout of
nav2_params.yaml and nav_bt.xml is measured on the wall clock (use_sim_time stays false, there is no /clock), so
at that speed valid goals are aborted ("Failed to make progress", BT Timeout, costmap update timeout). The launch
argument `sim_slowdown:=<factor>` (default 1.0 = real time, nothing changes) multiplies these timeouts by the
factor. The caller (launch/nav.launch.py) decides what to write and where; this module only computes.

Scaled: progress_checker.movement_time_allowance, controller_server/planner_server costmap_update_timeout and the
BT `Timeout` decorator (005-R34, nav_goal_timeout_s). Not scaled on purpose: rates, tolerances and
transform_tolerance (they compare stamps taken on the same slow clock source, not wall time).
"""
import copy
import math
import re
from typing import Any, Dict

# (node, path inside ros__parameters) of every wall-clock timeout scaled by the factor
NAV_TIMEOUT_PARAMS = (
    ('controller_server', ('progress_checker', 'movement_time_allowance')),
    ('controller_server', ('costmap_update_timeout',)),
    ('planner_server', ('costmap_update_timeout',)),
)

_TIMEOUT_MSEC = re.compile(r'(<Timeout\b[^>]*?\bmsec=")([0-9.]+)(")')


def parse_factor(value: Any) -> float:
    """`sim_slowdown` as a float; must be finite and > 0 (1.0 = real time)."""
    try:
        factor = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'sim_slowdown must be a number, got {value!r}') from None
    if not math.isfinite(factor) or factor <= 0.0:
        raise ValueError(f'sim_slowdown must be finite and > 0, got {value!r}')
    return factor


def scale_nav_params(params: Dict[str, Any], factor: float) -> Dict[str, Any]:
    """A copy of the parsed nav2_params.yaml with the timeouts of NAV_TIMEOUT_PARAMS multiplied by `factor`.
    Keys that are absent are skipped (a user params_file may omit them); the input is not modified."""
    factor = parse_factor(factor)
    out = copy.deepcopy(params)
    for node, path in NAV_TIMEOUT_PARAMS:
        holder = out.get(node, {}).get('ros__parameters')
        for key in path[:-1]:
            holder = holder.get(key) if isinstance(holder, dict) else None
        if isinstance(holder, dict) and isinstance(holder.get(path[-1]), (int, float)):
            holder[path[-1]] = float(holder[path[-1]]) * factor
    return out


def scale_bt_xml(text: str, factor: float) -> str:
    """The behavior tree text with every `<Timeout msec="...">` multiplied by `factor` (005-R34)."""
    factor = parse_factor(factor)

    def scaled(match):
        msec = float(match.group(2)) * factor
        return f'{match.group(1)}{int(round(msec))}{match.group(3)}'

    return _TIMEOUT_MSEC.sub(scaled, text)


def scaled_keys(params: Dict[str, Any]) -> list:
    """The `node.path` names of NAV_TIMEOUT_PARAMS that `params` really has (what scale_nav_params would scale).
    Empty for a custom params_file without them: the caller can then say that nothing was scaled."""
    found = []
    for node, path in NAV_TIMEOUT_PARAMS:
        holder = params.get(node, {}).get('ros__parameters') if isinstance(params, dict) else None
        for key in path[:-1]:
            holder = holder.get(key) if isinstance(holder, dict) else None
        if isinstance(holder, dict) and isinstance(holder.get(path[-1]), (int, float)):
            found.append(f'{node}.{".".join(path)}')
    return found


def count_bt_timeouts(text: str) -> int:
    """How many `<Timeout msec="...">` decorators scale_bt_xml would scale."""
    return len(_TIMEOUT_MSEC.findall(text))
