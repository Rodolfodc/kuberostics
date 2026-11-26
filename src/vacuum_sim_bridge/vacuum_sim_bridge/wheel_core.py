"""Twist -> wheel linear speeds, with clamping and a command timeout (pure Python, no ROS)."""
import math
from typing import Optional, Tuple

from vacuum_sim_bridge.profile import RobotProfile


def _clamp(x: float, limit: float) -> float:
    return max(-limit, min(limit, x))


class WheelCommandCore:
    """Holds the last command and turns it into (left, right) wheel speeds in m/s, forward positive.

    The model's python_controler clears the speed signals after consuming them, so the caller must write the
    output every cycle; after `cmd_timeout` seconds without a command the output is (0, 0). Time is injected.
    """

    def __init__(self, profile: RobotProfile):
        self._p = profile
        self._v = 0.0
        self._w = 0.0
        self._stamp: Optional[float] = None

    def command(self, linear: float, angular: float, now: float) -> None:
        if not (math.isfinite(linear) and math.isfinite(angular)):
            linear = angular = 0.0
        self._v = _clamp(linear, self._p.max_linear)
        self._w = _clamp(angular, self._p.max_angular)
        self._stamp = now

    def expired(self, now: float) -> bool:
        return self._stamp is None or (now - self._stamp) > self._p.cmd_timeout

    def output(self, now: float) -> Tuple[float, float]:
        if self.expired(now):
            return 0.0, 0.0
        half = self._p.wheel_separation / 2.0
        # counter-clockwise (positive) angular.z: the right wheel (ahead of the turn) is faster
        return self._v - self._w * half, self._v + self._w * half
