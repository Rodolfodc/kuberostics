"""Pure logic of the coverage_server stub (T010, 001-R06). No ROS imports (D-18)."""
from typing import Tuple

MAX_FAKE_RATIO = 0.99     # a stub never claims the room is finished
FAKE_RATIO_PER_S = 0.01


class CoverageRamp:
    """Constant velocity command and a fake, slowly increasing covered ratio."""

    def __init__(self, linear_x: float, angular_z: float):
        self._cmd = (float(linear_x), float(angular_z))

    def command(self) -> Tuple[float, float]:
        return self._cmd

    def ratio(self, elapsed_s: float) -> float:
        return max(0.0, min(MAX_FAKE_RATIO, FAKE_RATIO_PER_S * elapsed_s))
