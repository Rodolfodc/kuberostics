"""Small pure helpers shared by the stuck_detector and escape_server nodes (spec 005). No ROS imports (D-18).

Toolbox classes (D-19): they keep what the caller stores and answer questions about it; the caller owns the
clock and passes every timestamp.
"""
import math
from typing import List, Optional, Sequence, Tuple


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation about z (2D pose)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class ScanCache:
    """Last laser scan with the time it was received (005-R04: `scan_ranges` of the event is only copied from a
    scan younger than `scan_max_age_s`)."""

    def __init__(self) -> None:
        self._t: Optional[float] = None
        self._ranges: List[float] = []
        self._angle_min = 0.0
        self._angle_increment = 0.0

    def store(self, t: float, ranges: Sequence[float], angle_min: float, angle_increment: float) -> None:
        self._t = t
        self._ranges = list(ranges)
        self._angle_min = angle_min
        self._angle_increment = angle_increment

    def age(self, t: float) -> Optional[float]:
        return None if self._t is None else t - self._t

    def fresh_ranges(self, t: float, max_age_s: float) -> List[float]:
        age = self.age(t)
        if age is None or age > max_age_s:
            return []
        return list(self._ranges)

    def ranges(self) -> List[float]:
        return list(self._ranges)

    def geometry(self) -> Optional[Tuple[float, float]]:
        return None if self._t is None else (self._angle_min, self._angle_increment)


class RisingEdge:
    """false -> true detector; the first message `true` counts as a press (005-R05)."""

    def __init__(self) -> None:
        self._last = False

    def update(self, value: bool) -> bool:
        edge = bool(value) and not self._last
        self._last = bool(value)
        return edge


def scan_geometry_or_default(geometry: Optional[Tuple[float, float]], n_ranges: int) -> Tuple[float, float]:
    """`(angle_min, angle_increment)` of the live laser, or, when none was received, a full circle from -pi (a
    documented assumption: the event does not carry the geometry)."""
    if geometry is not None:
        return geometry
    return -math.pi, (2.0 * math.pi / n_ranges if n_ranges else 0.0)
