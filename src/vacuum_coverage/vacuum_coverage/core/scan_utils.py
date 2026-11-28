"""Pure helpers over `sensor_msgs/LaserScan`-like arrays (spec 005). No ROS imports (D-18).

Convention: angles are in the laser frame, 0 is straight ahead and positive is to the left (counter-clockwise,
the ROS convention); sample i is at `angle_min + i * angle_increment`. A reading is *invalid* when it is NaN or
not positive; `inf` is a valid "nothing in range" reading, i.e. free space.
"""
import math
from typing import Optional, Sequence, Tuple


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _valid(r: float) -> bool:
    return not math.isnan(r) and r > 0.0


def side_means(ranges: Sequence[float], angle_min: float, angle_increment: float,
               range_cap_m: float = 5.0) -> Tuple[float, float]:
    """Mean range of the left half (0 < angle < pi) and of the right half (-pi < angle < 0).

    Invalid readings are dropped; every other reading, `inf` included, is capped at `range_cap_m` so the result
    is finite and deterministic. A half with no valid reading has mean 0.0."""
    sums = {'left': [0.0, 0], 'right': [0.0, 0]}
    for i, r in enumerate(ranges):
        if not _valid(r):
            continue
        a = wrap_angle(angle_min + i * angle_increment)
        side = 'left' if 0.0 < a < math.pi else 'right' if -math.pi < a < 0.0 else None
        if side is None:
            continue
        sums[side][0] += min(r, range_cap_m)
        sums[side][1] += 1
    left = sums['left'][0] / sums['left'][1] if sums['left'][1] else 0.0
    right = sums['right'][0] / sums['right'][1] if sums['right'][1] else 0.0
    return left, right


def sector_min(ranges: Sequence[float], angle_min: float, angle_increment: float, center_rad: float,
               half_width_rad: float) -> Optional[float]:
    """Smallest valid reading within `center_rad +- half_width_rad`; `inf` when the sector is all free;
    None when the scan has no valid reading there (e.g. a front-only laser asked about the rear)."""
    best = None
    for i, r in enumerate(ranges):
        if not _valid(r):
            continue
        if abs(wrap_angle(angle_min + i * angle_increment - center_rad)) <= half_width_rad + 1e-9:
            best = r if best is None else min(best, r)
    return best
