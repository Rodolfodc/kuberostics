"""Raw laser data of the model -> LaserScan fields (pure Python, no ROS)."""
import math
from dataclasses import dataclass
from typing import List, Optional

from vacuum_sim_bridge.profile import RobotProfile


@dataclass(frozen=True)
class ScanData:
    ranges: List[float]
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    warning: Optional[str] = None


def convert_scan(raw, profile: RobotProfile) -> Optional[ScanData]:
    """`raw` is the distSignal float array. A miss is written as 0 by the model script, so every value below
    range_min (also negative or NaN) becomes inf, and so does every value above range_max. Returns None when
    there are fewer than 2 beams. angle_max is always angle_min + (n - 1) * increment."""
    n = len(raw)
    if n < 2:
        return None
    lo, hi = profile.scan_range_min, profile.scan_range_max
    ranges = []
    for r in raw:
        r = float(r)
        ranges.append(r if (lo <= r <= hi) else math.inf)   # NaN fails both comparisons
    inc = profile.scan_angle_increment
    warning = None
    if n != profile.scan_num_beams:
        warning = f'received {n} beams, expected {profile.scan_num_beams}; keeping increment {inc:.6f} rad'
    return ScanData(ranges, profile.scan_angle_min, profile.scan_angle_min + (n - 1) * inc, inc, lo, hi, warning)
