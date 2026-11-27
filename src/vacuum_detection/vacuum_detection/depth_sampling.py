"""Depth lookup at a pixel (spec 009a-R12 support). Pure numpy, no ROS (D-18): only computes."""
import math

import numpy as np


def depth_at(depth_m: np.ndarray, u: float, v: float, half_window_px: int = 2) -> float:
    """Median of the valid (finite, > 0) depths in a square window around (u, v); NaN if none or outside."""
    h, w = depth_m.shape[:2]
    if not (math.isfinite(u) and math.isfinite(v)):
        return math.nan
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return math.nan
    win = depth_m[max(0, vi - half_window_px):vi + half_window_px + 1,
                  max(0, ui - half_window_px):ui + half_window_px + 1]
    valid = win[np.isfinite(win) & (win > 0.0)]
    return float(np.median(valid)) if valid.size else math.nan
