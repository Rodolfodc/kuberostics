"""CoveragePlanner: boustrophedon waypoints over a region (005-R17). Pure Python, no ROS (D-18, D-19).

It only computes. Waypoints are the two ends of every free lane segment; Nav2 plans between them, so the
planner needs no intermediate points. Obstacles (OCCUPIED and UNKNOWN cells) are inflated by
`robot_radius_m + margin_m`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from vacuum_mapping.core.types import Raster

from .grid_ops import reachable_mask


@dataclass(frozen=True)
class Waypoint:
    """Goal pose in the map frame; `pass_index` counts from 0."""
    x: float
    y: float
    yaw: float
    pass_index: int = 0


def _runs(row: np.ndarray) -> List[Tuple[int, int]]:
    """(first, last) index of every run of True in a 1-D bool array."""
    padded = np.concatenate(([False], row, [False]))
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    return [(int(a), int(b) - 1) for a, b in zip(edges[::2], edges[1::2])]


class CoveragePlanner:
    """Defaults are those of spec 005 (R17); the node passes its parameters, nothing is hard-coded there."""

    def __init__(self, lane_spacing_m: float = 0.25, robot_radius_m: float = 0.18, margin_m: float = 0.05,
                 min_segment_m: float = 0.30) -> None:
        if lane_spacing_m <= 0:
            raise ValueError('lane_spacing_m must be positive')
        if robot_radius_m < 0 or margin_m < 0:
            raise ValueError('robot_radius_m and margin_m must not be negative')
        if min_segment_m < 0:
            raise ValueError('min_segment_m must not be negative')
        self.lane_spacing_m = lane_spacing_m
        self.clearance_m = robot_radius_m + margin_m
        self.min_segment_m = min_segment_m

    def reachable(self, raster: Raster, region: np.ndarray) -> np.ndarray:
        """Cells of `region` where the robot centre may be (the lanes run on these)."""
        return reachable_mask(raster, self.clearance_m) & region

    def plan(self, raster: Raster, region: np.ndarray, passes: int = 1) -> List[Waypoint]:
        """Waypoints of all passes, in driving order. Pass 1 runs along the longer axis of the region; each
        further pass rotates the lanes by 90 degrees (pass 3 reverses pass 1: it starts from the far side)."""
        reach = self.reachable(raster, region)
        if not reach.any():
            return []
        rows, cols = np.nonzero(region)
        along_x_first = (cols.max() - cols.min()) >= (rows.max() - rows.min())
        out: List[Waypoint] = []
        for k in range(max(1, passes)):
            out.extend(self._pass(raster, reach, along_x_first == (k % 2 == 0), (k // 2) % 2 == 1, k))
        return out

    def _pass(self, raster: Raster, reach: np.ndarray, along_x: bool, reverse: bool, k: int) -> List[Waypoint]:
        grid = reach if along_x else reach.T          # grid[lane, along]
        used = np.flatnonzero(grid.any(axis=1))
        lo, hi = int(used[0]), int(used[-1])
        step = max(1, int(round(self.lane_spacing_m / raster.resolution)))
        lanes = list(range(lo, hi + 1, step))
        if hi - lanes[-1] > step / 2:                 # keep the last strip within half a lane of a lane
            lanes.append(hi)
        if reverse:
            lanes.reverse()
        min_cells = self.min_segment_m / raster.resolution
        out: List[Waypoint] = []
        for i, lane in enumerate(lanes):
            forward = (i % 2 == 0) != reverse
            runs = [(a, b) for a, b in _runs(grid[lane]) if (b - a + 1) >= min_cells - 1e-9]
            if not forward:
                runs.reverse()
            for a, b in runs:
                first, last = (a, b) if forward else (b, a)
                for along in (first, last):
                    r, c = (lane, along) if along_x else (along, lane)
                    x, y = raster.cell_to_world(r, c)
                    out.append(Waypoint(x, y, self._yaw(along_x, forward), k))
        return out

    @staticmethod
    def _yaw(along_x: bool, forward: bool) -> float:
        if along_x:
            return 0.0 if forward else math.pi
        return math.pi / 2 if forward else -math.pi / 2
