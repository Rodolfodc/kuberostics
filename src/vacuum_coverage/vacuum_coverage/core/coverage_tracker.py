"""CoverageTracker: covered mask and `covered_ratio` (005-R21). Pure Python, no ROS (D-18).

It only computes: the caller feeds it poses (from TF) and decides when to publish the ratio.
`coverable_mask` is re-exported because ratio and plan must share one definition of "coverable".
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from vacuum_mapping.core.types import Raster

from .grid_ops import coverable_mask  # noqa: F401  (re-export)


class CoverageTracker:
    """Marks the cells within `tool_radius_m` of every pose and reports covered / coverable.

    `coverable` is the denominator (see `coverable_mask`); `region` is only used by `covered_grid` to tell
    "outside the region" (-1) from "not covered yet" (0). With an empty `coverable` the ratio is 1.0: there
    is nothing left to clean.
    """

    def __init__(self, raster: Raster, coverable: np.ndarray, tool_radius_m: float,
                 region: Optional[np.ndarray] = None) -> None:
        if coverable.shape != raster.cells.shape:
            raise ValueError('coverable must have the raster shape')
        if region is not None and region.shape != raster.cells.shape:
            raise ValueError('region must have the raster shape')
        if tool_radius_m <= 0:
            raise ValueError('tool_radius_m must be positive')
        self.raster = raster
        self._coverable = coverable.astype(bool)
        self._region = self._coverable if region is None else region.astype(bool)
        self._tool = tool_radius_m
        self._covered = np.zeros_like(self._coverable)
        self._reach = int(np.ceil(tool_radius_m / raster.resolution)) + 1

    @property
    def covered_cells(self) -> np.ndarray:
        """Boolean mask of the cells marked so far (a copy-free view; do not modify)."""
        return self._covered

    @property
    def coverable_count(self) -> int:
        return int(self._coverable.sum())

    @property
    def covered_count(self) -> int:
        return int((self._covered & self._coverable).sum())

    @property
    def covered_ratio(self) -> float:
        total = self.coverable_count
        return 1.0 if total == 0 else self.covered_count / total

    def mark_pose(self, x: float, y: float) -> int:
        """Mark the cells whose centre is within the tool radius of (x, y); returns the newly covered
        coverable cells."""
        rr, cc = self.raster.world_to_cell(x, y)
        r0, r1 = max(0, rr - self._reach), min(self.raster.height, rr + self._reach + 1)
        c0, c1 = max(0, cc - self._reach), min(self.raster.width, cc + self._reach + 1)
        if r0 >= r1 or c0 >= c1:
            return 0
        rows = np.arange(r0, r1)[:, None]
        cols = np.arange(c0, c1)[None, :]
        cx = self.raster.origin_x + (cols + 0.5) * self.raster.resolution
        cy = self.raster.origin_y + (rows + 0.5) * self.raster.resolution
        disc = (cx - x) ** 2 + (cy - y) ** 2 <= self._tool ** 2 + 1e-12
        window = self._covered[r0:r1, c0:c1]
        disc &= self._region[r0:r1, c0:c1]          # walls and other rooms are never "covered"
        fresh = disc & ~window & self._coverable[r0:r1, c0:c1]
        window |= disc
        return int(fresh.sum())

    def mark_segment(self, start: Tuple[float, float], end: Tuple[float, float],
                     step_m: Optional[float] = None) -> int:
        """Mark the poses along a straight segment (a pose stream sampled too sparsely)."""
        step = step_m or self.raster.resolution
        length = float(np.hypot(end[0] - start[0], end[1] - start[1]))
        n = max(1, int(np.ceil(length / step)))
        return sum(self.mark_pose(start[0] + (end[0] - start[0]) * i / n,
                                  start[1] + (end[1] - start[1]) * i / n) for i in range(n + 1))

    def covered_grid(self) -> np.ndarray:
        """int8 grid for `/myRobot/coverage/covered`: 100 covered, 0 not covered, -1 outside the region."""
        grid = np.full(self.raster.cells.shape, -1, dtype=np.int8)
        grid[self._region] = 0
        grid[self._covered & self._region] = 100
        return grid
