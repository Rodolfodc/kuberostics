"""GridView: read-only view of an occupancy grid with world/cell conversion and obstacle inflation.

WHY: mapping (frontiers) and coverage (planner) need the same grid arithmetic. Keeping it here, with no
ROS imports, lets both packages share it without depending on each other (Art. III.1).

Conventions (identical to `vacuum_mapping.core.types.Raster`, and pinned by a test there):
row 0 is the lowest y, column 0 the lowest x; `origin` is the world position of the lower-left corner of
cell (0, 0); `cell_to_world` returns the cell CENTRE; `world_to_cell` uses floor and may return indexes
outside the grid. Cell codes are those of `nav_msgs/OccupancyGrid` after thresholding.

Only computes (D-19): no publishing, no policy.
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


class GridView:
    """Immutable-by-convention view of a three-class occupancy array (int8: -1, 0, 100)."""

    def __init__(self, cells: np.ndarray, resolution: float,
                 origin_x: float = 0.0, origin_y: float = 0.0) -> None:
        if cells.ndim != 2:
            raise ValueError('cells must be a 2-D array')
        if resolution <= 0:
            raise ValueError('resolution must be positive')
        self.cells = cells
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

    @property
    def height(self) -> int:
        return int(self.cells.shape[0])

    @property
    def width(self) -> int:
        return int(self.cells.shape[1])

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def cell_to_world(self, row: float, col: float) -> Tuple[float, float]:
        """World coordinates (x, y) of the CENTRE of cell (row, col)."""
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """(row, col) of the cell containing the world point; may lie outside the grid."""
        return (int(np.floor((y - self.origin_y) / self.resolution)),
                int(np.floor((x - self.origin_x) / self.resolution)))

    def inflated_obstacles(self, radius_m: float, include_unknown: bool = False) -> np.ndarray:
        """Boolean mask of the cells whose centre lies within `radius_m` of an obstacle cell.

        Obstacles are OCCUPIED cells; UNKNOWN cells only when `include_unknown` (default False: the
        frontier logic needs free cells next to unknown space to stay usable). The obstacle cells
        themselves are in the mask. The grid border is NOT an obstacle. A radius of 0 or less returns
        just the obstacle cells. The disk uses Euclidean distance in cells, `dr*dr + dc*dc <= (r/res)^2`.
        """
        obstacle = self.cells == OCCUPIED
        if include_unknown:
            obstacle = obstacle | (self.cells == UNKNOWN)
        reach = int(np.floor(max(radius_m, 0.0) / self.resolution + 1e-9))
        if reach == 0:
            return obstacle
        out = obstacle.copy()
        h, w = obstacle.shape
        limit = (radius_m / self.resolution) ** 2 + 1e-9
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                if (dr == 0 and dc == 0) or dr * dr + dc * dc > limit or abs(dr) >= h or abs(dc) >= w:
                    continue
                src_r = slice(max(0, -dr), min(h, h - dr))
                src_c = slice(max(0, -dc), min(w, w - dc))
                dst_r = slice(max(0, dr), min(h, h + dr))
                dst_c = slice(max(0, dc), min(w, w + dc))
                out[dst_r, dst_c] |= obstacle[src_r, src_c]
        return out
