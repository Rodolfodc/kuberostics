"""FrontierFinder: where the known map meets the unknown (003-R09).

WHY: exploration needs candidate goals. A frontier cell is a known FREE cell with at least one UNKNOWN
cell among its 8 neighbours; 8-connected frontier cells form a cluster. Each cluster of at least
`min_frontier_cells` becomes a `Frontier` whose centroid is snapped to the nearest free cell that keeps
`robot_radius_m` of clearance from OCCUPIED cells (UNKNOWN does not count as an obstacle for clearance,
otherwise no frontier cell could ever qualify).

Only computes (D-19): grid, robot position and blacklist in, ordered candidates out. The caller picks one,
sends it and decides what to blacklist. Order: ascending `distance(robot, centroid) - gain_weight * size`
(metres and cells), ties broken by centroid x then y, so the result is deterministic.

The blacklist is a list of world points; a frontier is skipped when its (snapped) centroid is within
`blacklist_tolerance_m` of any of them. The tolerance and `max_snap_m` (frontiers with no qualifying free
cell within that distance are dropped and logged) are constructor arguments, not node parameters.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from vacuum_grid.grid_view import FREE, UNKNOWN, GridView

from .types import Logger, null_logger

Metrics = Callable[[Dict[str, float]], None]


def _no_metrics(_values: Dict[str, float]) -> None:
    """Default metrics callback: drops the values."""


@dataclass(frozen=True)
class Frontier:
    """One frontier cluster: snapped centroid in world coordinates and number of cells."""
    centroid: Tuple[float, float]
    size: int


class FrontierFinder:
    def __init__(self, min_frontier_cells: int = 8, gain_weight: float = 0.05, robot_radius_m: float = 0.16,
                 max_snap_m: float = 1.0, blacklist_tolerance_m: float = 0.3,
                 log: Logger = null_logger, metrics: Metrics = _no_metrics) -> None:
        self._min_cells = min_frontier_cells
        self._gain = gain_weight
        self._radius = robot_radius_m
        self._max_snap = max_snap_m
        self._tolerance = blacklist_tolerance_m
        self._log = log
        self._metrics = metrics

    def find(self, grid: GridView, robot_xy: Tuple[float, float],
             blacklist: Iterable[Tuple[float, float]] = ()) -> List[Frontier]:
        mask = self._frontier_mask(grid.cells)
        clusters = self._clusters(mask)
        inflated = grid.inflated_obstacles(self._radius)
        usable = (grid.cells == FREE) & ~inflated
        black = [(float(b[0]), float(b[1])) for b in blacklist]

        kept: List[Frontier] = []
        too_small = no_snap = blacklisted = 0
        for cells in clusters:
            if len(cells) < self._min_cells:
                too_small += 1
                continue
            snapped = self._snap(grid, usable, cells)
            if snapped is None:
                no_snap += 1
                self._log('frontier_finder: no free cell with %.2f m clearance within %.2f m of a %d-cell '
                          'cluster; snap failed, cluster dropped' % (self._radius, self._max_snap, len(cells)))
                continue
            if any(math.hypot(snapped[0] - b[0], snapped[1] - b[1]) <= self._tolerance for b in black):
                blacklisted += 1
                continue
            kept.append(Frontier(snapped, len(cells)))

        def cost(f: Frontier) -> Tuple[float, float, float]:
            d = math.hypot(f.centroid[0] - robot_xy[0], f.centroid[1] - robot_xy[1])
            return (round(d - self._gain * f.size, 9), f.centroid[0], f.centroid[1])

        kept.sort(key=cost)
        self._metrics({'clusters': float(len(clusters)), 'frontiers': float(len(kept)),
                       'too_small': float(too_small), 'snap_failed': float(no_snap),
                       'blacklisted': float(blacklisted)})
        return kept

    @staticmethod
    def _frontier_mask(cells: np.ndarray) -> np.ndarray:
        unknown = cells == UNKNOWN
        near_unknown = np.zeros_like(unknown)
        h, w = unknown.shape
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                near_unknown[max(0, dr):h + min(0, dr), max(0, dc):w + min(0, dc)] |= \
                    unknown[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
        return (cells == FREE) & near_unknown

    @staticmethod
    def _clusters(mask: np.ndarray) -> List[List[Tuple[int, int]]]:
        """8-connected components, numbered in scan order of their first cell."""
        seen = np.zeros_like(mask)
        out: List[List[Tuple[int, int]]] = []
        h, w = mask.shape
        for r, c in np.argwhere(mask):
            r, c = int(r), int(c)
            if seen[r, c]:
                continue
            seen[r, c] = True
            stack, members = [(r, c)], []
            while stack:
                cr, cc = stack.pop()
                members.append((cr, cc))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nr, nc = cr + dr, cc + dc
                        if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not seen[nr, nc]:
                            seen[nr, nc] = True
                            stack.append((nr, nc))
            out.append(members)
        return out

    def _snap(self, grid: GridView, usable: np.ndarray,
              cells: Sequence[Tuple[int, int]]) -> Optional[Tuple[float, float]]:
        rows = np.array([c[0] for c in cells], dtype=float)
        cols = np.array([c[1] for c in cells], dtype=float)
        mr, mc = float(rows.mean()), float(cols.mean())      # centroid in cell units (cell centres at +0.5)
        reach = int(math.ceil(self._max_snap / grid.resolution))
        r0, r1 = max(0, int(mr) - reach), min(grid.height, int(mr) + reach + 1)
        c0, c1 = max(0, int(mc) - reach), min(grid.width, int(mc) + reach + 1)
        window = np.argwhere(usable[r0:r1, c0:c1])
        if window.size == 0:
            return None
        window = window + np.array([r0, c0])
        dist2 = (window[:, 0] - mr) ** 2 + (window[:, 1] - mc) ** 2
        order = np.lexsort((window[:, 1], window[:, 0], np.round(dist2, 9)))
        best = window[order[0]]
        if math.sqrt(float(dist2[order[0]])) * grid.resolution > self._max_snap + 1e-9:
            return None
        return grid.cell_to_world(int(best[0]), int(best[1]))
