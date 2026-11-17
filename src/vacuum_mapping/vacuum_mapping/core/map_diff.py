"""MapDiff: divergence between a regenerated occupancy raster and the committed one (003-R34).

WHY: every mapping run produces a slightly different raster. The project keeps the first accepted one
and a history of later runs; this class says how far a new run drifted, without touching any file.
Pure computation (D-18): rasters in, a `MapDiffReport` out; reading files is `map_io`, printing is the CLI.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .types import OCCUPIED, UNKNOWN, Logger, Raster, null_logger

_EPS = 1e-9
# Upper bound of the pairwise-distance block (floats); keeps memory near 80 MB in the worst case.
_BLOCK_ELEMENTS = 10_000_000


@dataclass(frozen=True)
class MapDiffReport:
    """Result of comparing a `new` raster with the `committed` one."""
    pixel_diff_percent: float
    compared_cells: int
    differing_cells: int
    comparison_resolution_m: float
    resolution_committed: float
    resolution_new: float
    origin_committed: Tuple[float, float]
    origin_new: Tuple[float, float]
    size_committed: Tuple[int, int]  # (width, height) in cells
    size_new: Tuple[int, int]
    median_occupied_distance_m: Optional[float]  # None when either map has no occupied cell

    @property
    def resolution_changed(self) -> bool:
        return not math.isclose(self.resolution_committed, self.resolution_new, rel_tol=1e-9)

    @property
    def origin_shift_m(self) -> Tuple[float, float]:
        """new origin minus committed origin."""
        return (self.origin_new[0] - self.origin_committed[0],
                self.origin_new[1] - self.origin_committed[1])

    @property
    def origin_changed(self) -> bool:
        dx, dy = self.origin_shift_m
        return abs(dx) > 1e-6 or abs(dy) > 1e-6

    @property
    def size_changed(self) -> bool:
        return self.size_committed != self.size_new


def _resample(raster: Raster, x0: float, y0: float, res: float, rows: int, cols: int) -> np.ndarray:
    """`raster` seen on the comparison grid; cells outside the raster are UNKNOWN (003-R34)."""
    xs = x0 + (np.arange(cols) + 0.5) * res
    ys = y0 + (np.arange(rows) + 0.5) * res
    ix = np.floor((xs - raster.origin_x) / raster.resolution + _EPS).astype(np.int64)
    iy = np.floor((ys - raster.origin_y) / raster.resolution + _EPS).astype(np.int64)
    ok_x = (ix >= 0) & (ix < raster.width)
    ok_y = (iy >= 0) & (iy < raster.height)
    out = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    sub = raster.cells[np.ix_(np.clip(iy, 0, raster.height - 1), np.clip(ix, 0, raster.width - 1))]
    out[np.ix_(ok_y, ok_x)] = sub[np.ix_(ok_y, ok_x)]
    return out


def _occupied_points(raster: Raster) -> np.ndarray:
    """World coordinates (N, 2) of the centres of the occupied cells."""
    rows, cols = np.nonzero(raster.cells == OCCUPIED)
    return np.stack([raster.origin_x + (cols + 0.5) * raster.resolution,
                     raster.origin_y + (rows + 0.5) * raster.resolution], axis=1)


def _nearest_distances(points: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Distance from each point to the nearest reference point (exact, blockwise, numpy only)."""
    block = max(1, _BLOCK_ELEMENTS // max(1, len(reference)))
    out = np.empty(len(points))
    for start in range(0, len(points), block):
        chunk = points[start:start + block]
        d2 = ((chunk[:, None, :] - reference[None, :, :]) ** 2).sum(axis=2)
        out[start:start + block] = np.sqrt(d2.min(axis=1))
    return out


class MapDiff:
    """Compares two rasters in the world frame. Only computes; the caller decides what to do with it."""

    def __init__(self, log: Logger = null_logger) -> None:
        self._log = log

    def compare(self, committed: Raster, new: Raster) -> MapDiffReport:
        """Pixel difference over the union of the extents, plus geometry changes and the 003-R08 median.

        Compared on the finer of the two resolutions; a cell differs when its class (occupied, free,
        unknown) differs; a cell outside a map is unknown in that map. The median is the one of 003-R08:
        for each occupied cell of `new`, the distance to the nearest occupied cell of `committed`.
        """
        res = min(committed.resolution, new.resolution)
        x0 = min(committed.origin_x, new.origin_x)
        y0 = min(committed.origin_y, new.origin_y)
        x1 = max(committed.origin_x + committed.width * committed.resolution,
                 new.origin_x + new.width * new.resolution)
        y1 = max(committed.origin_y + committed.height * committed.resolution,
                 new.origin_y + new.height * new.resolution)
        cols = max(1, int(math.ceil((x1 - x0) / res - _EPS)))
        rows = max(1, int(math.ceil((y1 - y0) / res - _EPS)))

        a = _resample(committed, x0, y0, res, rows, cols)
        b = _resample(new, x0, y0, res, rows, cols)
        differing = int(np.count_nonzero(a != b))

        median: Optional[float] = None
        ref_pts, new_pts = _occupied_points(committed), _occupied_points(new)
        if len(ref_pts) and len(new_pts):
            median = float(np.median(_nearest_distances(new_pts, ref_pts)))

        report = MapDiffReport(
            pixel_diff_percent=100.0 * differing / (rows * cols),
            compared_cells=rows * cols,
            differing_cells=differing,
            comparison_resolution_m=res,
            resolution_committed=committed.resolution,
            resolution_new=new.resolution,
            origin_committed=(committed.origin_x, committed.origin_y),
            origin_new=(new.origin_x, new.origin_y),
            size_committed=(committed.width, committed.height),
            size_new=(new.width, new.height),
            median_occupied_distance_m=median,
        )
        self._log('map_diff: pixel difference %.2f %% (%d of %d cells)'
                  % (report.pixel_diff_percent, differing, rows * cols))
        return report


def format_report(report: MapDiffReport) -> str:
    """Markdown text of the divergence report (003-R34). Route, parameters and commit are added by the caller."""
    r = report
    lines = ['# Map divergence report', '']
    lines.append('- Pixel difference: %.2f %% (%d of %d cells, compared at %g m over the union of both extents)'
                 % (r.pixel_diff_percent, r.differing_cells, r.compared_cells, r.comparison_resolution_m))
    if r.resolution_changed:
        lines.append('- Resolution: %g m -> %g m (changed)' % (r.resolution_committed, r.resolution_new))
    else:
        lines.append('- Resolution: %g m (unchanged)' % r.resolution_new)
    if r.origin_changed:
        dx, dy = r.origin_shift_m
        lines.append('- Origin: (%s, %s) -> (%s, %s), shift (%+.3f, %+.3f) m (changed)'
                     % (r.origin_committed + r.origin_new + (dx, dy)))
    else:
        lines.append('- Origin: (%s, %s) (unchanged)' % r.origin_new)
    if r.size_changed:
        lines.append('- Size: %d x %d -> %d x %d cells (changed)' % (r.size_committed + r.size_new))
    else:
        lines.append('- Size: %d x %d cells (unchanged)' % r.size_new)
    if r.median_occupied_distance_m is None:
        lines.append('- Median occupied-cell distance (003-R08): not computable (a map has no occupied cell)')
    else:
        lines.append('- Median occupied-cell distance (003-R08): %.3f m' % r.median_occupied_distance_m)
    return '\n'.join(lines) + '\n'
