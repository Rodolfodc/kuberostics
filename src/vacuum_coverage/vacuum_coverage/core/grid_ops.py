"""Grid helpers of the coverage cores: clearance, dilation, reachable and coverable masks.

WHY: planner and tracker must agree on what the robot can reach and what the tool can clean, otherwise the
`covered_ratio` of 005-R21 would be measured against cells no lane can ever touch. Erosion and components
come from vacuum_mapping (not duplicated); a shared `vacuum_grid` package is planned by T309 of spec 003.
Pure Python plus numpy, no ROS (D-18).
"""
from __future__ import annotations

import math

import numpy as np

from vacuum_mapping.core.room_segmenter import erode
from vacuum_mapping.core.types import FREE, Raster


def radius_to_cells(radius_m: float, resolution: float) -> int:
    """Whole cells covering `radius_m` (rounded up, so the clearance is never smaller than asked)."""
    return max(0, int(math.ceil(radius_m / resolution - 1e-9)))


def dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Cells within `radius_cells` (Euclidean) of a True cell of `mask`; outside the array counts as False."""
    if radius_cells <= 0:
        return mask.copy()
    h, w = mask.shape
    r = radius_cells
    padded = np.pad(mask, r, constant_values=False)
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dy * dy + dx * dx <= r * r and (dy or dx):
                out |= padded[r + dy:r + dy + h, r + dx:r + dx + w]
    return out


def reachable_mask(raster: Raster, clearance_m: float) -> np.ndarray:
    """Cells where the robot centre may be: FREE with a disc of `clearance_m` entirely FREE.

    UNKNOWN and OCCUPIED both block, and so does the edge of the array.
    """
    return erode(raster.cells == FREE, radius_to_cells(clearance_m, raster.resolution))


def coverable_mask(raster: Raster, region: np.ndarray, clearance_m: float, tool_radius_m: float,
                   lane_spacing_m: float = 0.0) -> np.ndarray:
    """Cells of `region` that lanes can be guaranteed to clean: the reachable cells (where the robot centre
    goes) plus a band around them.

    The band is NOT the full tool radius. A cell next to the end of a lane, midway between two lanes, is
    `lane_spacing_m / 2` to the side of the nearest lane, so only `sqrt(tool^2 - (spacing/2)^2)` of the tool
    reaches past the lane end (a floor to whole cells keeps it conservative). Without it, 5 % of the cells
    along the walls could never be reached by any lane and the 95 % of 005-R18 would be a coin toss. The
    strip between a wall and the first lane (robot radius + margin from the wall) is left out for the same
    reason. With `lane_spacing_m` 0 (unknown) the band is the full tool radius.
    """
    reach = reachable_mask(raster, clearance_m) & region
    half = lane_spacing_m / 2.0
    band_m = math.sqrt(max(tool_radius_m ** 2 - half ** 2, 0.0))
    cells = int(math.floor(band_m / raster.resolution + 1e-9))
    return region & dilate(reach, cells)
