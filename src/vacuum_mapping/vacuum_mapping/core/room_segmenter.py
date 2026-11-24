"""RoomSegmenter: occupancy raster -> raw room label raster (003-R16).

WHY: rooms are the unit of coverage (spec 005) and of labels. A door is the only place where free space
is narrower than `door_width_m`, so eroding the free space by half a door width cuts rooms apart at
doors; each surviving blob is a room seed and the original free cells then join the nearest seed.
numpy only (Q-16 fallback): the erosion is a stack of shifted comparisons, the two flood fills are
plain breadth-first searches, so there is no scipy/OpenCV dependency to resolve on both distros.

Pure computation (D-18): raster in, label raster out. To add another method, see the `SegmentationMethod`
protocol and the README.
"""
from __future__ import annotations

from collections import deque
from typing import Deque, Protocol, Tuple

import numpy as np

from .types import FREE, Logger, Raster, null_logger


class SegmentationMethod(Protocol):
    """What the node needs from any segmentation method: raster in, int32 labels out (0 = no room,
    rooms numbered 1..N consecutively in scan order, same shape as the raster, deterministic)."""

    def segment(self, raster: Raster) -> np.ndarray:
        ...


def erode(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Cells of `mask` whose whole disc of `radius_cells` lies inside `mask` (outside the array = not in mask)."""
    if radius_cells <= 0:
        return mask.copy()
    h, w = mask.shape
    r = radius_cells
    padded = np.pad(mask, r, constant_values=False)
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dy * dy + dx * dx <= r * r and (dy or dx):
                out &= padded[r + dy:r + dy + h, r + dx:r + dx + w]
    return out


def _neighbours(width: int) -> Tuple[int, int, int, int]:
    return (-width, width, -1, 1)


def label_components(mask: np.ndarray) -> Tuple[np.ndarray, int]:
    """4-connected components of `mask`, numbered 1..N in scan order of their first cell."""
    h, w = mask.shape
    pw = w + 2  # one-cell border of False so the search needs no bounds checks
    flat = np.pad(mask, 1, constant_values=False).ravel().tolist()
    labels = [0] * len(flat)
    steps = _neighbours(pw)
    count = 0
    for start in range(len(flat)):
        if flat[start] and not labels[start]:
            count += 1
            labels[start] = count
            stack = [start]
            while stack:
                cell = stack.pop()
                for step in steps:
                    nxt = cell + step
                    if flat[nxt] and not labels[nxt]:
                        labels[nxt] = count
                        stack.append(nxt)
    out = np.asarray(labels, dtype=np.int32).reshape(h + 2, pw)[1:-1, 1:-1]
    return out, count


def grow_labels(seeds: np.ndarray, free: np.ndarray) -> np.ndarray:
    """Multi-source breadth-first growth of `seeds` through `free` cells (4-connected, nearest seed wins).

    Ties are broken by queue order, which is scan order of the seed cells, so the result is deterministic.
    """
    h, w = seeds.shape
    pw = w + 2
    can = np.pad(free, 1, constant_values=False).ravel().tolist()
    labels = np.pad(seeds, 1, constant_values=0).ravel().tolist()
    queue: Deque[int] = deque(i for i, v in enumerate(labels) if v)
    steps = _neighbours(pw)
    while queue:
        cell = queue.popleft()
        value = labels[cell]
        for step in steps:
            nxt = cell + step
            if can[nxt] and not labels[nxt]:
                labels[nxt] = value
                queue.append(nxt)
    return np.asarray(labels, dtype=np.int32).reshape(h + 2, pw)[1:-1, 1:-1].copy()


class RoomSegmenter:
    """Erosion by half a door width, components as seeds, nearest-seed growth (003-R16)."""

    def __init__(self, door_width_m: float = 0.9, min_room_area_m2: float = 2.0,
                 log: Logger = null_logger) -> None:
        if door_width_m <= 0:
            raise ValueError('door_width_m must be positive')
        if min_room_area_m2 < 0:
            raise ValueError('min_room_area_m2 must not be negative')
        self._door_width_m = door_width_m
        self._min_area_m2 = min_room_area_m2
        self._log = log

    def segment(self, raster: Raster) -> np.ndarray:
        """Label raster (int32): 0 on occupied, unknown and unreachable cells, rooms numbered from 1.

        Occupied and unknown cells both block growth, so a room never leaks through unexplored space.
        The erosion radius is `door_width_m / 2` rounded to whole cells.
        """
        free = raster.cells == FREE
        radius = int(round(self._door_width_m / 2.0 / raster.resolution))
        components, n_found = label_components(erode(free, radius))

        cell_area = raster.resolution ** 2
        sizes = np.bincount(components.ravel(), minlength=n_found + 1)
        keep = [k for k in range(1, n_found + 1) if sizes[k] * cell_area >= self._min_area_m2]
        renumber = np.zeros(n_found + 1, dtype=np.int32)
        for new_label, old_label in enumerate(keep, start=1):
            renumber[old_label] = new_label  # keeps scan order, so numbering is stable
        seeds = renumber[components]

        labels = grow_labels(seeds, free)
        self._log('room_segmenter: %d rooms (erosion radius %d cells, %d components below %.2f m2 dropped)'
                  % (len(keep), radius, n_found - len(keep), self._min_area_m2))
        return labels
