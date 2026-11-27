"""RegionBuilder: which cells a CoverRoom goal has to clean (005-R16). Pure Python, no ROS (D-18).

Three sources, in order of preference: the cells of a named room, the room under the robot, and, when the
room topics have not been received, the free cells connected to the robot capped by `max_region_area_m2`.
The caller reads the topics and passes what it has; this class only computes.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from vacuum_mapping.core.types import FREE, Logger, Raster, Room, null_logger

# Nearest free cell search around a robot standing on a wall or unknown cell (localisation error).
SNAP_RADIUS_M = 0.5


class UnknownRoom(ValueError):
    """The goal names a room that does not exist (or no room topics were received): reject the goal."""


class NoRegion(ValueError):
    """There is no free cell near the robot to start from."""


@dataclass(frozen=True)
class Region:
    """`mask` (bool, raster shape) plus how it was obtained, for the log and the telemetry."""
    mask: np.ndarray
    source: str                       # 'room' | 'robot_room' | 'connected'
    room_index: Optional[int] = None
    truncated: bool = False


class RegionBuilder:
    def __init__(self, max_region_area_m2: float = 150.0, log: Logger = null_logger) -> None:
        if max_region_area_m2 <= 0:
            raise ValueError('max_region_area_m2 must be positive')
        self._max_area = max_region_area_m2
        self._log = log

    def build(self, raster: Raster, robot_xy: Tuple[float, float], room_id: str = '',
              rooms: Optional[Sequence[Room]] = None, labels: Optional[np.ndarray] = None) -> Region:
        """Region of a goal. `rooms` and `labels` are None until the room topics have been received."""
        have_rooms = rooms is not None and labels is not None
        free = raster.cells == FREE
        if room_id:
            if not have_rooms:
                raise UnknownRoom('room_id %r requested but no room topics were received' % room_id)
            match = [r for r in rooms if r.id == room_id]
            if not match:
                raise UnknownRoom('unknown room_id %r' % room_id)
            return self._room(free, labels, match[0].index, 'room')
        if have_rooms:
            index = self._room_under_robot(raster, labels, robot_xy)
            if index is not None:
                return self._room(free, labels, index, 'robot_room')
            self._log('WARN region_builder: no room label anywhere, using the connected free area')
        else:
            self._log('WARN region_builder: room topics not received, using the free area connected to the '
                      'robot (max %.0f m2)' % self._max_area)
        return self._connected(raster, free, robot_xy)

    def _room(self, free: np.ndarray, labels: np.ndarray, index: int, source: str) -> Region:
        mask = (labels == index) & free
        self._log('region_builder: %s index=%d cells=%d' % (source, index, int(mask.sum())))
        return Region(mask, source, index)

    @staticmethod
    def _room_under_robot(raster: Raster, labels: np.ndarray, robot_xy) -> Optional[int]:
        """Label at the robot cell; on a cell with no room, the label of the nearest labelled cell.

        Ties go to the smallest index so the answer is deterministic.
        """
        r, c = raster.world_to_cell(*robot_xy)
        if 0 <= r < raster.height and 0 <= c < raster.width and labels[r, c] > 0:
            return int(labels[r, c])
        rows, cols = np.nonzero(labels > 0)
        if rows.size == 0:
            return None
        d2 = (rows - r) ** 2 + (cols - c) ** 2
        best = d2 == d2.min()
        return int(labels[rows[best], cols[best]].min())

    def _connected(self, raster: Raster, free: np.ndarray, robot_xy) -> Region:
        start = self._snap(raster, free, robot_xy)
        cap = int(self._max_area / (raster.resolution ** 2))
        h, w = free.shape
        mask = np.zeros_like(free)
        mask[start] = True
        queue = deque([start])
        count = 1
        truncated = False
        while queue and not truncated:
            r, c = queue.popleft()
            for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):   # fixed order: deterministic
                if 0 <= nr < h and 0 <= nc < w and free[nr, nc] and not mask[nr, nc]:
                    if count >= cap:
                        truncated = True
                        break
                    mask[nr, nc] = True
                    count += 1
                    queue.append((nr, nc))
        if truncated:
            self._log('WARN region_builder: region capped at %.1f m2 (%d cells), BFS order from the robot'
                      % (self._max_area, count))
        self._log('region_builder: connected cells=%d start=%s' % (count, start))
        return Region(mask, 'connected', None, truncated)

    @staticmethod
    def _snap(raster: Raster, free: np.ndarray, robot_xy) -> Tuple[int, int]:
        r, c = raster.world_to_cell(*robot_xy)
        if 0 <= r < raster.height and 0 <= c < raster.width and free[r, c]:
            return r, c
        reach = int(np.ceil(SNAP_RADIUS_M / raster.resolution))
        r0, r1 = max(0, r - reach), min(raster.height, r + reach + 1)
        c0, c1 = max(0, c - reach), min(raster.width, c + reach + 1)
        rows, cols = np.nonzero(free[r0:r1, c0:c1])
        if rows.size == 0:
            raise NoRegion('no free cell within %.2f m of the robot' % SNAP_RADIUS_M)
        rows, cols = rows + r0, cols + c0
        d2 = (rows - r) ** 2 + (cols - c) ** 2
        i = int(np.lexsort((cols, rows, d2))[0])
        return int(rows[i]), int(cols[i])
