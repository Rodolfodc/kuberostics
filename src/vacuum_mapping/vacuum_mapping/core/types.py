"""Shared value types of the mapping cores.

WHY: every core (MapDiff, RoomSegmenter, RoomTracker, RoomLabeler) must agree on one grid layout and one
set of cell codes, and none of them may know about ROS messages or files. These dataclasses are that
agreement; the node and the file readers convert to and from them.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Callable, Tuple

import numpy as np

# Cell codes, identical to nav_msgs/OccupancyGrid after thresholding.
UNKNOWN = -1
FREE = 0
OCCUPIED = 100

# D-20: pure logic reports through an injected callback, never through a global logger.
Logger = Callable[[str], None]


def null_logger(_message: str) -> None:
    """Default logger: drops the message."""


@dataclass(frozen=True)
class Raster:
    """Occupancy raster in the ROS layout: row 0 is the lowest y, column 0 the lowest x.

    `cells` holds UNKNOWN / FREE / OCCUPIED (int8). The origin is the world position of the lower-left
    corner of cell (0, 0); the map yaw is not supported (slam_toolbox maps have yaw 0).
    """
    cells: np.ndarray
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0

    def __post_init__(self) -> None:
        if self.cells.ndim != 2:
            raise ValueError('cells must be a 2-D array')
        if self.resolution <= 0:
            raise ValueError('resolution must be positive')

    @property
    def height(self) -> int:
        return int(self.cells.shape[0])

    @property
    def width(self) -> int:
        return int(self.cells.shape[1])

    def cell_to_world(self, row: float, col: float) -> Tuple[float, float]:
        """World coordinates of the CENTRE of cell (row, col)."""
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """(row, col) of the cell containing the world point; may lie outside the raster."""
        return (int(np.floor((y - self.origin_y) / self.resolution)),
                int(np.floor((x - self.origin_x) / self.resolution)))


def classify_occupancy(values: np.ndarray, occupied_thresh: int = 65, free_thresh: int = 25) -> np.ndarray:
    """Turn OccupancyGrid values (-1, 0..100) into the three cell codes.

    WHY: slam_toolbox publishes probabilities; the cores reason on three classes only. Values between
    the thresholds count as unknown, the conservative choice for segmentation and diffing.
    """
    out = np.full(values.shape, UNKNOWN, dtype=np.int8)
    out[(values >= 0) & (values <= free_thresh)] = FREE
    out[values >= occupied_thresh] = OCCUPIED
    return out


class LabelSource(enum.IntEnum):
    """Mirrors the `Room.msg` constants DEFAULT, RULE, USER (proposed contract of 003)."""
    DEFAULT = 0
    RULE = 1
    USER = 2


@dataclass(frozen=True)
class Room:
    """One room, mirror of the proposed `vacuum_interfaces/Room` (003-R20)."""
    index: int
    id: str
    label: str
    label_source: LabelSource
    centroid: Tuple[float, float]
    area_m2: float


def room_id(index: int) -> str:
    """`room_%02d` of the index (003-R20)."""
    return 'room_%02d' % index
