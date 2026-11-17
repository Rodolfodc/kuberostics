"""RoomTracker: keeps room identity stable across segmentations (003-R19, R20).

WHY: coverage plans and user labels refer to `Room.index`/`id`. A new map or a re-run of the segmenter
renumbers raw labels in scan order, so each new room is matched to the previous room it overlaps most.

No hidden state (D-19): the previous result is passed in and the new one returned, including
`next_index`, the counter that guarantees indexes are never reused.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .types import LabelSource, Logger, Room, null_logger, room_id


@dataclass(frozen=True)
class TrackedMap:
    """Segmentation with stable indexes: `labels` holds `Room.index` (0 = no room)."""
    labels: np.ndarray
    rooms: Tuple[Room, ...]
    next_index: int = 1

    @staticmethod
    def empty() -> 'TrackedMap':
        """Starting state: no rooms, first index will be 1."""
        return TrackedMap(np.zeros((0, 0), dtype=np.int32), (), 1)


class RoomTracker:
    """One-to-one matching of new rooms to previous rooms by overlap, largest first."""

    def __init__(self, min_overlap: float = 0.5, log: Logger = null_logger) -> None:
        self._min_overlap = min_overlap
        self._log = log

    def track(self, previous: Optional[TrackedMap], raw_labels: np.ndarray,
              resolution: float, origin_x: float, origin_y: float) -> TrackedMap:
        """Assign persistent indexes to the rooms of `raw_labels` (output of a segmentation method).

        A new room keeps index, id, label and label_source of the previous room with the largest overlap,
        if that overlap is at least `min_overlap` of the NEW room's area (003-R19). `previous` must use the
        same grid geometry as `raw_labels`; pass None for the first segmentation.
        """
        previous = previous or TrackedMap.empty()
        prev_by_index: Dict[int, Room] = {r.index: r for r in previous.rooms}
        raw_ids = [int(v) for v in np.unique(raw_labels) if v > 0]
        new_area = {k: int(np.count_nonzero(raw_labels == k)) for k in raw_ids}

        pairs: List[Tuple[int, int, int]] = []  # (-overlap cells, previous index, raw label)
        if previous.rooms and previous.labels.shape != raw_labels.shape:
            self._log('room_tracker: WARNING previous labels %s and new labels %s differ in shape (map grew?); '
                      'labels cannot be carried over, every room gets a new index'
                      % (previous.labels.shape, raw_labels.shape))
        if previous.labels.shape == raw_labels.shape:
            both = (previous.labels > 0) & (raw_labels > 0)
            keys, counts = np.unique(
                np.stack([previous.labels[both], raw_labels[both]], axis=1), axis=0, return_counts=True)
            for (p, k), n in zip(keys.tolist(), counts.tolist()):
                if p in prev_by_index and n >= self._min_overlap * new_area[k]:
                    pairs.append((-n, p, k))
        pairs.sort()  # largest overlap first; ties: lowest previous index, then lowest raw label

        assigned: Dict[int, int] = {}  # raw label -> previous index
        taken = set()
        for _neg, p, k in pairs:
            if k not in assigned and p not in taken:
                assigned[k] = p
                taken.add(p)

        next_index = previous.next_index
        index_of: Dict[int, int] = {}
        for k in raw_ids:  # raw labels are in scan order, so new indexes are too
            if k in assigned:
                index_of[k] = assigned[k]
            else:
                index_of[k] = next_index
                next_index += 1

        lut = np.zeros(max(raw_ids, default=0) + 1, dtype=np.int32)
        for k, idx in index_of.items():
            lut[k] = idx
        labels = lut[np.where(raw_labels > 0, raw_labels, 0)]

        rooms = tuple(sorted((self._make_room(labels, index_of[k], prev_by_index.get(assigned.get(k, -1)),
                                              resolution, origin_x, origin_y) for k in raw_ids),
                             key=lambda r: r.index))
        kept = len(assigned)
        self._log('room_tracker: %d rooms (%d kept, %d new), %d previous retired, next index %d'
                  % (len(rooms), kept, len(rooms) - kept, len(prev_by_index) - kept, next_index))
        return TrackedMap(labels, rooms, next_index)

    @staticmethod
    def _make_room(labels: np.ndarray, index: int, prior: Optional[Room],
                   resolution: float, origin_x: float, origin_y: float) -> Room:
        rows, cols = np.nonzero(labels == index)
        centroid = (origin_x + (float(cols.mean()) + 0.5) * resolution,
                    origin_y + (float(rows.mean()) + 0.5) * resolution)
        area = float(len(rows)) * resolution * resolution
        if prior is not None:
            return dataclasses.replace(prior, centroid=centroid, area_m2=area)
        rid = room_id(index)
        return Room(index, rid, rid, LabelSource.DEFAULT, centroid, area)
