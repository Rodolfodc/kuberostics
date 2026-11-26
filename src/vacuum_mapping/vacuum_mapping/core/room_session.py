"""RoomSession: the state behind the `room_segmenter` node, with no ROS (D-18).

WHY: segmentation, tracking, rule labels, user labels and the rooms file must work together, and be tested
without a graph. The node only converts messages to a `Raster` and publishes what this returns.

Identity across segmentations (003-R19): `RoomTracker` matches by overlap of label rasters, which needs the
same grid shape. The session therefore re-seeds the "previous" result from room CENTROIDS whenever it has no
usable one (restart, or the map grew): each saved room is looked up at its centroid in the new raw
segmentation, and that raw room inherits its index, label and label source. A restart and a grown map thus keep
labels. User labels are persisted on every change (003-R23).
"""
from __future__ import annotations

import dataclasses
import os
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from ..rooms_io import RoomsFile, read_rooms, write_rooms
from .room_labeler import Observation, RoomLabeler, Rule
from .room_segmenter import RoomSegmenter
from .room_tracker import RoomTracker, TrackedMap
from .types import LabelSource, Logger, Raster, Room, null_logger



class RoomSession:
    def __init__(self, rooms_path: str, map_name: str, rules: Mapping[str, Rule],
                 door_width_m: float = 0.9, min_room_area_m2: float = 2.0, min_votes: int = 2,
                 start_pose: Optional[Tuple[float, float, float]] = None, log: Logger = null_logger) -> None:
        self._path = rooms_path
        self._map_name = map_name
        self._start_pose = start_pose
        self._log = log
        self._segmenter = RoomSegmenter(door_width_m, min_room_area_m2, log=log)
        self._tracker = RoomTracker(log=log)
        self._labeler = RoomLabeler(rules, min_votes=min_votes, log=log)
        self._tracked = TrackedMap.empty()
        self._raster: Optional[Raster] = None
        self._observations: List[Observation] = []
        if os.path.exists(rooms_path):
            saved = read_rooms(rooms_path)
            self._tracked = TrackedMap(np.zeros((0, 0), dtype=np.int32), saved.rooms, saved.next_index)
            self._start_pose = saved.start_pose if saved.start_pose is not None else start_pose
            log('room_session: loaded %d rooms from %s' % (len(saved.rooms), rooms_path))

    @property
    def rooms(self) -> Tuple[Room, ...]:
        return self._tracked.rooms

    @property
    def labels(self) -> np.ndarray:
        return self._tracked.labels

    def segment(self, raster: Raster) -> TrackedMap:
        raw = self._segmenter.segment(raster)
        previous = self._tracked
        if previous.rooms and previous.labels.shape != raw.shape:
            previous = self._seed_from_centroids(previous, raw, raster)
        tracked = self._tracker.track(previous, raw, raster.resolution, raster.origin_x, raster.origin_y)
        self._raster = raster
        self._tracked = tracked
        self._apply_observations()
        self._persist()
        return self._tracked

    def set_observations(self, observations: Sequence[Observation]) -> None:
        """Replace the current set of detections (the caller deduplicates by marker id) and relabel."""
        self._observations = list(observations)
        if self._raster is not None and self._apply_observations():
            self._persist()

    def set_label(self, room_id: str, label: str) -> Tuple[bool, str]:
        """003-R23: user label; unknown id or empty label is rejected and nothing changes."""
        if not label.strip():
            return False, 'empty label'
        for i, room in enumerate(self._tracked.rooms):
            if room.id == room_id:
                rooms = list(self._tracked.rooms)
                rooms[i] = dataclasses.replace(room, label=label, label_source=LabelSource.USER)
                self._tracked = dataclasses.replace(self._tracked, rooms=tuple(rooms))
                self._log('room_session: %s labelled %r by the user' % (room_id, label))
                self._persist()
                return True, 'ok'
        return False, 'unknown room id %r' % room_id

    # ------------------------------------------------------------------ internals
    def _apply_observations(self) -> bool:
        if self._raster is None or not self._tracked.rooms:
            return False
        r = self._raster
        new = self._labeler.label_rooms(self._tracked.rooms, self._tracked.labels, r.resolution,
                                        r.origin_x, r.origin_y, self._observations)
        changed = new != self._tracked.rooms
        self._tracked = dataclasses.replace(self._tracked, rooms=new)
        return changed

    def _seed_from_centroids(self, previous: TrackedMap, raw: np.ndarray, raster: Raster) -> TrackedMap:
        by_raw: Dict[int, Room] = {}
        for room in sorted(previous.rooms, key=lambda r: r.index):
            row, col = raster.world_to_cell(*room.centroid)
            if 0 <= row < raw.shape[0] and 0 <= col < raw.shape[1] and raw[row, col] > 0:
                by_raw.setdefault(int(raw[row, col]), room)
        lut = np.zeros(int(raw.max()) + 1 if raw.size else 1, dtype=np.int32)
        for k, room in by_raw.items():
            lut[k] = room.index
        labels = lut[np.where(raw > 0, raw, 0)]
        self._log('room_session: previous labels not usable on this grid; %d of %d rooms re-seeded by centroid'
                  % (len(by_raw), len(previous.rooms)))
        return TrackedMap(labels, tuple(by_raw.values()), previous.next_index)

    def _persist(self) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(self._path)), exist_ok=True)
        write_rooms(self._path, RoomsFile(self._map_name, self._start_pose, self._tracked.next_index,
                                          self._tracked.rooms))
