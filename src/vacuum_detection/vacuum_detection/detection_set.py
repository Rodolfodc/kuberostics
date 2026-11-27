"""Persistent set of detections with merge by label and proximity (spec 009a-R13). Pure Python, no ROS (D-18).

Replaces `find_matching_detection` of the 2025S2 producer: one entry per object. The set only stores and
merges; what is published and when is decided by the caller.
"""
import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Detection:
    det_id: int
    label: str
    x: float
    y: float
    z: float
    stamp: float                # stamp of the first photo that saw it
    confidence: float
    count: int = 1              # observations merged into this entry


class DetectionSet:
    def __init__(self, merge_distance_m: float = 0.5):
        if merge_distance_m < 0.0:
            raise ValueError('merge_distance_m must be >= 0')
        self.merge_distance_m = merge_distance_m
        self._items: List[Detection] = []
        self._next_id = 0

    def __len__(self) -> int:
        return len(self._items)

    def items(self) -> List[Detection]:
        return list(self._items)

    def add(self, label: str, x: float, y: float, z: float, stamp: float,
            confidence: float) -> Tuple[Detection, bool]:
        """Merge into the nearest same-label entry within the threshold (running mean of the position),
        else create one. Returns (entry, is_new). Non-finite positions are a caller bug (ValueError)."""
        if not all(math.isfinite(c) for c in (x, y, z)):
            raise ValueError('non-finite position')
        best = None
        for d in self._items:
            if d.label != label:
                continue
            dist = math.hypot(d.x - x, d.y - y)
            if dist <= self.merge_distance_m and (best is None or dist < best[0]):
                best = (dist, d)
        if best is not None:
            d = best[1]
            n = d.count
            d.x, d.y, d.z = ((d.x * n + x) / (n + 1), (d.y * n + y) / (n + 1), (d.z * n + z) / (n + 1))
            d.count = n + 1
            d.confidence = max(d.confidence, confidence)
            return d, False
        d = Detection(self._next_id, label, x, y, z, stamp, confidence)
        self._next_id += 1
        self._items.append(d)
        return d, True
