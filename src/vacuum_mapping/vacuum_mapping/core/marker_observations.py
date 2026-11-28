"""Adapter from `/detections_markers` markers to `Observation`s for RoomLabeler (003-R22).

WHY: the projector (spec 009a) publishes `visualization_msgs/MarkerArray` for the 2025S2 semantic node,
and re-publishes its WHOLE persistent set every ~2 s (specs/009a-offload/markers-format.md, sections 3 to 7).
Turning every message into new observations would count one object once per republish and make
`min_votes` meaningless. So observations are keyed by marker id (latest wins) and a repeated array
changes nothing.

Pure Python, duck-typed: a marker only needs `ns`, `id`, `text` and `pose.position.x/y`, so the ROS node
passes the messages as they are and the tests use `SimpleNamespace`. Parsing rule, identical to the
consumer (`_detections_cb`): `ns == 'detections_text'`, non-empty text, class = text before the first
`(`, stripped and lowercased; NaN/inf positions are skipped (the consumer would raise on them).
Markers are never deleted by the projector, so nothing is expired here; `clear()` serves a new map.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Tuple

from .room_labeler import Observation
from .types import Logger, null_logger

TEXT_NAMESPACE = 'detections_text'


def parse_marker(marker) -> Optional[Tuple[int, Observation]]:
    """`(marker id, Observation)` or None when the marker is not a usable text detection."""
    if getattr(marker, 'ns', None) != TEXT_NAMESPACE or not getattr(marker, 'text', ''):
        return None
    cls = marker.text.split('(')[0].strip().lower()
    if not cls:
        return None
    x, y = float(marker.pose.position.x), float(marker.pose.position.y)
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return int(marker.id), Observation(cls, x, y)


class ObservationBuffer:
    """Current set of detections keyed by marker id."""

    def __init__(self, log: Logger = null_logger) -> None:
        self._by_id: Dict[int, Observation] = {}
        self._log = log

    def update(self, markers: Iterable) -> bool:
        """Merge one MarkerArray (`.markers` list or any iterable of markers). Returns True if the set changed."""
        changed = False
        skipped = 0
        for marker in getattr(markers, 'markers', markers):
            parsed = parse_marker(marker)
            if parsed is None:
                skipped += 1
                continue
            key, obs = parsed
            if self._by_id.get(key) != obs:
                self._by_id[key] = obs
                changed = True
        if changed:
            self._log('marker_observations: %d observations (%d markers skipped)' % (len(self._by_id), skipped))
        return changed

    def observations(self) -> List[Observation]:
        """Deterministic order (by marker id)."""
        return [self._by_id[k] for k in sorted(self._by_id)]

    def clear(self) -> None:
        self._by_id.clear()

    def __len__(self) -> int:
        return len(self._by_id)
