"""Wiring of the projector decisions (spec 009a-R12, R13). Pure Python, no ROS (D-18).

filter (label, confidence) -> ProjectorCore (depth, TF at the photo stamp) -> filter (NaN, area) -> DetectionSet.
The node feeds it detections and reads `markers()`; it makes no decision itself.
"""
import logging
import math
from typing import Any, Dict, List, Optional

from vacuum_detection.detection_filter import DetectionFilter, normalize_label
from vacuum_detection.detection_set import DetectionSet
from vacuum_detection.marker_spec import MarkerSpec, build_markers
from vacuum_detection.projection_core import ProjectorCore


class ProjectionPipeline:
    def __init__(self, core: ProjectorCore, policy: DetectionFilter, detections: DetectionSet,
                 logger: Optional[Any] = None):
        self.core, self.policy, self.set = core, policy, detections
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self._m = {'accepted': 0, 'merged': 0, 'dropped_label': 0, 'dropped_confidence': 0,
                   'dropped_nan': 0, 'dropped_area': 0}

    def process(self, stamp: float, label: str, confidence: float, u: float, v: float, depth: float) -> str:
        """'new', 'merged' or the drop reason ('label', 'confidence', 'depth', 'tf', 'nan', 'area').
        `depth` NaN means no depth was found for the photo (counted as dropped_depth by the core)."""
        reason = self.policy.check_detection(label, confidence)
        p = None
        if reason is None:
            p = self.core.project(stamp, u, v, depth)       # counts dropped_depth / dropped_tf itself
            if p is None:
                return 'depth' if not (math.isfinite(depth) and depth > 0.0) else 'tf'
            reason = self.policy.check_position(p[0], p[1])
            if reason is None and not math.isfinite(p[2]):
                reason = 'nan'
        if reason is not None:
            self._count_drop(reason)
            self._log.debug(f'detection {label!r} dropped: {reason}')
            return reason
        det, is_new = self.set.add(normalize_label(label), p[0], p[1], p[2], stamp, confidence)
        self._m['accepted' if is_new else 'merged'] += 1
        if is_new:
            self._log.info(f'new detection {det.label} ({det.det_id}) at ({det.x:.2f}, {det.y:.2f})')
        return 'new' if is_new else 'merged'

    def _count_drop(self, reason: str) -> None:
        self._m['dropped_' + reason] += 1

    def markers(self) -> List[MarkerSpec]:
        """The WHOLE persistent set (the consumer keeps no history, markers-format.md section 7)."""
        return build_markers(self.set.items())

    def snapshot(self) -> Dict[str, Any]:
        return {**self.core.snapshot(), **self._m, 'detections': len(self.set)}
