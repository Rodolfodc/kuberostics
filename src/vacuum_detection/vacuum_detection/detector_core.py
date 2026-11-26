"""Detector logic without ROS or YOLO (spec 009a-R02, D-18). The model is injected behind `DetectorModel`.

A toolbox class: it runs the model on one image, filters by score, bounds the count, times the inference and keeps
counters. It never publishes and never reads a topic; what to do with the result (and the header it goes out
with) is the caller's business. An empty list is a normal, healthy result (R02: "I looked and saw nothing");
`None` means the inference failed, and the caller must NOT publish an empty array for it (health is the age of the
last response, an empty array would hide a broken model).
"""
import logging
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Protocol


@dataclass(frozen=True)
class Detection:
    """One box in pixels of the input image: centre (cx, cy), size (w, h), class label, score in 0..1."""
    label: str
    score: float
    cx: float
    cy: float
    w: float
    h: float


class DetectorModel(Protocol):
    """What the core needs from an inference engine. `image` is an RGB uint8 array (H, W, 3)."""

    def predict(self, image: Any) -> List[Detection]: ...


class DetectorCore:
    def __init__(self, model: DetectorModel, clock: Callable[[], float], score_threshold: float = 0.25,
                 max_detections: int = 100, logger: Optional[Any] = None):
        if not 0.0 <= score_threshold <= 1.0:
            raise ValueError(f'score_threshold must be in 0..1, got {score_threshold}')
        if max_detections < 1:
            raise ValueError(f'max_detections must be >= 1, got {max_detections}')
        self._model = model
        self._clock = clock                       # monotonic seconds, only to time the inference
        self._threshold = score_threshold
        self._max = max_detections
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self._m = {'processed': 0, 'empty': 0, 'errors': 0, 'detections': 0,
                   'last_inference_ms': 0.0, 'max_inference_ms': 0.0, 'total_inference_ms': 0.0}

    def process(self, image: Any) -> Optional[List[Detection]]:
        """Detections above the threshold, best first, at most `max_detections`; None if the model raised."""
        t0 = self._clock()
        try:
            found = self._model.predict(image)
        except Exception as exc:                  # any engine failure: count it and let the caller skip the frame
            self._m['errors'] += 1
            self._log.error(f'inference failed: {exc!r}')
            return None
        ms = (self._clock() - t0) * 1000.0
        kept = sorted((d for d in found if d.score >= self._threshold), key=lambda d: -d.score)[:self._max]
        self._m['processed'] += 1
        self._m['detections'] += len(kept)
        self._m['last_inference_ms'] = ms
        self._m['max_inference_ms'] = max(self._m['max_inference_ms'], ms)
        self._m['total_inference_ms'] += ms
        if not kept:
            self._m['empty'] += 1
        return kept

    def snapshot(self) -> Dict[str, Any]:
        snap = dict(self._m)
        total = snap.pop('total_inference_ms')
        snap['mean_inference_ms'] = total / self._m['processed'] if self._m['processed'] else 0.0
        return snap


class NullModel:
    """Detects nothing. For bring-up and tests of the graph without weights; never a real detector."""

    def predict(self, image: Any) -> List[Detection]:
        return []
