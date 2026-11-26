"""Policy filters of the projector (spec 009a-R13, D-16). Pure Python, no ROS (D-18).

Moved here from the 2025S2 producer (confidence, allowed labels, mapped area). Transport health is NOT policy:
it stays in `detection_selector`. To add a filter, add a check method returning a reason string and call it
from `ProjectionPipeline`; each reason becomes a `dropped_<reason>` counter.
"""
import math
from typing import Callable, Iterable, Optional


def normalize_label(label: str) -> str:
    """The form the semantic node derives from the marker text (lowercase, stripped)."""
    return label.strip().lower()


class DetectionFilter:
    def __init__(self, min_confidence: float = 0.7, allowed_labels: Iterable[str] = (),
                 is_mapped: Optional[Callable[[float, float], bool]] = None):
        """`allowed_labels` empty means every label. `is_mapped(x, y)` None disables the area check."""
        self.min_confidence = min_confidence
        self.allowed = {normalize_label(s) for s in allowed_labels}
        self.is_mapped = is_mapped

    def check_detection(self, label: str, confidence: float) -> Optional[str]:
        """None if accepted, else the reason: 'label', 'confidence'."""
        lab = normalize_label(label)
        # '(' would corrupt "<label> (<id>)", which the consumer splits at the first '('
        if not lab or '(' in lab or (self.allowed and lab not in self.allowed):
            return 'label'
        if not (confidence >= self.min_confidence):          # also rejects NaN
            return 'confidence'
        return None

    def check_position(self, x: float, y: float) -> Optional[str]:
        """None if accepted, else 'nan' (never publish it, the consumer raises) or 'area'."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return 'nan'
        if self.is_mapped is not None and not self.is_mapped(x, y):
            return 'area'
        return None


class GridArea:
    """Mapped-area test over a grid (an OccupancyGrid copied into plain values by the node).

    Same indexing as the consumer (`int((p - origin) / resolution)`, cell value > 0 is a mapped room), so a
    detection accepted here is one the consumer can place in a room.
    """

    def __init__(self, origin_x: float, origin_y: float, resolution: float, width: int, height: int, data):
        if resolution <= 0.0:
            raise ValueError('resolution must be > 0')
        self._o = (origin_x, origin_y)
        self._res, self._w, self._h, self._data = resolution, width, height, data

    def __call__(self, x: float, y: float) -> bool:
        col = int((x - self._o[0]) / self._res)
        row = int((y - self._o[1]) / self._res)
        if not (0 <= col < self._w and 0 <= row < self._h):
            return False
        return self._data[row * self._w + col] > 0
