"""Small cache keyed by message stamp (spec 009a-R12 support). Pure Python, no ROS, no clock (D-18).

The node stores each depth image under its header stamp; a detection is matched to the depth taken at the same
photo instant. Age is measured on the stamps themselves (newest stored stamp), so the cache needs no clock.
"""
from typing import Any, Optional


class StampCache:
    def __init__(self, max_size: int = 8, max_age_s: float = 2.0):
        if max_size < 1 or max_age_s <= 0.0:
            raise ValueError('max_size must be >= 1 and max_age_s > 0')
        self.max_size = max_size
        self.max_age_s = max_age_s
        self._items = []        # (stamp, value), oldest first

    def __len__(self) -> int:
        return len(self._items)

    def put(self, stamp: float, value: Any) -> None:
        self._items = [it for it in self._items if it[0] != stamp]
        self._items.append((stamp, value))
        self._items.sort(key=lambda it: it[0])
        newest = self._items[-1][0]
        self._items = [it for it in self._items if newest - it[0] <= self.max_age_s]
        del self._items[:max(0, len(self._items) - self.max_size)]

    def get(self, stamp: float, tolerance_s: float = 0.0) -> Optional[Any]:
        """Value whose stamp is closest to `stamp` within `tolerance_s`, or None."""
        best = None
        for s, v in self._items:
            d = abs(s - stamp)
            if d <= tolerance_s and (best is None or d < best[0]):
                best = (d, v)
        return None if best is None else best[1]
