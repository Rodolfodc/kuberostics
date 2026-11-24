"""Rate limiter by photo stamp (spec 009a-R03, R11). Pure Python, no ROS (D-18).

A toolbox class: it only answers "should this frame pass?". Where the frame goes and how it is encoded is the
caller's business. The rate follows the frame stamp, not the arrival time, so network or executor jitter does not
change which photos are sent. The clock is injected only as a fallback for unstamped frames (stamp 0).
"""
import logging
from typing import Any, Callable, Dict, Optional

_EPS = 1e-9


class ThrottleCore:
    def __init__(self, clock: Callable[[], float], rate_hz: float, logger: Optional[Any] = None):
        if not rate_hz > 0.0:
            raise ValueError(f'rate_hz must be positive, got {rate_hz}')
        self._clock = clock
        self._period = 1.0 / rate_hz
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self._last: Optional[float] = None       # time base of the last frame that passed
        self._m = {'passed': 0, 'dropped_rate': 0, 'restarts': 0}

    def snapshot(self) -> Dict[str, Any]:
        return dict(self._m)

    def offer(self, stamp_s: float) -> bool:
        """True if the frame with this stamp must be sent."""
        t = stamp_s if stamp_s > 0.0 else self._clock()   # unstamped frame: use the arrival instant
        if self._last is not None and t < self._last:
            # Time went backwards (simulation reset): waiting for the old time base would freeze the stream.
            self._log.warning(f'stamp {t:.3f} is before the last sent {self._last:.3f}: throttle restarted')
            self._m['restarts'] += 1
            self._last = None
        if self._last is not None and t - self._last < self._period - _EPS:
            self._m['dropped_rate'] += 1
            return False
        self._last = t
        self._m['passed'] += 1
        return True
