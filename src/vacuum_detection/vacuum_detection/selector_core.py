"""Circuit breaker and filters of the detection selector (spec 009a, plan). Pure Python, no ROS (D-18).

The clock, the notification callback and the logger are injected (D-20). Time is in seconds on the robot
clock; a response is described only by its source ('cloud' | 'local') and its photo `stamp` (seconds).

Order applied to each response: (1) health tracking and breaker transitions (cloud responses only),
(2) source gate (009a-R07), (3) age filter (009a-R05), (4) watermark filter (009a-R06).
"""
import logging
from typing import Any, Callable, Dict, Optional

CLOSED = 'CLOSED'
OPEN = 'OPEN'
HALF_OPEN = 'HALF_OPEN'
CLOUD = 'cloud'
LOCAL = 'local'


class SelectorCore:
    def __init__(self, clock: Callable[[], float],
                 notify: Optional[Callable[[str, str], None]] = None,
                 logger: Optional[Any] = None,
                 max_age_s: float = 0.67, open_threshold_s: float = 1.5,
                 close_threshold_s: float = 0.5, stable_window_s: float = 3.0):
        self._clock = clock
        self._notify = notify if notify is not None else (lambda severity, code: None)
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self.max_age_s = max_age_s
        self.open_threshold_s = open_threshold_s
        self.close_threshold_s = close_threshold_s
        self.stable_window_s = stable_window_s
        self.state = CLOSED
        self.watermark = float('-inf')
        self._last_good_cloud = clock()   # CLOSED: the grace period counts from startup
        self._last_cloud = clock()        # HALF_OPEN: arrival of the previous cloud response
        self._stable_since = 0.0
        self._m = {'forwarded': 0, 'dropped_stale': 0, 'dropped_watermark': 0,
                   'dropped_inactive': 0, 'transitions': 0}

    def snapshot(self) -> Dict[str, Any]:
        """State plus counters, for OffloadStatus and the periodic telemetry."""
        return {'state': self.state, 'watermark': self.watermark, **self._m}

    def _go(self, state: str, why: str) -> None:
        self._log.info(f'selector {self.state} -> {state}: {why}')
        self.state = state
        self._m['transitions'] += 1

    def tick(self) -> None:
        """Call periodically: detects the silence of the cloud (009a-R08) without needing a response."""
        now = self._clock()
        if self.state == CLOSED and now - self._last_good_cloud > self.open_threshold_s:
            self._go(OPEN, f'no healthy cloud response for {now - self._last_good_cloud:.2f} s')
            self._notify('WARN', 'CLOUD_DETECTOR_DOWN')
        elif self.state == HALF_OPEN and now - self._last_cloud > self.open_threshold_s:
            self._go(OPEN, 'cloud silent during the stability window')

    def on_response(self, source: str, stamp: float) -> bool:
        """Returns True if the response must be forwarded to /myRobot/perception/detections."""
        now = self._clock()
        age = now - stamp
        if source == CLOUD:
            self._track_cloud(now, age)
        else:
            self.tick()
        wanted = CLOUD if self.state == CLOSED else LOCAL
        if source != wanted:
            self._m['dropped_inactive'] += 1                        # 009a-R07
            return False
        if age > self.max_age_s:
            self._m['dropped_stale'] += 1                           # 009a-R05
            return False
        if not stamp > self.watermark:
            self._m['dropped_watermark'] += 1                       # 009a-R06
            return False
        self.watermark = stamp
        self._m['forwarded'] += 1
        return True

    def _track_cloud(self, now: float, age: float) -> None:
        self.tick()                                                 # silence before this response counts
        if self.state == CLOSED:
            if age <= self.open_threshold_s:
                self._last_good_cloud = now
        elif self.state == OPEN:
            if age <= self.close_threshold_s:                       # 009a-R09
                self._go(HALF_OPEN, f'cloud response with age {age:.2f} s')
                self._stable_since = now
        elif self.state == HALF_OPEN:
            if age > self.close_threshold_s:                        # 009a-R10 (violation)
                self._go(OPEN, f'cloud response with age {age:.2f} s')
            elif now - self._stable_since >= self.stable_window_s:  # 009a-R10
                self._go(CLOSED, f'stable for {now - self._stable_since:.2f} s')
                self._last_good_cloud = now
                self._notify('INFO', 'CLOUD_DETECTOR_RECOVERED')
        self._last_cloud = now
