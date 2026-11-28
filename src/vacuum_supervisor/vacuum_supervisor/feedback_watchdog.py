"""Feedback watchdog of the supervisor (005-R34). Pure Python, no ROS, no clock of its own.

Toolbox classes (D-19): they only compute. The caller passes `now` (seconds on any monotonic clock), decides
when to arm, what to cancel and what to do with the verdict.

`FeedbackWatchdog` says when an action server must be treated as lost: no feedback for `stale_s` while a goal
is active, or the action server is not ready (staleness of feedback, not DDS graph discovery, which does not
notice a killed process within seconds). `CancelWindow` bounds how long the caller keeps trying to deliver the
cancel of the Nav2 goals (`nav_cancel_s`).
"""
import logging
from typing import Any, Dict, Optional

FEEDBACK_STALE = 'feedback_stale'
SERVER_NOT_READY = 'server_not_ready'


class FeedbackWatchdog:
    def __init__(self, stale_s: float = 2.5, logger: Optional[Any] = None):
        self.stale_s = float(stale_s)
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self._armed_at: Optional[float] = None
        self._last_feedback: Optional[float] = None
        self._metrics = {'arms': 0, 'feedback': 0, 'losses': 0}

    @property
    def armed(self) -> bool:
        return self._armed_at is not None

    def arm(self, now: float) -> None:
        """A goal became active. The acceptance counts as the first sign of life."""
        self._armed_at = now
        self._last_feedback = now
        self._metrics['arms'] += 1
        self._log.info(f'feedback watchdog armed (stale after {self.stale_s:.1f} s)')

    def feed(self, now: float) -> None:
        if self.armed:
            self._last_feedback = now
            self._metrics['feedback'] += 1

    def disarm(self) -> None:
        if self.armed:
            self._log.info('feedback watchdog disarmed')
        self._armed_at = self._last_feedback = None

    def check(self, now: float, server_ready: bool) -> Optional[str]:
        """Returns the loss reason once and disarms; None while the server looks alive or no goal is active."""
        if not self.armed:
            return None
        reason = None
        if not server_ready:
            reason = SERVER_NOT_READY
        elif now - self._last_feedback >= self.stale_s:
            reason = FEEDBACK_STALE
        if reason is None:
            return None
        silence = now - self._last_feedback
        self._metrics['losses'] += 1
        self._log.error(f'action server lost: {reason} (no feedback for {silence:.2f} s)')
        self.disarm()
        return reason

    def snapshot(self) -> Dict[str, Any]:
        return {'watchdog_armed': self.armed, **{f'watchdog_{k}': v for k, v in self._metrics.items()}}


class CancelWindow:
    """Retries an operation (the cancel request) until it succeeds or `window_s` has elapsed."""

    def __init__(self, window_s: float = 1.0, logger: Optional[Any] = None):
        self.window_s = float(window_s)
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self._deadline: Optional[float] = None

    @property
    def pending(self) -> bool:
        return self._deadline is not None

    def start(self, now: float) -> None:
        self._deadline = now + self.window_s

    def done(self) -> None:
        self._deadline = None

    def holds(self, now: float) -> bool:
        """True while a new goal must wait: the cancel is being delivered and the window is not over."""
        return self._deadline is not None and now <= self._deadline

    def should_try(self, now: float) -> bool:
        """True while the caller must still try. When the window is over, logs once and returns False."""
        if self._deadline is None:
            return False
        if now > self._deadline:
            self._log.error(f'cancel not delivered within {self.window_s:.1f} s: giving up')
            self._deadline = None
            return False
        return True
