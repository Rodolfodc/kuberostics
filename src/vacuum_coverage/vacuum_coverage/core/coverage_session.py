"""CoverageSession and ResumeStore: waypoint sequencing of one CoverRoom goal (005-R19 to R21, R32).

Pure Python, no ROS, no clock (D-18): the node passes `t` (seconds on its clock) so every timeout is testable
with a simulated clock. The node asks `step(t)` what to do, does it (send a NavigateToPose goal, cancel it,
finish the action) and reports `goal_result(t, reached)`. The session never talks to Nav2 itself.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from vacuum_mapping.core.types import Logger, null_logger

from .coverage_planner import Waypoint
from .coverage_tracker import CoverageTracker


class StepKind(enum.Enum):
    SEND = 'send'        # send `waypoint` as a NavigateToPose goal
    WAIT = 'wait'        # a goal is in flight
    CANCEL = 'cancel'    # the in-flight goal timed out: cancel it (the attempt is already counted)
    DONE = 'done'        # finish the action with `success`; cancel the Nav2 goal first when `cancel_goal`


@dataclass(frozen=True)
class Step:
    kind: StepKind
    waypoint: Optional[Waypoint] = None
    index: int = -1
    success: Optional[bool] = None
    reason: str = ''
    cancel_goal: bool = False
    degraded: bool = False        # success below target_ratio but at or above min_acceptable_ratio
    covered_ratio: float = 0.0


@dataclass(frozen=True)
class SessionLimits:
    """Defaults of spec 005 (R20, R21); the node passes its parameters."""
    waypoint_timeout_s: float = 120.0
    coverage_timeout_s: float = 1800.0
    max_skip_ratio: float = 0.3
    target_ratio: float = 0.95
    min_acceptable_ratio: float = 0.70   # waypoints exhausted below target: >= this is a degraded success
    retries: int = 1          # R20: one retry, then skip


class CoverageSession:
    def __init__(self, waypoints: Sequence[Waypoint], tracker: CoverageTracker, passes: int = 1,
                 limits: SessionLimits = SessionLimits(), log: Logger = null_logger) -> None:
        self._wps: List[Waypoint] = list(waypoints)
        self._tracker = tracker
        self._last_pass = max(1, passes) - 1
        self._lim = limits
        self._log = log
        self._index = 0
        self._attempts = 0
        self._skipped = 0
        self._in_flight = False
        self._sent_t = 0.0
        self._active_s = 0.0          # time spent running before the current stretch (R32)
        self._seg_start: Optional[float] = None
        self._done: Optional[Step] = None

    # ---- lifecycle of the goal -------------------------------------------------------------------------
    def start(self, t: float) -> None:
        self._seg_start = t
        self._log('coverage_session: start waypoints=%d coverable_cells=%d'
                  % (len(self._wps), self._tracker.coverable_count))

    def suspend(self, t: float) -> None:
        """The goal was cancelled (escape): keep position in the list and the covered mask (R32). An
        interrupted waypoint is not a failure and its attempt count is kept."""
        self._accumulate(t)
        if self._in_flight:
            self._in_flight = False
            self._attempts = max(0, self._attempts - 1)
        self._log('coverage_session: suspended at waypoint %d/%d ratio=%.3f'
                  % (self._index, len(self._wps), self._tracker.covered_ratio))

    def resume(self, t: float) -> None:
        self._seg_start = t
        self._log('coverage_session: resumed at waypoint %d/%d' % (self._index, len(self._wps)))

    # ---- decisions --------------------------------------------------------------------------------------
    def step(self, t: float) -> Step:
        if self._done is not None:
            return Step(StepKind.DONE, success=self._done.success, reason=self._done.reason,
                        degraded=self._done.degraded, covered_ratio=self._done.covered_ratio)
        if self._active(t) > self._lim.coverage_timeout_s:
            return self._finish(False, 'coverage_timeout')
        if (self._index < len(self._wps) and self._wps[self._index].pass_index >= self._last_pass
                and self._tracker.covered_ratio >= self._lim.target_ratio):
            return self._finish(True, 'target_ratio')
        if self._in_flight:
            if t - self._sent_t > self._lim.waypoint_timeout_s:
                wp_index = self._index
                self._log('coverage_session: waypoint %d timed out after %.0f s'
                          % (wp_index, t - self._sent_t))
                self._in_flight = False
                failed = self._attempt_failed()
                if failed is not None:
                    return Step(StepKind.DONE, success=False, reason=failed.reason, cancel_goal=True)
                return Step(StepKind.CANCEL, self._wps[wp_index], wp_index)
            return Step(StepKind.WAIT, self._wps[self._index], self._index)
        if self._index >= len(self._wps):
            ratio = self._tracker.covered_ratio
            if ratio >= self._lim.target_ratio:
                return self._finish(True, 'exhausted')
            if ratio >= self._lim.min_acceptable_ratio:
                return self._finish(True, 'exhausted_degraded', degraded=True)
            return self._finish(False, 'below_min_ratio')
        self._in_flight = True
        self._sent_t = t
        self._attempts += 1
        return Step(StepKind.SEND, self._wps[self._index], self._index)

    def goal_result(self, t: float, reached: bool) -> None:
        """Outcome of the in-flight Nav2 goal. Results with nothing in flight (a goal already cancelled by
        timeout, or a late reply) are ignored."""
        if not self._in_flight:
            self._log('coverage_session: result ignored, no goal in flight')
            return
        self._in_flight = False
        if reached:
            self._index += 1
            self._attempts = 0
        else:
            self._attempt_failed()

    # ---- state ------------------------------------------------------------------------------------------
    @property
    def tracker(self) -> CoverageTracker:
        return self._tracker

    @property
    def waypoints(self) -> List[Waypoint]:
        return list(self._wps)

    @property
    def finished(self) -> bool:
        return self._done is not None

    @property
    def degraded(self) -> bool:
        return bool(self._done and self._done.degraded)

    @property
    def success(self) -> Optional[bool]:
        return None if self._done is None else self._done.success

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def skipped_waypoints(self) -> int:
        return self._skipped

    @property
    def covered_ratio(self) -> float:
        return self._tracker.covered_ratio

    def snapshot(self) -> Dict[str, object]:
        """Counters for telemetry (D-20)."""
        return {'waypoints_total': len(self._wps), 'current_index': self._index,
                'skipped_waypoints': self._skipped, 'covered_ratio': round(self._tracker.covered_ratio, 3),
                'in_flight': self._in_flight, 'finished': self.finished}

    # ---- internals --------------------------------------------------------------------------------------
    def _attempt_failed(self) -> Optional[Step]:
        """Retry once, then skip. Returns the DONE step when the skip limit is exceeded."""
        if self._attempts <= self._lim.retries:
            self._log('coverage_session: waypoint %d failed, retrying (attempt %d)' % (self._index, self._attempts))
            return None
        self._skipped += 1
        self._log('coverage_session: waypoint %d skipped (%d skipped of %d)'
                  % (self._index, self._skipped, len(self._wps)))
        self._index += 1
        self._attempts = 0
        if self._skipped > self._lim.max_skip_ratio * len(self._wps):
            return self._finish(False, 'skip_ratio', cancel=False)
        return None

    def _finish(self, success: bool, reason: str, cancel: bool = True, degraded: bool = False) -> Step:
        step = Step(StepKind.DONE, success=success, reason=reason, cancel_goal=cancel and self._in_flight,
                    degraded=degraded, covered_ratio=self._tracker.covered_ratio)
        self._done = step
        self._in_flight = False if cancel else self._in_flight
        self._log('coverage_session: done success=%s degraded=%s reason=%s ratio=%.3f skipped=%d'
                  % (success, degraded, reason, self._tracker.covered_ratio, self._skipped))
        return step

    def _accumulate(self, t: float) -> None:
        if self._seg_start is not None:
            self._active_s += max(0.0, t - self._seg_start)
            self._seg_start = None

    def _active(self, t: float) -> float:
        return self._active_s + (0.0 if self._seg_start is None else max(0.0, t - self._seg_start))


class ResumeStore:
    """Keeps the suspended session of one room for `resume_window_s` (005-R32)."""

    def __init__(self, resume_window_s: float = 120.0, log: Logger = null_logger) -> None:
        self._window = resume_window_s
        self._log = log
        self._room: Optional[str] = None
        self._session: Optional[CoverageSession] = None
        self._saved_t = 0.0

    def save(self, room_id: str, session: CoverageSession, t: float) -> None:
        self._room, self._session, self._saved_t = room_id, session, t
        self._log('resume_store: saved room=%r at waypoint %d' % (room_id, session.current_index))

    def take(self, room_id: str, t: float) -> Optional[CoverageSession]:
        """The saved session when `room_id` matches and the window has not expired; the store is emptied
        either way, so a goal for another room discards the old state."""
        session, room, saved_t = self._session, self._room, self._saved_t
        self.clear()
        if session is None:
            return None
        if room != room_id:
            self._log('resume_store: dropped state of room=%r (goal for room=%r)' % (room, room_id))
            return None
        if t - saved_t > self._window:
            self._log('resume_store: state of room=%r expired' % room)
            return None
        return session

    def clear(self) -> None:
        self._room, self._session = None, None
