"""Submode state machine of the supervisor (D-01). Pure Python, no ROS (spec 001, plan).

Inputs: orchestrator mode, StuckEvent, escape action result, cover_room result, dock timer expiry.
Outputs: list of `Effect` that the ROS node executes (cancel actions, send goals, lock, notification, docking
request, dock timer).

Logs and telemetry (D-20): the caller injects a `logger` (anything with debug/info/warning/error taking one
string, such as `logging.Logger` or an rclpy logger) and reads `snapshot()` for periodic metrics.
"""
import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

NONE = 'NONE'
EXPLORING = 'EXPLORING'
COVERING = 'COVERING'
ESCAPING = 'ESCAPING'
ESCAPE_FAILED = 'ESCAPE_FAILED'

ACTIVE_MODES = ('CLEANING', 'AUTODOCKING')


@dataclass(frozen=True)
class Effect:
    kind: str            # cancel_actions | send_cover_room | send_explore | send_escape | lock_stop | notify
                         # | request_dock | start_dock_timer | cancel_dock_timer
    value: Any = None    # lock_stop: bool; notify: (severity, code)


class SubmodeMachine:
    def __init__(self, logger: Optional[Any] = None, dock_after_coverage: bool = False,
                 explore_on_start: bool = False):
        # explore_on_start enables the EXPLORING rows of spec 003-R14. Default false: spec 001 holds unchanged.
        self.explore_on_start = explore_on_start
        self.explored = False           # latched /myRobot/map/explored; only recorded
        self.exploration_done = False   # set when an Explore goal ends; stops a loop on failure (003-R14)
        # dock_after_coverage enables the rows of spec 005-R30 (cover_room result -> docking request). While
        # amendment A2 of spec 001 is not applied it stays false and the 001 behaviour holds unchanged.
        self.dock_after_coverage = dock_after_coverage
        self.coverage_blocked = False   # true after the cover_room result until the mode leaves CLEANING
        self._dock_timer_active = False
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self.submode = NONE
        self.mode = ''
        self.reason = 'initial'
        self.resume_submode = NONE   # where to return after the escape
        self.locked = False          # stop lock; independent of the submode, released only by reset()
        self._metrics = {
            'transitions': 0, 'stuck_events': 0, 'escapes_succeeded': 0,
            'escapes_failed': 0, 'escape_timeouts': 0, 'resets': 0,
            'coverage_results': 0, 'explore_results': 0, 'dock_requests': 0, 'dock_request_failures': 0,
        }

    def snapshot(self) -> Dict[str, Any]:
        """Current state plus counters, for the periodic telemetry."""
        return {'mode': self.mode, 'submode': self.submode, 'reason': self.reason,
                'locked': self.locked, 'coverage_blocked': self.coverage_blocked,
                'explored': self.explored, 'exploration_done': self.exploration_done, **self._metrics}

    def on_mode(self, mode: str) -> List[Effect]:
        self.mode = mode
        pre = self._leave_cleaning() if mode != 'CLEANING' else []     # 005-R30: clears coverage_blocked
        return pre + self._on_mode(mode)

    def _on_mode(self, mode: str) -> List[Effect]:
        if self.submode == ESCAPING:
            # 001-R21: the escape keeps running, but what comes after it depends on the new mode.
            if mode == 'CLEANING' and self.resume_submode == EXPLORING:
                pass          # the mode is republished every 0.2 s: it is not a change (003-R14)
            else:
                self.resume_submode = COVERING if mode == 'CLEANING' else NONE
        if mode not in ACTIVE_MODES:
            if self.submode == NONE:
                return []
            # 001-R05: outside CLEANING/AUTODOCKING the submode is NONE (cancels whatever is ongoing).
            # 001-R22: the lock is not touched here; only the user's reset releases it.
            self._go(NONE, f'mode {mode or "<empty>"}')
            self.resume_submode = NONE
            return [Effect('cancel_actions')]
        if mode == 'AUTODOCKING' and self.submode in (COVERING, EXPLORING):
            self._go(NONE, 'mode AUTODOCKING')                # 001-R21: docking must not compete with coverage
            self.resume_submode = NONE
            return [Effect('cancel_actions')]
        if mode == 'CLEANING' and self.submode == NONE:
            if self.locked:                                   # 001-R22
                self._log.debug('CLEANING ignored: stop lock is on, waiting for reset')
                return []
            if self.coverage_blocked:                         # 005-R30: coverage already ended, docking asked
                self._log.debug('CLEANING ignored: coverage ended, waiting for the docking request')
                return []
            if self.explore_on_start and not self.explored and not self.exploration_done:
                self._go(EXPLORING, 'mode CLEANING, map not explored')     # 003-R14
                return [Effect('send_explore')]
            self._go(COVERING, 'mode CLEANING')               # 001-R06
            return [Effect('send_cover_room')]
        return []

    def on_explored(self, explored: bool) -> List[Effect]:
        """Latched `/myRobot/map/explored`. Recorded only: the Explore result decides the transition."""
        if explored != self.explored:
            self._log.info(f'explored = {explored}')
        self.explored = explored
        return []

    def on_explore_result(self, success: bool) -> List[Effect]:
        """The `explore` goal ended on its own, or was declared lost by the watchdog (003-R14)."""
        if self.submode != EXPLORING or self.mode != 'CLEANING':   # late result after a cancellation
            self._log.debug(f'explore result ignored (submode {self.submode}, mode {self.mode or "<empty>"})')
            return []
        self._metrics['explore_results'] += 1
        self.exploration_done = True
        self._go(COVERING, 'explore succeeded' if success else 'explore failed, covering with the partial map')
        return [Effect('send_cover_room')]

    def on_cover_result(self, success: bool) -> List[Effect]:
        """The `cover_room` goal ended on its own, successfully or not (005-R30)."""
        if not self.dock_after_coverage:
            self._log.debug('cover_room result ignored (dock_after_coverage is off)')
            return []
        if self.submode != COVERING or self.mode != 'CLEANING':   # late result after a cancellation
            self._log.debug(f'cover_room result ignored (submode {self.submode}, mode {self.mode or "<empty>"})')
            return []
        self._metrics['coverage_results'] += 1
        self._metrics['dock_requests'] += 1
        self.coverage_blocked = True
        self._dock_timer_active = True
        self._go(NONE, 'cover_room succeeded' if success else 'cover_room failed')
        self.resume_submode = NONE
        notify = (Effect('notify', ('INFO', 'COVERAGE_DONE')) if success
                  else Effect('notify', ('WARN', 'COVERAGE_FAILED')))
        return [notify, Effect('request_dock'), Effect('start_dock_timer')]

    def on_dock_timeout(self) -> List[Effect]:
        """`dock_request_timeout_s` elapsed and the mode is still CLEANING (005-R30, Art. VI.3)."""
        if not self._dock_timer_active or self.mode != 'CLEANING' or not self.coverage_blocked:
            self._log.debug('dock timeout ignored')
            return []
        self._dock_timer_active = False
        self._metrics['dock_request_failures'] += 1
        self._log.error('docking was not reached within dock_request_timeout_s: no new cover_room goal')
        return [Effect('notify', ('ERROR', 'DOCK_REQUEST_FAILED'))]

    def _leave_cleaning(self) -> List[Effect]:
        if not self.coverage_blocked and not self._dock_timer_active:
            return []
        self.coverage_blocked = False
        self._log.info(f'coverage_blocked cleared (mode {self.mode or "<empty>"})')
        if self._dock_timer_active:
            self._dock_timer_active = False
            return [Effect('cancel_dock_timer')]
        return []

    def on_stuck(self) -> List[Effect]:
        if self.locked:                                       # 001-R22: the lock blocks every escape anyway
            self._log.debug('stuck event ignored: stop lock is on')
            return []
        if self.submode in (COVERING, EXPLORING):             # 001-R07, 003-R14
            self._metrics['stuck_events'] += 1
            self.resume_submode = self.submode
            self._go(ESCAPING, f'stuck while {"exploring" if self.submode == EXPLORING else "covering"}')
            return [Effect('cancel_actions'), Effect('send_escape')]
        if self.submode == NONE and self.mode == 'AUTODOCKING':
            self._metrics['stuck_events'] += 1
            self.resume_submode = NONE
            self._go(ESCAPING, 'stuck while autodocking')
            return [Effect('send_escape')]
        self._log.debug(f'stuck event ignored (submode {self.submode}, mode {self.mode or "<empty>"})')
        return []

    def on_escape_result(self, success: bool) -> List[Effect]:
        if self.submode != ESCAPING:                          # late result after a cancellation
            self._log.debug(f'escape result ignored (submode {self.submode})')
            return []
        if success:                                           # 001-R08
            self._metrics['escapes_succeeded'] += 1
            self._go(self.resume_submode, 'escape succeeded')
            if self.submode == COVERING:
                return [Effect('send_cover_room')]
            if self.submode == EXPLORING:
                return [Effect('send_explore')]
            return []
        return self._fail('escape failed')

    def on_escape_timeout(self) -> List[Effect]:
        """The supervisor exceeded `escape_timeout_s` (001-R09)."""
        if self.submode != ESCAPING:
            self._log.debug(f'escape timeout ignored (submode {self.submode})')
            return []
        self._metrics['escape_timeouts'] += 1
        return [Effect('cancel_actions')] + self._fail('escape timed out')

    def reset(self) -> List[Effect]:
        """User's reset service: releases the stop lock, whatever the current submode (001-R22)."""
        if not self.locked:
            self._log.debug(f'reset ignored (not locked, submode {self.submode})')
            return []
        self._metrics['resets'] += 1
        self.locked = False
        if self.submode == ESCAPE_FAILED:
            self._go(NONE, 'user reset')
            self.resume_submode = NONE
        else:
            self.reason = 'user reset'
            self._log.info(f'stop lock released by user reset (submode {self.submode})')
        return [Effect('lock_stop', False)]

    def _fail(self, reason: str) -> List[Effect]:
        self._metrics['escapes_failed'] += 1
        self.locked = True
        self._go(ESCAPE_FAILED, reason)
        return [Effect('lock_stop', True), Effect('notify', ('ERROR', 'ESCAPE_FAILED'))]

    def _go(self, new: str, reason: str) -> None:
        old = self.submode
        self.submode = new
        self.reason = reason
        self._metrics['transitions'] += 1
        msg = f'submode {old} -> {new} ({reason})'
        if new == ESCAPE_FAILED:
            self._log.error(msg)
        else:
            self._log.info(msg)
