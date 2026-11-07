"""Submode state machine of the supervisor (D-01). Pure Python, no ROS (spec 001, plan).

Inputs: orchestrator mode, StuckEvent, escape action result.
Outputs: list of `Effect` that the ROS node executes (cancel actions, send goals, lock, notification).

Logs and telemetry (D-20): the caller injects a `logger` (anything with debug/info/warning/error taking one
string, such as `logging.Logger` or an rclpy logger) and reads `snapshot()` for periodic metrics.
"""
import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

NONE = 'NONE'
COVERING = 'COVERING'
ESCAPING = 'ESCAPING'
ESCAPE_FAILED = 'ESCAPE_FAILED'

ACTIVE_MODES = ('CLEANING', 'AUTODOCKING')


@dataclass(frozen=True)
class Effect:
    kind: str            # cancel_actions | send_cover_room | send_escape | lock_stop | notify
    value: Any = None    # lock_stop: bool; notify: (severity, code)


class SubmodeMachine:
    def __init__(self, logger: Optional[Any] = None):
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self.submode = NONE
        self.mode = ''
        self.reason = 'initial'
        self.resume_submode = NONE   # where to return after the escape
        self.locked = False          # stop lock; independent of the submode, released only by reset()
        self._metrics = {
            'transitions': 0, 'stuck_events': 0, 'escapes_succeeded': 0,
            'escapes_failed': 0, 'escape_timeouts': 0, 'resets': 0,
        }

    def snapshot(self) -> Dict[str, Any]:
        """Current state plus counters, for the periodic telemetry."""
        return {'mode': self.mode, 'submode': self.submode, 'reason': self.reason,
                'locked': self.locked, **self._metrics}

    def on_mode(self, mode: str) -> List[Effect]:
        self.mode = mode
        if self.submode == ESCAPING:
            # 001-R21: the escape keeps running, but what comes after it depends on the new mode.
            self.resume_submode = COVERING if mode == 'CLEANING' else NONE
        if mode not in ACTIVE_MODES:
            if self.submode == NONE:
                return []
            # 001-R05: outside CLEANING/AUTODOCKING the submode is NONE (cancels whatever is ongoing).
            # 001-R22: the lock is not touched here; only the user's reset releases it.
            self._go(NONE, f'mode {mode or "<empty>"}')
            self.resume_submode = NONE
            return [Effect('cancel_actions')]
        if mode == 'AUTODOCKING' and self.submode == COVERING:
            self._go(NONE, 'mode AUTODOCKING')                # 001-R21: docking must not compete with coverage
            self.resume_submode = NONE
            return [Effect('cancel_actions')]
        if mode == 'CLEANING' and self.submode == NONE:
            if self.locked:                                   # 001-R22
                self._log.debug('CLEANING ignored: stop lock is on, waiting for reset')
                return []
            self._go(COVERING, 'mode CLEANING')               # 001-R06
            return [Effect('send_cover_room')]
        return []

    def on_stuck(self) -> List[Effect]:
        if self.submode == COVERING:                          # 001-R07
            self._metrics['stuck_events'] += 1
            self.resume_submode = COVERING
            self._go(ESCAPING, 'stuck while covering')
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
