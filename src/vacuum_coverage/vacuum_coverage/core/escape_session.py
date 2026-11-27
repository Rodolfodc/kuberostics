"""EscapeSession: one Escape goal around `EscapeScript` (spec 005-R12, R14). No ROS imports (D-18).

Toolbox class (D-19): it owns no clock, reads no topic and publishes nothing. The caller passes `t`, the pose
`(x, y, yaw)` (None when the odometry is missing or stale), the front and rear clearances, and publishes the
returned command. It adds to the script what the goal needs:

- the goal timeout (`goal.timeout_s`, or `default_timeout_s` when it is not positive), counted from `start`;
- a failure without moving when there is no odometry (a robot is never driven blind) or when it is lost;
- the `outcome` parameter: `auto` runs the script, `succeed` / `fail` / `hang` keep the stub behaviours of spec 001
  (reverse, turn, then finish; `hang` holds a zero command until the timeout).
"""
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Union

from vacuum_coverage.core.escape_script import Command, EscapeConfig, EscapeScript, Pose, Result

OUTCOMES = ('auto', 'succeed', 'fail', 'hang')


def _noop(*_args) -> None:
    return None


@dataclass(frozen=True)
class Final:
    success: bool
    reason: str
    phase: str = 'session'


@dataclass(frozen=True)
class OutcomeStub:
    """Parameters of the stub behaviours (`succeed`, `fail`, `hang`)."""
    reverse_s: float = 1.0
    turn_s: float = 1.0
    reverse_linear_x: float = -0.1
    turn_angular_z: float = 0.8


class EscapeSession:
    strategy = 'scripted'

    def __init__(self, config: EscapeConfig, outcome: str = 'auto', stub: Optional[OutcomeStub] = None,
                 logger: Callable[[str, str], None] = _noop, metrics: Callable[[str, float], None] = _noop):
        if outcome not in OUTCOMES:
            raise ValueError(f'outcome must be one of {OUTCOMES}, got {outcome!r}')
        self.config = config
        self.outcome = outcome
        self._stub = stub or OutcomeStub()
        self._log = logger
        self._script = EscapeScript(config, logger, metrics)      # validates the configuration (005-R11 c)
        self._final: Optional[Final] = None
        self._t0: Optional[float] = None
        self.timeout_s = config.default_timeout_s

    def start(self, t: float, event_ranges: Sequence[float], pose: Optional[Pose], angle_min: float,
              angle_increment: float, timeout_s: float = 0.0) -> None:
        self._final = None
        self._t0 = t
        self.timeout_s = timeout_s if timeout_s > 0.0 else self.config.default_timeout_s
        self._log('INFO', f'session start: outcome={self.outcome}, timeout {self.timeout_s:.1f} s')
        if self.outcome == 'auto' and pose is not None:
            self._script.start(event_ranges, pose, angle_min, angle_increment)

    def step(self, t: float, pose: Optional[Pose], front_range: Optional[float],
             rear_range: Optional[float]) -> Union[Command, Final]:
        if self._t0 is None:
            raise RuntimeError('step() before start()')
        if self._final is not None:
            return self._final
        if t - self._t0 >= self.timeout_s:
            return self._end(False, 'timeout')
        if self.outcome != 'auto':
            return self._stub_step(t - self._t0)
        if pose is None:
            return self._end(False, 'odometry lost' if self._script.state != 'IDLE' else 'no odometry')
        out = self._script.step(t, pose, front_range, rear_range)
        if isinstance(out, Result):
            return self._end(out.success, out.reason, out.phase)
        return out

    def snapshot(self) -> dict:
        snap = dict(self._script.snapshot())
        snap['outcome'] = self.outcome
        return snap

    # ----------------------------------------------------------------- private

    def _stub_step(self, elapsed: float) -> Union[Command, Final]:
        s = self._stub
        if elapsed < s.reverse_s:
            return Command(s.reverse_linear_x, 0.0)
        if elapsed < s.reverse_s + s.turn_s:
            return Command(0.0, s.turn_angular_z)
        if self.outcome == 'hang':
            return Command(0.0, 0.0)
        return self._end(self.outcome == 'succeed', f'outcome {self.outcome}')

    def _end(self, success: bool, reason: str, phase: str = 'session') -> Final:
        self._final = Final(success, reason, phase)
        self._log('INFO' if success else 'WARN', f'session end: {"success" if success else "failure"}: {reason}')
        return self._final
