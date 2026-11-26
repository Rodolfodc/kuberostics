"""StuckCore: sliding-window stuck detection (spec 005, 005-R01 to R05). No ROS imports (D-18).

Toolbox class (D-19): it only computes. The caller feeds the commanded velocity and the measured pose with a
timestamp it owns, decides the thresholds (constructor) and what to do with the returned cause (publish a
`StuckEvent`, log, count). It reads no topic, owns no clock and publishes nothing.

Measured motion is integrated incrementally (sum of the translation steps and of the wrapped yaw steps between
consecutive samples), so a yaw crossing +-pi is not mistaken for a full turn or for no turn. The window slides:
at time t it covers the samples from the latest one at or before `t - window_s` up to t, so it is never shorter
than `window_s` and the detection time after a stop is `window_s` plus at most one sample period.

D-20: `logger(level, message)` receives every state transition; `metrics(name, value)` the counters.
"""
import math
from collections import deque
from typing import Callable, Optional, Tuple

NO_PROGRESS = 'NO_PROGRESS'
BUMPER = 'BUMPER'

Pose = Tuple[float, float, float]    # x, y, yaw (rad)


def _noop(*_args) -> None:
    return None


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class StuckCore:
    def __init__(self, cmd_linear_min_m_s: float = 0.05, cmd_angular_min_rad_s: float = 0.20,
                 window_s: float = 3.0, min_translation_m: float = 0.03, min_rotation_rad: float = 0.10,
                 cooldown_s: float = 5.0, cmd_timeout_s: float = 0.5, max_sample_gap_s: float = 0.5,
                 logger: Callable[[str, str], None] = _noop,
                 metrics: Callable[[str, float], None] = _noop):
        if window_s <= 0.0:
            raise ValueError('window_s must be > 0')
        if cooldown_s < 0.0:
            raise ValueError('cooldown_s must be >= 0')
        if min_translation_m < 0.0 or min_rotation_rad < 0.0:
            raise ValueError('min_translation_m and min_rotation_rad must be >= 0')
        if cmd_timeout_s <= 0.0 or max_sample_gap_s <= 0.0:
            raise ValueError('cmd_timeout_s and max_sample_gap_s must be > 0')
        self.cmd_linear_min_m_s = cmd_linear_min_m_s
        self.cmd_angular_min_rad_s = cmd_angular_min_rad_s
        self.window_s = window_s
        self.min_translation_m = min_translation_m
        self.min_rotation_rad = min_rotation_rad
        self.cooldown_s = cooldown_s
        self.cmd_timeout_s = cmd_timeout_s
        self.max_sample_gap_s = max_sample_gap_s
        self._log = logger
        self._metrics = metrics
        self._samples = deque()          # (t, cumulative translation, cumulative |yaw|)
        self._cum_trans = 0.0
        self._cum_rot = 0.0
        self._last_pose: Optional[Pose] = None
        self._last_t: Optional[float] = None
        self._cooldown_until: Optional[float] = None
        self._events = 0
        self._bumper_events = 0
        self._resets = 0
        self._window_trans = 0.0
        self._window_rot = 0.0
        self._window_elapsed = 0.0

    # ------------------------------------------------------------------ public

    def update(self, t: float, cmd_linear: float, cmd_angular: float, pose: Optional[Pose],
               cmd_age_s: float = 0.0) -> Optional[str]:
        """One sample. `cmd_age_s` is how old the command is (twist_mux goes silent on an input timeout instead of
        sending zero, so an old command counts as zero). `pose=None` means no usable measurement. Returns
        `NO_PROGRESS` when the robot is stuck, else None."""
        if self._last_t is not None and t < self._last_t:
            self._reset('time went backwards')
        self._last_t = t
        if self._cooldown_until is not None:
            if t < self._cooldown_until:
                return None
            self._cooldown_until = None
            self._log('INFO', f'cooldown ended at t={t:.2f}; monitoring restarts with a fresh window')
        active = (cmd_age_s <= self.cmd_timeout_s and
                  (abs(cmd_linear) >= self.cmd_linear_min_m_s or abs(cmd_angular) >= self.cmd_angular_min_rad_s))
        if not active:
            self._reset('command below both thresholds' if cmd_age_s <= self.cmd_timeout_s else 'stale command')
            return None
        if pose is None:
            self._reset('no pose measurement')
            return None
        if self._samples and t - self._samples[-1][0] > self.max_sample_gap_s:
            self._reset(f'sample gap {t - self._samples[-1][0]:.2f} s')
        if not self._samples:
            self._samples.append((t, 0.0, 0.0))
            self._cum_trans = self._cum_rot = 0.0
            self._last_pose = pose
            self._window_trans = self._window_rot = self._window_elapsed = 0.0
            self._log('INFO', f'monitoring started at t={t:.2f}')
            return None
        self._cum_trans += math.hypot(pose[0] - self._last_pose[0], pose[1] - self._last_pose[1])
        self._cum_rot += abs(_wrap(pose[2] - self._last_pose[2]))
        self._last_pose = pose
        self._samples.append((t, self._cum_trans, self._cum_rot))
        while len(self._samples) >= 2 and self._samples[1][0] <= t - self.window_s:
            self._samples.popleft()
        t0, trans0, rot0 = self._samples[0]
        self._window_elapsed = t - t0
        self._window_trans = self._cum_trans - trans0
        self._window_rot = self._cum_rot - rot0
        if (self._window_elapsed >= self.window_s - 1e-9 and self._window_trans < self.min_translation_m
                and self._window_rot < self.min_rotation_rad):
            self._events += 1
            self._log('WARN', f'stuck: {self._window_elapsed:.2f} s of command with translation '
                              f'{self._window_trans:.3f} m and rotation {self._window_rot:.3f} rad '
                              f'(cmd {cmd_linear:.2f} m/s, {cmd_angular:.2f} rad/s)')
            self._metrics('stuck_events_total', self._events)
            self._start_cooldown(t)
            return NO_PROGRESS
        return None

    def bumper_pressed(self, t: float) -> Optional[str]:
        """A press of the optional bumper (005-R05). Same cooldown as the other cause."""
        self._last_t = t if self._last_t is None else max(self._last_t, t)
        if self._cooldown_until is not None and t < self._cooldown_until:
            self._log('INFO', f'bumper press at t={t:.2f} ignored during the cooldown')
            return None
        self._bumper_events += 1
        self._log('WARN', f'bumper pressed at t={t:.2f}')
        self._metrics('bumper_events_total', self._bumper_events)
        self._start_cooldown(t)
        return BUMPER

    def snapshot(self) -> dict:
        if self._cooldown_until is not None and (self._last_t or 0.0) < self._cooldown_until:
            state = 'cooldown'
        elif self._samples:
            state = 'monitoring'
        else:
            state = 'idle'
        return {'state': state, 'events': self._events, 'bumper_events': self._bumper_events,
                'window_resets': self._resets, 'window_elapsed_s': round(self._window_elapsed, 3),
                'window_translation_m': round(self._window_trans, 4),
                'window_rotation_rad': round(self._window_rot, 4)}

    # ----------------------------------------------------------------- private

    def _start_cooldown(self, t: float) -> None:
        self._samples.clear()
        self._cooldown_until = t + self.cooldown_s
        self._log('INFO', f'cooldown until t={self._cooldown_until:.2f}')

    def _reset(self, reason: str) -> None:
        if self._samples:
            self._resets += 1
            self._log('INFO', f'window reset: {reason}')
        self._samples.clear()
        self._window_trans = self._window_rot = self._window_elapsed = 0.0
