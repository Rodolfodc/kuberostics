"""EscapeScript: the deterministic scripted escape (spec 005, 005-R08 to R11). No ROS imports (D-18).

Toolbox class (D-19): it only computes the next velocity (or the final outcome) from the pose and the clearances
that the caller passes in. It owns no clock (the caller passes `t`), reads no topic, publishes nothing and does not
enforce the goal timeout (005-R12 is the caller's job). Poses are `(x, y, yaw)`; distances and angles come from
the odometry, so a robot that does not move is bounded by the budgets, never by the script.

State machine (every transition is logged, D-20)::

    REVERSE -> TURN -> (front clear? DONE success : next attempt: REVERSE ...)
    primary ends at max_attempts or primary_budget_s, whichever is first (an attempt in progress is abandoned)
    FALLBACK_SWEEP (360 deg, record front clearance per heading) -> FALLBACK_FACE (largest clearance, 0..180 deg)
        -> front clear? FALLBACK_ADVANCE : DONE failure -> DONE (success unless the advance met an obstacle)
    fallback_budget_s expired -> DONE failure

Left/right of the turn is decided once, at `start`, from the scan of the stuck event (mean range per side, tie to
the left); the same direction is used by every attempt and by the sweep.

Front clearance: `front_range` is the minimum range in the front sector, computed by the caller (see
`scan_utils.sector_min`). `None` means "no valid reading" and is NOT a clear front. `rear_range=None` means the
scan does not cover the rear: the rear check is skipped.
"""
import math
from dataclasses import dataclass
from typing import Callable, List, NamedTuple, Optional, Sequence, Tuple, Union

from vacuum_coverage.core.scan_utils import side_means, wrap_angle

Pose = Tuple[float, float, float]


def _noop(*_args) -> None:
    return None


class Command(NamedTuple):
    linear: float       # m/s
    angular: float      # rad/s


class Result(NamedTuple):
    success: bool
    reason: str
    phase: str          # 'primary' or 'fallback'


@dataclass(frozen=True)
class EscapeConfig:
    """Parameters of the script. Defaults are those of spec 005 (005-R08 to R12)."""
    reverse_m: float = 0.20
    rear_clearance_m: float = 0.15
    turn_deg_base: float = 90.0
    turn_deg_step: float = 45.0
    front_clearance_m: float = 0.40
    max_attempts: int = 3
    fallback_forward_m: float = 0.30
    reverse_speed_m_s: float = 0.15
    turn_speed_rad_s: float = 1.0
    primary_budget_s: float = 5.0
    fallback_budget_s: float = 11.5
    default_timeout_s: float = 17.0
    front_stop_m: float = 0.15            # the fallback advance stops (and fails) below this front range
    angle_tolerance_rad: float = 0.05     # a turn ends when it is within this of the target (about one control tick)
    distance_tolerance_m: float = 0.01
    range_cap_m: float = 5.0              # readings above this are capped when comparing the sides of the scan
    # Wheel-bridge limits: PLACEHOLDERS until the wheel-bridge measurement in the simulator. The linear one is the 0.30 m/s limit of
    # spec 009a; the angular one equals the default turn speed. Replace with the measured values.
    max_linear_speed_m_s: float = 0.30
    max_angular_speed_rad_s: float = 1.0

    def fallback_worst_case_s(self) -> float:
        """Sweep 360 degrees + facing turn of 180 degrees + advance, at the configured speeds."""
        w, v = self.turn_speed_rad_s, self.reverse_speed_m_s
        return 2.0 * math.pi / w + math.pi / w + self.fallback_forward_m / v

    def validate(self) -> List[str]:
        """Problems that make the configuration unusable (empty list when it is valid; 005-R11 c)."""
        bad = []
        for name in ('reverse_m', 'rear_clearance_m', 'front_clearance_m', 'fallback_forward_m',
                     'reverse_speed_m_s', 'turn_speed_rad_s', 'primary_budget_s', 'fallback_budget_s',
                     'default_timeout_s', 'turn_deg_base', 'range_cap_m'):
            if not getattr(self, name) > 0.0:
                bad.append(f'{name} must be > 0')
        if self.max_attempts < 1:
            bad.append('max_attempts must be >= 1')
        if self.turn_deg_step < 0.0:
            bad.append('turn_deg_step must be >= 0')
        if bad:
            return bad
        if self.reverse_speed_m_s > self.max_linear_speed_m_s:
            bad.append(f'reverse_speed_m_s {self.reverse_speed_m_s} exceeds the wheel-bridge limit '
                       f'{self.max_linear_speed_m_s} m/s')
        if self.turn_speed_rad_s > self.max_angular_speed_rad_s:
            bad.append(f'turn_speed_rad_s {self.turn_speed_rad_s} exceeds the wheel-bridge limit '
                       f'{self.max_angular_speed_rad_s} rad/s')
        worst = self.fallback_worst_case_s()
        if worst > self.fallback_budget_s:
            bad.append(f'the fallback worst case ({worst:.2f} s) does not fit fallback_budget_s '
                       f'({self.fallback_budget_s} s)')
        if self.primary_budget_s + self.fallback_budget_s >= self.default_timeout_s:
            bad.append(f'primary_budget_s + fallback_budget_s ({self.primary_budget_s + self.fallback_budget_s} s) '
                       f'must be below default_timeout_s ({self.default_timeout_s} s)')
        return bad


class EscapeScript:
    def __init__(self, config: Optional[EscapeConfig] = None,
                 logger: Callable[[str, str], None] = _noop, metrics: Callable[[str, float], None] = _noop):
        self.config = config or EscapeConfig()
        problems = self.config.validate()
        if problems:
            raise ValueError('; '.join(problems))
        self._log = logger
        self._metrics = metrics
        self.state = 'IDLE'
        self._result: Optional[Result] = None

    # ------------------------------------------------------------------ public

    def start(self, event_ranges: Sequence[float], pose: Pose, angle_min: float, angle_increment: float) -> None:
        """Begins a new escape. `event_ranges` is `StuckEvent.scan_ranges` and `angle_min`/`angle_increment` its
        geometry (the event does not carry them, the caller takes them from the live laser)."""
        left, right = side_means(event_ranges, angle_min, angle_increment, self.config.range_cap_m)
        self._dir = 1.0 if left >= right else -1.0           # ties go left (005-R09)
        self._attempt = 0
        self._t0: Optional[float] = None
        self._t_fallback: Optional[float] = None
        self._result = None
        self._samples: List[Tuple[float, float]] = []        # (swept angle, front clearance)
        self._face_angle = 0.0
        self._face_dir = 1.0
        self._last_pose = pose
        self._log('INFO', f'escape start: left mean {left:.2f} m, right mean {right:.2f} m -> turn '
                          f'{"left" if self._dir > 0 else "right"}')
        self._enter('REVERSE', pose)

    def step(self, t: float, pose: Pose, front_range: Optional[float],
             rear_range: Optional[float]) -> Union[Command, Result]:
        if self.state == 'IDLE':
            raise RuntimeError('step() before start()')
        if self._result is not None:
            return self._result
        cfg = self.config
        if self._t0 is None:
            self._t0 = t
        self._yaw_acc += self._dir_free_dyaw(pose)
        self._last_pose = pose
        front_ok = front_range is not None and front_range >= cfg.front_clearance_m
        primary = self.state in ('REVERSE', 'TURN')
        if primary and t - self._t0 >= cfg.primary_budget_s:
            self._log('WARN', f'primary budget {cfg.primary_budget_s:.1f} s used up in {self.state} of attempt '
                              f'{self._attempt + 1}: attempt abandoned, running the fallback')
            self._start_fallback(t, pose)
            primary = False
        if not primary and self.state != 'DONE' and t - self._t_fallback >= cfg.fallback_budget_s:
            return self._finish(False, 'fallback budget exhausted', 'fallback')
        while True:
            if self.state == 'REVERSE':
                blocked = rear_range is not None and rear_range < cfg.rear_clearance_m
                if self._distance(pose) >= cfg.reverse_m - cfg.distance_tolerance_m or blocked:
                    self._log('INFO', f'reverse done ({self._distance(pose):.2f} m'
                                      f'{", rear blocked" if blocked else ""})')
                    self._enter('TURN', pose)
                    continue
                return Command(-cfg.reverse_speed_m_s, 0.0)
            if self.state == 'TURN':
                target = math.radians(cfg.turn_deg_base + self._attempt * cfg.turn_deg_step)
                if self._dir * self._yaw_acc >= target - cfg.angle_tolerance_rad:
                    if front_ok:
                        return self._finish(True, f'front clear ({front_range:.2f} m) after attempt '
                                                  f'{self._attempt + 1}', 'primary')
                    self._log('INFO', f'attempt {self._attempt + 1} failed: front range {front_range}')
                    self._attempt += 1
                    if self._attempt >= cfg.max_attempts:
                        self._log('WARN', f'{cfg.max_attempts} attempts exhausted, running the fallback')
                        self._start_fallback(t, pose)
                    else:
                        self._enter('REVERSE', pose)
                    continue
                return Command(0.0, self._dir * cfg.turn_speed_rad_s)
            if self.state == 'FALLBACK_SWEEP':
                if front_range is not None:
                    self._samples.append((self._dir * self._yaw_acc, min(front_range, cfg.range_cap_m)))
                if self._dir * self._yaw_acc >= 2.0 * math.pi - cfg.angle_tolerance_rad:
                    self._plan_facing()
                    self._enter('FALLBACK_FACE', pose)
                    continue
                return Command(0.0, self._dir * cfg.turn_speed_rad_s)
            if self.state == 'FALLBACK_FACE':
                if self._face_dir * self._yaw_acc >= self._face_angle - cfg.angle_tolerance_rad:
                    if not front_ok:
                        return self._finish(False, f'front blocked after facing (range {front_range})', 'fallback')
                    self._enter('FALLBACK_ADVANCE', pose)
                    continue
                return Command(0.0, self._face_dir * cfg.turn_speed_rad_s)
            if self.state == 'FALLBACK_ADVANCE':
                if front_range is not None and front_range < cfg.front_stop_m:
                    return self._finish(False, f'advance blocked by an obstacle (range {front_range:.2f} m)',
                                        'fallback')
                if self._distance(pose) >= cfg.fallback_forward_m - cfg.distance_tolerance_m:
                    return self._finish(True, f'advanced {self._distance(pose):.2f} m toward the free heading',
                                        'fallback')
                return Command(cfg.reverse_speed_m_s, 0.0)
            raise RuntimeError(f'unexpected state {self.state}')

    def snapshot(self) -> dict:
        res = self._result
        return {'state': self.state, 'attempt': getattr(self, '_attempt', 0),
                'success': None if res is None else res.success,
                'reason': '' if res is None else res.reason}

    # ----------------------------------------------------------------- private

    def _enter(self, state: str, pose: Pose) -> None:
        self._log('INFO', f'{self.state} -> {state}' + (f' (attempt {self._attempt + 1})'
                                                        if state in ('REVERSE', 'TURN') else ''))
        self.state = state
        self._entry_xy = pose[:2]
        self._yaw_acc = 0.0

    def _dir_free_dyaw(self, pose: Pose) -> float:
        return wrap_angle(pose[2] - self._last_pose[2])

    def _distance(self, pose: Pose) -> float:
        return math.hypot(pose[0] - self._entry_xy[0], pose[1] - self._entry_xy[1])

    def _start_fallback(self, t: float, pose: Pose) -> None:
        self._t_fallback = t
        self._samples = []
        self._enter('FALLBACK_SWEEP', pose)

    def _plan_facing(self) -> None:
        """Picks the heading of the largest front clearance seen during the sweep: the middle of the first run of
        samples at the maximum (so the robot faces the centre of an opening, not its edge)."""
        if not self._samples:
            self._face_angle, self._face_dir = 0.0, 1.0
            self._log('WARN', 'sweep recorded no front clearance: no facing turn')
            return
        best = max(c for _, c in self._samples)
        idx = [i for i, (_, c) in enumerate(self._samples) if c >= best - 1e-9]
        run = [idx[0]]
        for i in idx[1:]:
            if i != run[-1] + 1:
                break
            run.append(i)
        theta = self._samples[run[len(run) // 2]][0]          # swept angle in the sweep direction, 0..2pi
        theta = min(max(theta, 0.0), 2.0 * math.pi)
        if theta <= math.pi:
            self._face_angle, self._face_dir = theta, self._dir
        else:
            self._face_angle, self._face_dir = 2.0 * math.pi - theta, -self._dir
        self._log('INFO', f'largest clearance {best:.2f} m at {math.degrees(theta):.0f} deg of the sweep: '
                          f'facing turn {math.degrees(self._face_angle):.0f} deg '
                          f'{"left" if self._face_dir > 0 else "right"}')

    def _finish(self, success: bool, reason: str, phase: str) -> Result:
        self._result = Result(success, reason, phase)
        self._log('INFO' if success else 'WARN',
                  f'{self.state} -> DONE: {"success" if success else "failure"} ({phase}): {reason}')
        self.state = 'DONE'
        self._metrics('escape_success', 1 if success else 0)
        return self._result
