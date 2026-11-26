"""ExplorationSession: decisions of autonomous frontier exploration (003-R10, R11, R32), no ROS imports.

WHY: `exploration_server` must repeat "take the best frontier, send it, retry once, blacklist" and decide when
the goal ends. Keeping that here makes it testable without a Nav2 server (D-18, D-19). The caller owns time,
the grid, the robot pose and the Nav2 goal: it passes `now` (seconds, any monotonic clock) and the current
grid to `step()`, executes the returned `Step` (SEND a goal, CANCEL the goal in flight, WAIT, DONE) and reports
the outcome of each Nav2 goal with `goal_result()`.

Rules (assumptions are named, they are not in the spec text):
- 003-R10: a frontier whose goal aborts or exceeds `frontier_timeout_s` is retried once, then blacklisted.
- 003-R10 (assumption A): a frontier that Nav2 REACHES but that is still a frontier in the next map is reached
  again; after `max_reaches` reaches it is blacklisted (otherwise it would be re-sent until the timeout).
- 003-R11: success when the finder returns no frontier; failure at the deadline (`explore_timeout_s`, or the
  goal's `timeout_s` when positive) or when blacklisted/seen > `max_blacklist_ratio`.
- 003-R11 (assumption B): "frontiers seen" counts distinct frontiers returned by the finder, merged within
  `blacklist_tolerance_m`, including the blacklisted ones.
- 003-R32: `suspend()` (a cancel) keeps the blacklist, the counters and the clock origin; the next `begin()`
  resumes and ignores its own timeout. `end()` and a DONE step discard the session. The clock keeps running
  while suspended (the origin does not restart, as the spec requires).
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from vacuum_grid.grid_view import GridView

from .frontier_finder import Frontier, FrontierFinder
from .types import Logger, null_logger

Point = Tuple[float, float]
Metrics = Callable[[Dict[str, float]], None]


class StepKind(enum.Enum):
    SEND = 'send'
    CANCEL = 'cancel'
    WAIT = 'wait'
    DONE = 'done'


@dataclass(frozen=True)
class Step:
    kind: StepKind
    frontier: Optional[Frontier] = None
    success: bool = False
    reason: str = ''
    cancel_goal: bool = False        # DONE only: a Nav2 goal is still in flight and must be cancelled
    done: int = 0                    # DONE only: counters of the finished session (the session is reset after)
    blacklisted: int = 0


@dataclass
class ExplorationConfig:
    frontier_timeout_s: float = 60.0
    explore_timeout_s: float = 900.0
    max_blacklist_ratio: float = 0.5
    max_reaches: int = 3
    min_frontier_cells: int = 8
    gain_weight: float = 0.05
    robot_radius_m: float = 0.16
    blacklist_tolerance_m: float = 0.3

    def validate(self) -> None:
        if self.frontier_timeout_s <= 0 or self.explore_timeout_s <= 0:
            raise ValueError('frontier_timeout_s and explore_timeout_s must be positive')
        if not 0.0 <= self.max_blacklist_ratio <= 1.0:
            raise ValueError('max_blacklist_ratio must be in [0, 1]')
        if self.max_reaches < 1:
            raise ValueError('max_reaches must be at least 1')


def _near(a: Point, b: Point, tolerance: float) -> bool:
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= tolerance


class ExplorationSession:
    def __init__(self, config: ExplorationConfig, finder: Optional[FrontierFinder] = None,
                 log: Logger = null_logger, metrics: Optional[Metrics] = None) -> None:
        config.validate()
        self._cfg = config
        self._log = log
        self._metrics = metrics or (lambda _v: None)
        self._finder = finder or FrontierFinder(
            min_frontier_cells=config.min_frontier_cells, gain_weight=config.gain_weight,
            robot_radius_m=config.robot_radius_m, blacklist_tolerance_m=config.blacklist_tolerance_m, log=log)
        self._reset()

    def _reset(self) -> None:
        self._open = False                 # a session exists (running or suspended)
        self._running = False
        self._start = 0.0
        self.deadline = 0.0
        self._blacklist: List[Point] = []
        self._seen: List[Point] = []
        self._reaches: List[List] = []     # [[point, count]]
        self._done = 0
        self._target: Optional[Frontier] = None
        self._attempts = 0
        self._sent_at: Optional[float] = None    # time the goal in flight was sent; None = nothing in flight
        self._remaining = 0
        self._now = 0.0

    # ---- lifecycle of a session ---------------------------------------------------------------------------
    def begin(self, now: float, timeout_s: float) -> bool:
        """Starts a session, or resumes the suspended one (003-R32). Returns True when resumed."""
        resumed = self._open
        if not resumed:
            self._reset()
            self._open = True
            self._start = now
            self.deadline = now + (timeout_s if timeout_s > 0 else self._cfg.explore_timeout_s)
        self._running = True
        self._now = now
        self._sent_at = None
        self._target = None
        self._attempts = 0
        self._log(f'exploration {"resumed" if resumed else "started"}: blacklisted={len(self._blacklist)} '
                  f'seen={len(self._seen)} deadline in {self.deadline - now:.1f} s')
        return resumed

    def suspend(self, now: float) -> None:
        """A cancel: the goal in flight is dropped, everything else is kept for the next `begin()` (003-R32)."""
        if self._open:
            self._running = False
            self._sent_at = None
            self._target = None
            self._log(f'exploration suspended at {now - self._start:.1f} s: blacklisted={len(self._blacklist)}')

    def end(self) -> None:
        self._reset()

    # ---- decisions --------------------------------------------------------------------------------------
    def step(self, now: float, grid: GridView, robot_xy: Point) -> Step:
        if not self._running:
            return Step(StepKind.WAIT, reason='not_running')
        self._now = now
        in_flight = self._sent_at is not None
        if now >= self.deadline:
            return self._finish(False, 'timeout', in_flight)
        if in_flight:
            if now - self._sent_at >= self._cfg.frontier_timeout_s:
                self._log(f'frontier goal timed out after {self._cfg.frontier_timeout_s:.0f} s: cancelling')
                self._sent_at = None
                self._attempt_failed()
                return Step(StepKind.CANCEL, reason='frontier_timeout')
            return Step(StepKind.WAIT, reason='goal_in_flight')
        frontiers = self._finder.find(grid, robot_xy, self._blacklist)
        for f in frontiers:
            self._see(f.centroid)
        self._remaining = len(frontiers)
        if self._seen and len(self._blacklist) / len(self._seen) > self._cfg.max_blacklist_ratio:
            return self._finish(False, 'blacklist_ratio', False)
        if not frontiers:
            return self._finish(True, 'no_frontier', False)
        retry = None
        if self._target is not None:
            retry = next((f for f in frontiers if _near(f.centroid, self._target.centroid,
                                                        self._cfg.blacklist_tolerance_m)), None)
        chosen = retry or frontiers[0]
        if retry is None:
            self._attempts = 0
        self._target = chosen
        self._sent_at = now
        self._log(f'frontier goal ({chosen.centroid[0]:.2f}, {chosen.centroid[1]:.2f}) size={chosen.size} '
                  f'attempt={self._attempts + 1} remaining={len(frontiers)}')
        return Step(StepKind.SEND, frontier=chosen)

    def goal_result(self, now: float, reached: bool) -> None:
        """Outcome of the Nav2 goal in flight. Ignored when no goal is in flight (late reply of a cancelled one)."""
        if self._sent_at is None or self._target is None or not self._running:
            return
        self._sent_at = None
        if not reached:
            self._attempt_failed()
            return
        self._done += 1
        point = self._target.centroid
        entry = next((e for e in self._reaches if _near(e[0], point, self._cfg.blacklist_tolerance_m)), None)
        if entry is None:
            entry = [point, 0]
            self._reaches.append(entry)
        entry[1] += 1
        if entry[1] >= self._cfg.max_reaches:
            self._log(f'frontier ({point[0]:.2f}, {point[1]:.2f}) reached {entry[1]} times and still a frontier')
            self._blacklist_target()
        else:
            self._target = None
            self._attempts = 0

    def snapshot(self) -> Dict[str, float]:
        return {'seen': len(self._seen), 'blacklisted': len(self._blacklist), 'done': self._done,
                'remaining': self._remaining, 'attempts': self._attempts,
                'elapsed_s': max(0.0, self._now - self._start) if self._open else 0.0}

    @property
    def blacklisted(self) -> int:
        return len(self._blacklist)

    @property
    def done(self) -> int:
        return self._done

    @property
    def running(self) -> bool:
        return self._running

    @property
    def start_time(self) -> float:
        return self._start

    # ---- internals --------------------------------------------------------------------------------------
    def _see(self, point: Point) -> None:
        if not any(_near(point, s, self._cfg.blacklist_tolerance_m) for s in self._seen):
            self._seen.append(point)

    def _attempt_failed(self) -> None:
        self._attempts += 1
        if self._attempts >= 2:                       # the first attempt plus one retry (003-R10)
            self._blacklist_target()

    def _blacklist_target(self) -> None:
        if self._target is not None:
            self._blacklist.append(self._target.centroid)
            self._log(f'frontier ({self._target.centroid[0]:.2f}, {self._target.centroid[1]:.2f}) blacklisted '
                      f'({len(self._blacklist)} of {len(self._seen)} seen)')
        self._target = None
        self._attempts = 0

    def _finish(self, success: bool, reason: str, cancel_goal: bool) -> Step:
        self._log(f'exploration done success={success} reason={reason} done={self._done} '
                  f'blacklisted={len(self._blacklist)} seen={len(self._seen)}')
        self._metrics({'done': float(self._done), 'blacklisted': float(len(self._blacklist))})
        step = Step(StepKind.DONE, success=success, reason=reason, cancel_goal=cancel_goal,
                    done=self._done, blacklisted=len(self._blacklist))
        self._reset()
        return step
