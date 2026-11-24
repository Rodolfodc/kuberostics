"""ManagerSequencer (T521, spec 005, 005-R35 to R37, D-26): pure Python, no ROS imports.

It decides in which order the lifecycle managers are started and stopped and what a failure means. It does
not talk to ROS and it never sleeps: the caller injects

  call(manager, action, timeout_s) -> Outcome      sends one command to a manager (blocks up to timeout_s)
  wait_active(manager, timeout_s)  -> Outcome      waits until the manager reports is_active (up to timeout_s)
  clock()                          -> float        monotonic seconds
  log(level, message)                              level is 'info', 'warn' or 'error'
  notify(severity, code, text)                     a user-facing Notification

All the waiting happens inside the injected callables, which makes the class testable with a simulated clock.
The thin ROS node is `sequencer_node.py`.

Roles (what a failure of the manager means, 005-R36) are given by the manager name, see MANAGER_ROLES. To add a
manager: add it there, to the `managers` parameter and, if it needs a new failure rule, to `_on_failure`.
"""
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Callable, Dict, List, Optional, Sequence


class Action(Enum):
    STARTUP = 'startup'      # configure + activate every node of the manager
    PAUSE = 'pause'          # deactivate (can be resumed)
    RESET = 'reset'          # deactivate + cleanup
    SHUTDOWN = 'shutdown'    # deactivate + cleanup + shutdown
    # Deactivate, one by one, every node of the manager that is still active. Needed because the Nav2 manager
    # of Jazzy aborts PAUSE/RESET at the first node that cannot make the transition (the one that failed to
    # activate) and leaves the others active.
    DEACTIVATE_NODES = 'deactivate_nodes'


class Outcome(Enum):
    OK = 'OK'
    FAILED = 'FAILED'
    TIMEOUT = 'TIMEOUT'
    SKIPPED = 'SKIPPED'


class Role(Enum):
    CORE = 'core'
    NAV = 'nav'
    OPTIONAL = 'optional'


class State(Enum):
    NOT_STARTED = 'NOT_STARTED'
    STARTING_CORE = 'STARTING_CORE'
    STARTING_NAV = 'STARTING_NAV'
    STARTING_OPTIONAL = 'STARTING_OPTIONAL'
    ALL_ACTIVE = 'ALL_ACTIVE'
    FAILED_CORE = 'FAILED_CORE'
    DEGRADED_NAV = 'DEGRADED_NAV'
    DEGRADED_OPTIONAL = 'DEGRADED_OPTIONAL'
    SHUTTING_DOWN = 'SHUTTING_DOWN'


class Severity(IntEnum):
    """Same values as vacuum_interfaces/msg/Notification."""
    INFO = 0
    WARN = 1
    ERROR = 2


@dataclass(frozen=True)
class ManagerSpec:
    name: str
    role: Role


@dataclass(frozen=True)
class StepResult:
    manager: str
    action: Action
    outcome: Outcome
    duration_s: float = 0.0
    detail: str = ''


MANAGER_ROLES: Dict[str, Role] = {
    'lifecycle_manager_core': Role.CORE,
    'lifecycle_manager_nav': Role.NAV,
    'lifecycle_manager_optional': Role.OPTIONAL,
}

_STARTING = {Role.CORE: State.STARTING_CORE, Role.NAV: State.STARTING_NAV,
             Role.OPTIONAL: State.STARTING_OPTIONAL}
# worst first: the reported state when several managers failed
_SEVERITY_ORDER = [State.FAILED_CORE, State.DEGRADED_NAV, State.DEGRADED_OPTIONAL, State.ALL_ACTIVE]


def default_specs(names: Sequence[str]) -> List[ManagerSpec]:
    """Ordered specs from manager names. The first must be the core manager; unknown names are rejected."""
    if not names:
        raise ValueError('the manager list is empty')
    specs = []
    for name in names:
        if name not in MANAGER_ROLES:
            raise ValueError(f'unknown manager {name!r}; known: {sorted(MANAGER_ROLES)} (add it to MANAGER_ROLES)')
        specs.append(ManagerSpec(name, MANAGER_ROLES[name]))
    if specs[0].role is not Role.CORE:
        raise ValueError(f'the first manager must be the core manager, got {specs[0].name!r}')
    return specs


class GuardWatch:
    """Decides from samples of the stop_guard lifecycle state whether it was lost (005-R36).

    Samples are 'active', 'not_active' or 'unreachable' (no answer from its get_state service). A node that is
    not active is a loss at once. An unreachable one is a loss only after `unreachable_limit` unreachable samples
    in a row that are at least `min_spacing_s` apart: samples closer than that (for example triggered by a burst
    of transition events during a hiccup) are not counted, so one hiccup cannot look like two. Pure: the caller
    takes the samples, passes the time and decides what to do with the answer.
    """

    def __init__(self, unreachable_limit: int = 2, min_spacing_s: float = 0.0):
        self._limit = unreachable_limit
        self._spacing = min_spacing_s
        self._unreachable = 0
        self._last_counted = None

    @property
    def unreachable_count(self) -> int:
        return self._unreachable

    def feed(self, sample: str, now: float = 0.0) -> bool:
        if sample == 'active':
            self._unreachable = 0
            self._last_counted = None
            return False
        if sample == 'unreachable':
            if self._last_counted is not None and now - self._last_counted < self._spacing:
                return self._unreachable >= self._limit          # too close to the last one: not counted
            self._last_counted = now
            self._unreachable += 1
            return self._unreachable >= self._limit
        return True


def _noop_log(level: str, message: str) -> None:
    pass


def _noop_notify(severity: Severity, code: str, text: str) -> None:
    pass


class ManagerSequencer:
    def __init__(self, specs: Sequence[ManagerSpec],
                 call: Callable[[str, Action, float], Outcome],
                 wait_active: Callable[[str, float], Outcome],
                 clock: Callable[[], float],
                 log: Callable[[str, str], None] = _noop_log,
                 notify: Callable[[Severity, str, str], None] = _noop_notify,
                 startup_timeout_s: float = 30.0,
                 shutdown_timeout_s: float = 15.0):
        self._specs = list(specs)
        self._call = call
        self._wait_active = wait_active
        self._clock = clock
        self._log = log
        self._notify = notify
        self._startup_timeout_s = startup_timeout_s
        self._shutdown_timeout_s = shutdown_timeout_s
        self._state = State.NOT_STARTED
        self._results: List[StepResult] = []
        self._attempted: List[ManagerSpec] = []      # managers whose startup was tried, in order
        self._shutdown_results: Optional[List[StepResult]] = None
        self._abort = False
        self._stop_guard_lost = False
        self._guard_lost_pending = False     # lost before startup finished: handled when it does
        self._notifications = 0
        self._codes: List[str] = []

    # ------------------------------------------------------------ read-only view
    @property
    def state(self) -> State:
        return self._state

    @property
    def results(self) -> List[StepResult]:
        return list(self._results)

    def metrics(self) -> Dict[str, object]:
        data: Dict[str, object] = {'state': self._state.value, 'notifications': self._notifications,
                                   'codes': ','.join(self._codes)}
        for r in self._results:
            data[f'{r.action.value}_{r.manager}_s'] = r.duration_s
            data[f'{r.action.value}_{r.manager}_outcome'] = r.outcome.value
        return data

    # ------------------------------------------------------------ startup (005-R35, R36)
    def request_abort(self) -> None:
        """Safe to call from another thread: no manager is started after the current step."""
        self._abort = True

    @property
    def aborted(self) -> bool:
        return self._abort

    def run_startup(self) -> State:
        failed = State.ALL_ACTIVE
        core_failed = False
        for spec in self._specs:
            if core_failed or self._abort:
                reason = 'core failed' if core_failed else 'aborted'
                self._results.append(StepResult(spec.name, Action.STARTUP, Outcome.SKIPPED, 0.0, reason))
                self._log('warn', f'{spec.name}: not started ({reason})')
                continue
            self._set_state(_STARTING[spec.role])
            self._attempted.append(spec)
            result = self._start_one(spec)
            self._results.append(result)
            if result.outcome is Outcome.OK:
                self._log('info', f'{spec.name}: active after {result.duration_s:.2f} s')
                continue
            self._log('error' if spec.role is Role.CORE else 'warn',
                      f'{spec.name}: {result.outcome.value} ({result.detail}) after {result.duration_s:.2f} s')
            state = self._on_failure(spec)
            if state is State.FAILED_CORE:
                core_failed = True
            failed = min(failed, state, key=_SEVERITY_ORDER.index)
        if not self._abort:
            self._set_state(failed)
            # Safe guard: the node's worker handles guard events only after startup, so this branch is not
            # reached from it today; it keeps the core safe for any caller that reports a loss from another thread.
            if self._guard_lost_pending:
                self._guard_lost_pending = False
                self.on_stop_guard_lost()
        return self._state

    def _start_one(self, spec: ManagerSpec) -> StepResult:
        t0 = self._clock()
        deadline = t0 + self._startup_timeout_s
        outcome = self._call(spec.name, Action.STARTUP, self._startup_timeout_s)
        detail = 'startup command'
        if outcome is Outcome.OK:
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                outcome, detail = Outcome.TIMEOUT, 'startup command used the whole timeout'
            else:
                outcome = self._wait_active(spec.name, remaining)
                detail = 'is_active'
        return StepResult(spec.name, Action.STARTUP, outcome, self._clock() - t0,
                          detail if outcome is not Outcome.OK else '')

    def _on_failure(self, spec: ManagerSpec) -> State:
        """The rule of 005-R36 for the role of the failed manager."""
        if spec.role is Role.CORE:
            self._emit(Severity.ERROR, 'CORE_START_FAILED',
                       f'{spec.name} did not become active: nothing else is started')
            return State.FAILED_CORE
        if spec.role is Role.NAV:
            self._emit(Severity.WARN, 'NAV_UNAVAILABLE',
                       f'{spec.name} did not become active: no automatic motion until it does')
            self._deactivate_nav(spec, Action.RESET)         # a partly started nav stack must not stay active
            return State.DEGRADED_NAV
        self._emit(Severity.WARN, 'OPTIONAL_UNAVAILABLE',
                   f'{spec.name} did not become active: running without it (Art. VII)')
        return State.DEGRADED_OPTIONAL

    # ------------------------------------------------------------ runtime event (005-R36)
    def on_stop_guard_lost(self) -> None:
        if self._stop_guard_lost or self._abort or self._shutdown_results is not None:
            return
        if self._state is State.NOT_STARTED or self._state in _STARTING.values():
            # Art. VI: never drop it. nav must not stay active without the stop lock guarantee.
            self._log('warn', f'stop_guard lost in state {self._state.value}: handled when startup ends')
            self._guard_lost_pending = True
            return
        if self._state in (State.FAILED_CORE, State.SHUTTING_DOWN):
            self._log('info', f'stop_guard lost ignored in state {self._state.value} (nav is not running)')
            return
        self._stop_guard_lost = True
        self._emit(Severity.ERROR, 'STOP_GUARD_LOST', 'stop_guard is no longer active: nav deactivated')
        for spec in self._specs:
            if spec.role is Role.NAV:
                self._deactivate_nav(spec, Action.PAUSE)
        self._set_state(State.DEGRADED_NAV)

    # ------------------------------------------------------------ shutdown (005-R37)
    def run_shutdown(self) -> List[StepResult]:
        """Stops the started managers in reverse order (core last). Idempotent."""
        if self._shutdown_results is not None:
            self._log('info', 'shutdown already done: ignored')
            return list(self._shutdown_results)
        self._abort = True
        results: List[StepResult] = []
        self._shutdown_results = results
        if not self._attempted:
            self._log('info', 'shutdown: no manager was started')
            return results
        self._set_state(State.SHUTTING_DOWN)
        t0 = self._clock()
        remaining_managers = len(self._attempted)
        for spec in reversed(self._attempted):
            # the unused time of a step rolls over to the next, the total stays within the budget
            budget = max(0.0, self._shutdown_timeout_s - (self._clock() - t0))
            share = budget / remaining_managers
            remaining_managers -= 1
            ts = self._clock()
            outcome = self._call(spec.name, Action.SHUTDOWN, share)
            result = StepResult(spec.name, Action.SHUTDOWN, outcome, self._clock() - ts)
            results.append(result)
            self._log('info' if outcome is Outcome.OK else 'warn',
                      f'{spec.name}: shutdown {outcome.value} in {result.duration_s:.2f} s')
        self._results.extend(results)
        return list(results)

    # ------------------------------------------------------------ helpers
    def _deactivate_nav(self, spec: ManagerSpec, action: Action) -> None:
        self._best_effort(spec.name, action)
        self._best_effort(spec.name, Action.DEACTIVATE_NODES)

    def _best_effort(self, manager: str, action: Action) -> None:
        outcome = self._call(manager, action, self._startup_timeout_s)
        self._log('info' if outcome is Outcome.OK else 'warn', f'{manager}: {action.value} {outcome.value}')

    def _emit(self, severity: Severity, code: str, text: str) -> None:
        self._notifications += 1
        self._codes.append(code)
        self._log('info', f'notification {severity.name} {code}: {text}')
        self._notify(severity, code, text)

    def _set_state(self, state: State) -> None:
        if state is not self._state:
            self._log('info', f'state {self._state.value} -> {state.value}')
            self._state = state
