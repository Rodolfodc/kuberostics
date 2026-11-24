"""Unit tests of ManagerSequencer (T521, spec 005, D-26): pure Python, simulated clock, fake managers.

Every test name cites the requirement it verifies (Art. VIII).
"""
import pytest

from vacuum_bringup.manager_sequencer import (
    Action, GuardWatch, ManagerSequencer, ManagerSpec, Outcome, Role, Severity, State, default_specs)

CORE, NAV, OPT = 'lifecycle_manager_core', 'lifecycle_manager_nav', 'lifecycle_manager_optional'


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeManagers:
    """Scripted managers. `script[(manager, action)]` is an Outcome or (Outcome, seconds taken)."""

    def __init__(self, clock, script=None):
        self.clock = clock
        self.script = script or {}
        self.calls = []                  # (manager, action, timeout_s) in order
        self.waits = []                  # (manager, timeout_s)
        self.active_script = {}          # manager -> Outcome or (Outcome, seconds)
        self.after_wait = None           # optional hook(manager)
        self.hang_on_shutdown = False    # every stop uses the whole timeout it is given

    def _outcome(self, value):
        outcome, seconds = value if isinstance(value, tuple) else (value, 0.0)
        self.clock.now += seconds
        return outcome

    def call(self, manager, action, timeout_s):
        self.calls.append((manager, action, timeout_s))
        if self.hang_on_shutdown and action is Action.SHUTDOWN:
            self.clock.now += timeout_s
            return Outcome.TIMEOUT
        return self._outcome(self.script.get((manager, action), Outcome.OK))

    def wait_active(self, manager, timeout_s):
        self.waits.append((manager, timeout_s))
        outcome = self._outcome(self.active_script.get(manager, Outcome.OK))
        if self.after_wait:
            self.after_wait(manager)
        return outcome


class Recorder:
    def __init__(self):
        self.notes = []
        self.logs = []

    def notify(self, severity, code, text):
        self.notes.append((severity, code))

    def log(self, level, message):
        self.logs.append((level, message))


def build(script=None, specs=None, startup_timeout_s=30.0, shutdown_timeout_s=15.0):
    clock = FakeClock()
    managers = FakeManagers(clock, script)
    rec = Recorder()
    seq = ManagerSequencer(specs or default_specs([CORE, NAV, OPT]), managers.call, managers.wait_active,
                           clock, rec.log, rec.notify, startup_timeout_s=startup_timeout_s,
                           shutdown_timeout_s=shutdown_timeout_s)
    return seq, managers, rec, clock


def startups(managers):
    return [m for m, a, _ in managers.calls if a is Action.STARTUP]


def pairs(managers):
    return [(m, a) for m, a, _ in managers.calls]


def test_005_R35_starts_core_then_nav_then_optional_each_after_the_previous_is_active():
    events = []
    clock = FakeClock()

    def call(manager, action, timeout_s):
        events.append(('call', manager))
        return Outcome.OK

    def wait(manager, timeout_s):
        events.append(('wait', manager))
        return Outcome.OK

    seq = ManagerSequencer(default_specs([CORE, NAV, OPT]), call, wait, clock)
    assert seq.state is State.NOT_STARTED
    assert seq.run_startup() is State.ALL_ACTIVE
    # the next manager is not called before the previous one reported is_active
    assert events == [('call', CORE), ('wait', CORE), ('call', NAV), ('wait', NAV), ('call', OPT), ('wait', OPT)]


def test_005_R35_manager_that_never_becomes_active_is_declared_failed_after_the_timeout():
    seq, managers, rec, clock = build()
    managers.active_script[NAV] = (Outcome.TIMEOUT, 30.0)
    seq.run_startup()
    result = next(r for r in seq.results if r.manager == NAV and r.action is Action.STARTUP)
    assert result.outcome is Outcome.TIMEOUT
    assert dict(managers.waits)[NAV] <= 30.0
    assert clock.now == pytest.approx(30.0)
    assert seq.state is State.DEGRADED_NAV


def test_005_R35_wait_budget_is_what_is_left_of_the_startup_timeout_after_the_call():
    seq, managers, _, _ = build({(CORE, Action.STARTUP): (Outcome.OK, 12.0)}, startup_timeout_s=30.0)
    seq.run_startup()
    assert dict(managers.waits)[CORE] == pytest.approx(18.0)
    assert managers.calls[0][2] == pytest.approx(30.0)


def test_005_R35_a_call_that_takes_the_whole_timeout_leaves_no_wait_and_fails_the_step():
    seq, managers, _, _ = build({(NAV, Action.STARTUP): (Outcome.TIMEOUT, 30.0)})
    seq.run_startup()
    assert NAV not in dict(managers.waits)
    assert next(r for r in seq.results if r.manager == NAV).outcome is Outcome.TIMEOUT


def test_005_R36_core_failure_does_not_start_nav_or_optional():
    seq, managers, rec, _ = build({(CORE, Action.STARTUP): Outcome.FAILED})
    assert seq.run_startup() is State.FAILED_CORE
    assert startups(managers) == [CORE]
    assert (Severity.ERROR, 'CORE_START_FAILED') in rec.notes
    assert [r.manager for r in seq.results if r.outcome is Outcome.SKIPPED] == [NAV, OPT]


def test_005_R36_core_that_never_becomes_active_is_also_failed_core():
    seq, managers, rec, _ = build()
    managers.active_script[CORE] = Outcome.TIMEOUT
    assert seq.run_startup() is State.FAILED_CORE
    assert startups(managers) == [CORE]


def test_005_R36_nav_failure_keeps_core_leaves_nav_inactive_and_still_tries_optional():
    seq, managers, rec, _ = build({(NAV, Action.STARTUP): Outcome.FAILED})
    assert seq.run_startup() is State.DEGRADED_NAV
    assert startups(managers) == [CORE, NAV, OPT]
    assert (Severity.WARN, 'NAV_UNAVAILABLE') in rec.notes
    # a partially started nav manager is reset, so no nav node is left active
    assert (NAV, Action.RESET) in pairs(managers)
    # ... and whatever the manager left active is deactivated node by node (Jazzy aborts the reset early)
    assert (NAV, Action.DEACTIVATE_NODES) in pairs(managers)
    assert pairs(managers).index((NAV, Action.RESET)) < pairs(managers).index((NAV, Action.DEACTIVATE_NODES))
    assert (CORE, Action.RESET) not in pairs(managers)
    assert (CORE, Action.DEACTIVATE_NODES) not in pairs(managers)


def test_005_R36_optional_failure_does_not_stop_core_or_nav():
    seq, managers, rec, _ = build({(OPT, Action.STARTUP): Outcome.FAILED})
    assert seq.run_startup() is State.DEGRADED_OPTIONAL
    assert startups(managers) == [CORE, NAV, OPT]
    assert [n for n in rec.notes if n[0] is Severity.ERROR] == []
    assert [(m, a) for m, a, _t in managers.calls
            if m != OPT and a in (Action.RESET, Action.SHUTDOWN, Action.PAUSE, Action.DEACTIVATE_NODES)] == []


def test_005_R36_nav_and_optional_both_failing_reports_the_worse_state():
    seq, _, rec, _ = build({(NAV, Action.STARTUP): Outcome.FAILED, (OPT, Action.STARTUP): Outcome.FAILED})
    assert seq.run_startup() is State.DEGRADED_NAV
    assert {c for _, c in rec.notes} >= {'NAV_UNAVAILABLE', 'OPTIONAL_UNAVAILABLE'}


def test_005_R36_stop_guard_lost_while_running_deactivates_nav_and_raises_error():
    seq, managers, rec, _ = build()
    seq.run_startup()
    seq.on_stop_guard_lost()
    assert (NAV, Action.PAUSE) in pairs(managers)
    assert (NAV, Action.DEACTIVATE_NODES) in pairs(managers)
    assert (Severity.ERROR, 'STOP_GUARD_LOST') in rec.notes
    assert seq.state is State.DEGRADED_NAV


def test_005_R36_stop_guard_lost_is_reported_once():
    seq, managers, rec, _ = build()
    seq.run_startup()
    seq.on_stop_guard_lost()
    seq.on_stop_guard_lost()
    assert rec.notes.count((Severity.ERROR, 'STOP_GUARD_LOST')) == 1
    assert [a for _, a, _ in managers.calls].count(Action.PAUSE) == 1
    assert [a for _, a, _ in managers.calls].count(Action.DEACTIVATE_NODES) == 1


def test_005_R36_stop_guard_lost_in_failed_core_does_nothing():
    seq, managers, rec, _ = build({(CORE, Action.STARTUP): Outcome.FAILED})
    seq.run_startup()
    before = len(managers.calls)
    seq.on_stop_guard_lost()
    assert len(managers.calls) == before
    assert (Severity.ERROR, 'STOP_GUARD_LOST') not in rec.notes


def test_005_R36_stop_guard_lost_during_startup_is_handled_when_startup_ends():
    seq, managers, rec, _ = build()
    managers.after_wait = lambda m: seq.on_stop_guard_lost() if m == NAV else None    # lost while starting nav
    seq.run_startup()
    assert (Severity.ERROR, 'STOP_GUARD_LOST') in rec.notes
    assert (NAV, Action.PAUSE) in pairs(managers)
    assert (NAV, Action.DEACTIVATE_NODES) in pairs(managers)
    assert pairs(managers).index((OPT, Action.STARTUP)) < pairs(managers).index((NAV, Action.PAUSE))
    assert seq.state is State.DEGRADED_NAV


def test_005_R36_stop_guard_lost_before_startup_is_not_forgotten():
    seq, managers, rec, _ = build()
    seq.on_stop_guard_lost()
    seq.run_startup()
    assert (Severity.ERROR, 'STOP_GUARD_LOST') in rec.notes
    assert (NAV, Action.PAUSE) in pairs(managers)


def test_005_R37_shutdown_goes_in_reverse_order_core_last():
    seq, managers, _, _ = build()
    seq.run_startup()
    managers.calls.clear()
    seq.run_shutdown()
    assert pairs(managers) == [(OPT, Action.SHUTDOWN), (NAV, Action.SHUTDOWN), (CORE, Action.SHUTDOWN)]
    assert seq.state is State.SHUTTING_DOWN


def test_005_R37_a_failing_stop_does_not_prevent_the_next_one():
    seq, managers, _, _ = build({(NAV, Action.SHUTDOWN): Outcome.FAILED, (OPT, Action.SHUTDOWN): Outcome.FAILED})
    seq.run_startup()
    managers.calls.clear()
    results = seq.run_shutdown()
    assert [m for m, _ in pairs(managers)] == [OPT, NAV, CORE]
    assert [r.outcome for r in results] == [Outcome.FAILED, Outcome.FAILED, Outcome.OK]


def test_005_R37_shutdown_ends_within_manager_shutdown_timeout_and_still_tries_every_manager():
    seq, managers, _, clock = build({}, shutdown_timeout_s=15.0)
    seq.run_startup()
    managers.hang_on_shutdown = True
    t0 = clock.now
    results = seq.run_shutdown()
    assert clock.now - t0 <= 15.0 + 1e-9
    assert [r.manager for r in results] == [OPT, NAV, CORE]      # stop_guard's manager is not skipped
    assert all(r.outcome is Outcome.TIMEOUT for r in results)


def test_005_R37_unused_shutdown_time_rolls_over_to_the_next_manager():
    seq, managers, _, _ = build(shutdown_timeout_s=15.0)
    seq.run_startup()
    managers.calls.clear()
    seq.run_shutdown()
    assert [t for _, _, t in managers.calls] == pytest.approx([5.0, 7.5, 15.0])


def test_005_R37_shutdown_after_core_failure_stops_only_what_was_started():
    seq, managers, _, _ = build({(CORE, Action.STARTUP): Outcome.FAILED})
    seq.run_startup()
    managers.calls.clear()
    seq.run_shutdown()
    assert pairs(managers) == [(CORE, Action.SHUTDOWN)]


def test_005_R37_double_shutdown_is_a_no_op_that_returns_the_same_results():
    seq, managers, rec, _ = build()
    seq.run_startup()
    first = seq.run_shutdown()
    n = len(managers.calls)
    assert seq.run_shutdown() == first
    assert len(managers.calls) == n


def test_005_R37_shutdown_before_startup_calls_nothing():
    seq, managers, _, _ = build()
    assert seq.run_shutdown() == []
    assert managers.calls == []


def test_005_R37_abort_during_startup_skips_the_rest_and_shutdown_stops_only_started_managers():
    seq, managers, _, _ = build()
    managers.after_wait = lambda m: seq.request_abort() if m == CORE else None
    seq.run_startup()
    assert startups(managers) == [CORE]
    managers.calls.clear()
    seq.run_shutdown()
    assert pairs(managers) == [(CORE, Action.SHUTDOWN)]


def test_005_R35_every_transition_is_logged():
    seq, _, rec, _ = build({(NAV, Action.STARTUP): Outcome.FAILED})
    seq.run_startup()
    seq.run_shutdown()
    text = '\n'.join(m for _, m in rec.logs)
    for state in ('STARTING_CORE', 'STARTING_NAV', 'STARTING_OPTIONAL', 'DEGRADED_NAV', 'SHUTTING_DOWN'):
        assert state in text


def test_005_R35_metrics_report_state_and_per_step_durations():
    seq, _, _, _ = build({(CORE, Action.STARTUP): (Outcome.OK, 2.0)})
    seq.run_startup()
    m = seq.metrics()
    assert m['state'] == 'ALL_ACTIVE'
    assert m['startup_lifecycle_manager_core_s'] == pytest.approx(2.0)
    assert m['notifications'] == 0


def test_005_R36_metrics_carry_the_notification_codes_so_periodic_telemetry_shows_them():
    seq, _, _, _ = build({(NAV, Action.STARTUP): Outcome.FAILED})
    seq.run_startup()
    assert seq.metrics()['codes'] == 'NAV_UNAVAILABLE'


def test_005_R35_the_manager_list_comes_from_configuration():
    seq, managers, _, _ = build(specs=default_specs([CORE, OPT]))
    assert seq.run_startup() is State.ALL_ACTIVE
    assert startups(managers) == [CORE, OPT]


def test_005_R35_specs_reject_unknown_names_and_a_first_manager_that_is_not_core():
    with pytest.raises(ValueError):
        default_specs([CORE, 'lifecycle_manager_other'])
    with pytest.raises(ValueError):
        default_specs([NAV, CORE])
    with pytest.raises(ValueError):
        default_specs([])
    assert default_specs([CORE, NAV, OPT]) == [
        ManagerSpec(CORE, Role.CORE), ManagerSpec(NAV, Role.NAV), ManagerSpec(OPT, Role.OPTIONAL)]


def test_005_R36_guard_watch_a_stop_guard_that_is_not_active_is_lost_at_once():
    assert GuardWatch().feed('not_active') is True


def test_005_R36_guard_watch_one_unreachable_sample_is_not_a_loss_two_in_a_row_are():
    watch = GuardWatch()
    assert watch.feed('unreachable') is False
    assert watch.feed('active') is False          # an answer resets the count
    assert watch.feed('unreachable') is False
    assert watch.feed('unreachable') is True


def test_005_R36_guard_watch_does_not_depend_on_having_seen_the_activation_event():
    # the first sample ever is already 'not_active': it is a loss (a missed event must not hide it)
    assert GuardWatch().feed('not_active') is True
