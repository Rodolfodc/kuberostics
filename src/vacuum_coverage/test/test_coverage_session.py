"""T506: CoverageSession (waypoint retry/skip/timeouts, early success) and ResumeStore (005-R20, R21, R32)."""
import pytest

from coverage_fixtures import rect_room
from vacuum_coverage.core.coverage_planner import Waypoint
from vacuum_coverage.core.coverage_session import (CoverageSession, ResumeStore, SessionLimits, StepKind)
from vacuum_coverage.core.coverage_tracker import CoverageTracker, coverable_mask
from vacuum_mapping.core.types import FREE


def wps(n, pass_index=0):
    return [Waypoint(0.5 + 0.1 * i, 0.5, 0.0, pass_index) for i in range(n)]


def session(waypoints, passes=1, tracker=None, **limits):
    raster = rect_room(3.0, 2.0)
    region = raster.cells == FREE
    tracker = tracker or CoverageTracker(raster, coverable_mask(raster, region, 0.21, 0.15), 0.15, region=region)
    limits.setdefault('min_acceptable_ratio', 0.0)     # retry/skip tests do not care about the ratio
    return CoverageSession(waypoints, tracker, passes, SessionLimits(**limits)), tracker


def drive(s, t, outcomes):
    """Send each waypoint and answer with the next outcome of `outcomes` (True reached, False aborted)."""
    sent = []
    outcomes = list(outcomes)
    while True:
        step = s.step(t)
        if step.kind is StepKind.DONE:
            return step, sent
        assert step.kind is StepKind.SEND
        sent.append(step.index)
        t += 1.0
        s.goal_result(t, outcomes.pop(0))


def test_005_R19_waypoints_are_sent_in_order_one_at_a_time():
    s, _ = session(wps(3))
    s.start(0.0)
    first = s.step(0.0)
    assert (first.kind, first.index) == (StepKind.SEND, 0)
    assert s.step(0.5).kind is StepKind.WAIT          # still in flight
    step, sent = drive(session(wps(3))[0], 0.0, [True, True, True])
    assert sent == [0, 1, 2] and step.success


def test_005_R20_aborted_waypoint_is_retried_once_then_skipped():
    s, _ = session(wps(2), max_skip_ratio=0.9)
    s.start(0.0)
    step, sent = drive(s, 0.0, [False, False, True])
    assert sent == [0, 0, 1]
    assert s.skipped_waypoints == 1
    assert step.success                                # exhausted, skips within the limit


def test_005_R20_retry_that_succeeds_does_not_count_as_a_skip():
    s, _ = session(wps(2))
    s.start(0.0)
    step, sent = drive(s, 0.0, [False, True, True])
    assert sent == [0, 0, 1] and s.skipped_waypoints == 0 and step.success


def test_005_R20_waypoint_timeout_cancels_the_goal_then_retries_and_skips():
    s, _ = session(wps(2), waypoint_timeout_s=10.0, max_skip_ratio=0.9)
    s.start(0.0)
    assert s.step(0.0).kind is StepKind.SEND
    assert s.step(9.9).kind is StepKind.WAIT
    cancel = s.step(10.1)
    assert cancel.kind is StepKind.CANCEL and cancel.index == 0
    retry = s.step(10.2)
    assert (retry.kind, retry.index) == (StepKind.SEND, 0)            # the one retry
    assert s.step(20.5).kind is StepKind.CANCEL
    nxt = s.step(20.6)
    assert (nxt.kind, nxt.index) == (StepKind.SEND, 1)                # skipped, moves on
    assert s.skipped_waypoints == 1


def test_005_R20_late_result_of_a_cancelled_goal_is_ignored():
    s, _ = session(wps(2), waypoint_timeout_s=1.0, max_skip_ratio=0.9)
    s.start(0.0)
    s.step(0.0)
    assert s.step(2.0).kind is StepKind.CANCEL
    s.goal_result(2.1, True)                          # arrives with nothing in flight
    assert s.current_index == 0 and s.skipped_waypoints == 0


def test_005_R20_too_many_skips_fail_the_goal():
    s, _ = session(wps(4), max_skip_ratio=0.3)
    s.start(0.0)
    step, sent = drive(s, 0.0, [False] * 8)
    assert not step.success and step.reason == 'skip_ratio'
    assert s.skipped_waypoints == 2                   # 2 > 0.3 * 4 = 1.2 at the second skip


def test_005_R20_skips_exactly_at_the_ratio_are_within_the_limit():
    s, _ = session(wps(10), max_skip_ratio=0.3)
    s.start(0.0)
    outcomes = [False, False] * 3 + [True] * 7          # three skips of ten = 0.3
    step, _ = drive(s, 0.0, outcomes)
    assert step.success and s.skipped_waypoints == 3


def test_005_R20_goal_exceeding_coverage_timeout_fails_and_cancels_the_inflight_goal():
    s, _ = session(wps(3), coverage_timeout_s=100.0)
    s.start(0.0)
    s.step(0.0)
    step = s.step(100.5)
    assert step.kind is StepKind.DONE and not step.success and step.reason == 'coverage_timeout'
    assert step.cancel_goal
    assert s.step(101.0).kind is StepKind.DONE         # stays finished


def test_005_R21_goal_succeeds_early_when_the_target_ratio_is_reached_on_the_last_pass():
    s, tracker = session(wps(5), target_ratio=0.3)
    s.start(0.0)
    s.step(0.0)
    for x in (0.7, 1.0, 1.4, 1.8, 2.2, 2.6):
        for y in (0.5, 0.9, 1.3, 1.6):
            tracker.mark_pose(x, y)
    assert tracker.covered_ratio >= 0.3
    step = s.step(1.0)
    assert step.kind is StepKind.DONE and step.success and step.reason == 'target_ratio'
    assert step.cancel_goal


def test_005_R21_target_ratio_is_not_checked_before_the_last_requested_pass():
    first_pass = wps(2, pass_index=0)
    second_pass = wps(2, pass_index=1)
    s, tracker = session(first_pass + second_pass, passes=2, target_ratio=0.0)
    s.start(0.0)
    assert s.step(0.0).kind is StepKind.SEND           # ratio >= 0.0 already, but this is pass 0
    s.goal_result(1.0, True)
    s.step(1.0)
    s.goal_result(2.0, True)
    step = s.step(2.0)                                   # now on pass 1: target reached
    assert step.kind is StepKind.DONE and step.success and step.reason == 'target_ratio'


def test_005_R21_goal_with_no_waypoints_succeeds_immediately():
    s, _ = session([])
    s.start(0.0)
    step = s.step(0.0)
    assert step.kind is StepKind.DONE and step.success and step.reason.startswith('exhausted')


def cover_fraction(tracker, fraction):
    """Mark cells until about `fraction` of the coverable cells are covered."""
    for x in [0.3 + 0.05 * i for i in range(50)]:
        for y in [0.3 + 0.05 * j for j in range(30)]:
            if tracker.covered_ratio >= fraction:
                return
            tracker.mark_pose(x, y)


def test_005_R21_default_limits_are_target_095_and_min_acceptable_070():
    lim = SessionLimits()
    assert lim.target_ratio == 0.95 and lim.min_acceptable_ratio == 0.70


def test_005_R21_exhausted_at_or_above_target_is_a_plain_success():
    s, tracker = session([], target_ratio=0.95, min_acceptable_ratio=0.70)
    cover_fraction(tracker, 0.96)
    s.start(0.0)
    step = s.step(0.0)
    assert step.success and not step.degraded and step.reason == 'exhausted'
    assert step.covered_ratio >= 0.95


def test_005_R21_exhausted_between_minimum_and_target_is_a_degraded_success_with_the_ratio():
    s, tracker = session([], target_ratio=0.95, min_acceptable_ratio=0.70)
    cover_fraction(tracker, 0.80)
    s.start(0.0)
    step = s.step(0.0)
    assert step.success and step.degraded and step.reason == 'exhausted_degraded'
    assert 0.70 <= step.covered_ratio < 0.95
    assert s.degraded and s.step(1.0).degraded               # stays reported once finished


def test_005_R21_exhausted_below_the_minimum_is_a_failure():
    s, tracker = session([], target_ratio=0.95, min_acceptable_ratio=0.70)
    cover_fraction(tracker, 0.40)
    s.start(0.0)
    step = s.step(0.0)
    assert not step.success and not step.degraded and step.reason == 'below_min_ratio'


def test_005_R21_ratio_exactly_at_the_minimum_counts_as_degraded_success():
    s, tracker = session([], target_ratio=0.95, min_acceptable_ratio=0.0)
    step = s.step(0.0)                                         # ratio 0.0 >= 0.0
    assert step.success and step.degraded


def test_005_R21_snapshot_reports_progress_for_telemetry():
    s, _ = session(wps(3), max_skip_ratio=0.9)
    s.start(0.0)
    s.step(0.0)
    snap = s.snapshot()
    assert snap['waypoints_total'] == 3 and snap['current_index'] == 0
    assert {'skipped_waypoints', 'covered_ratio', 'in_flight', 'finished'} <= set(snap)


def test_005_R32_suspend_keeps_the_remaining_waypoints_and_the_covered_mask():
    s, tracker = session(wps(4))
    s.start(0.0)
    s.step(0.0)
    s.goal_result(1.0, True)
    s.step(1.0)                                          # waypoint 1 in flight when the escape cancels
    tracker.mark_pose(1.0, 1.0)
    covered = tracker.covered_count
    s.suspend(2.0)
    assert s.current_index == 1 and not s.finished
    s.resume(5.0)
    step = s.step(5.0)
    assert (step.kind, step.index) == (StepKind.SEND, 1)  # first waypoint not yet reached
    assert tracker.covered_count == covered
    assert s.skipped_waypoints == 0                       # an interruption is not a failure


def test_005_R32_time_spent_suspended_does_not_count_against_coverage_timeout():
    s, _ = session(wps(2), coverage_timeout_s=100.0)
    s.start(0.0)
    s.step(0.0)
    s.suspend(60.0)
    s.resume(1000.0)
    assert s.step(1030.0).kind is not StepKind.DONE      # 60 + 30 s active
    assert s.step(1041.0).kind is StepKind.DONE          # 60 + 41 s > 100


def test_005_R32_resume_store_returns_the_session_for_the_same_room_within_the_window():
    store = ResumeStore(resume_window_s=120.0)
    s, _ = session(wps(2))
    store.save('room_01', s, t=10.0)
    assert store.take('room_01', t=100.0) is s
    assert store.take('room_01', t=101.0) is None         # consumed


def test_005_R32_resume_store_forgets_after_the_window():
    store = ResumeStore(resume_window_s=120.0)
    s, _ = session(wps(2))
    store.save('room_01', s, t=10.0)
    assert store.take('room_01', t=130.5) is None


def test_005_R32_resume_store_ignores_another_room_and_keeps_nothing_afterwards():
    store = ResumeStore(resume_window_s=120.0)
    s, _ = session(wps(2))
    store.save('room_01', s, t=0.0)
    assert store.take('room_02', t=1.0) is None
    assert store.take('room_01', t=2.0) is None           # the other-room goal discarded the old state


def test_005_R32_clear_drops_the_saved_state():
    store = ResumeStore(resume_window_s=120.0)
    s, _ = session(wps(2))
    store.save('', s, t=0.0)
    store.clear()
    assert store.take('', t=1.0) is None
