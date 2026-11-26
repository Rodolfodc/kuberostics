"""ExplorationSession (pure core): 003-R10 (retry once, then blacklist), R11 (success, timeouts, blacklist ratio),
R32 (blacklist, counters and clock survive a cancel and a re-sent goal)."""
import numpy as np
import pytest

from vacuum_grid.grid_view import FREE, OCCUPIED, UNKNOWN, GridView
from vacuum_mapping.core.exploration_session import (ExplorationConfig, ExplorationSession, StepKind)

RES = 0.05


def room_with_unknown_strip(cols=60):
    """A free 30x`cols` room whose right edge touches unknown space (one frontier cluster)."""
    cells = np.full((30, cols), FREE, dtype=np.int8)
    cells[0, :] = OCCUPIED
    cells[-1, :] = OCCUPIED
    cells[:, 0] = OCCUPIED
    cells[1:-1, -3:] = UNKNOWN
    return GridView(cells, RES)


def two_frontiers():
    """Frontiers at the left and at the right edge, 2 m apart."""
    cells = np.full((30, 60), FREE, dtype=np.int8)
    cells[0, :] = OCCUPIED
    cells[-1, :] = OCCUPIED
    cells[1:-1, :3] = UNKNOWN
    cells[1:-1, -3:] = UNKNOWN
    return GridView(cells, RES)


def known_room():
    cells = np.full((30, 60), FREE, dtype=np.int8)
    cells[0, :] = OCCUPIED
    cells[-1, :] = OCCUPIED
    cells[:, 0] = OCCUPIED
    cells[:, -1] = OCCUPIED
    return GridView(cells, RES)


def make(**kw):
    log = []
    cfg = ExplorationConfig(**kw)
    return ExplorationSession(cfg, log=log.append), log


def test_R11_success_when_no_frontier_remains():
    s, _ = make()
    s.begin(0.0, 0.0)
    step = s.step(0.1, known_room(), (1.0, 1.0))
    assert step.kind is StepKind.DONE and step.success and step.reason == 'no_frontier'


def test_R10_first_step_sends_the_best_frontier():
    s, _ = make()
    s.begin(0.0, 0.0)
    step = s.step(0.1, room_with_unknown_strip(), (0.5, 0.7))
    assert step.kind is StepKind.SEND
    assert step.frontier.centroid[0] > 2.0


def test_R10_goal_in_flight_waits_until_the_outcome():
    s, _ = make()
    s.begin(0.0, 0.0)
    grid = room_with_unknown_strip()
    assert s.step(0.1, grid, (0.5, 0.7)).kind is StepKind.SEND
    assert s.step(0.2, grid, (0.5, 0.7)).kind is StepKind.WAIT


def test_R10_abort_retries_once_then_blacklists():
    s, _ = make()
    s.begin(0.0, 0.0)
    grid = two_frontiers()
    first = s.step(0.1, grid, (1.0, 0.7)).frontier
    s.goal_result(0.2, False)
    retry = s.step(0.3, grid, (1.0, 0.7))
    assert retry.kind is StepKind.SEND and retry.frontier.centroid == first.centroid, 'one retry of the same frontier'
    s.goal_result(0.4, False)
    nxt = s.step(0.5, grid, (1.0, 0.7))
    assert nxt.kind is StepKind.SEND and nxt.frontier.centroid != first.centroid, 'blacklisted after the retry'
    assert s.snapshot()['blacklisted'] == 1


def test_R10_frontier_timeout_cancels_then_retries_then_blacklists():
    s, _ = make(frontier_timeout_s=10.0)
    s.begin(0.0, 0.0)
    grid = two_frontiers()
    first = s.step(0.0, grid, (1.0, 0.7)).frontier
    assert s.step(9.9, grid, (1.0, 0.7)).kind is StepKind.WAIT
    assert s.step(10.0, grid, (1.0, 0.7)).kind is StepKind.CANCEL      # R31: the goal is bounded and cancelled
    retry = s.step(10.1, grid, (1.0, 0.7))
    assert retry.kind is StepKind.SEND and retry.frontier.centroid == first.centroid
    assert s.step(20.1, grid, (1.0, 0.7)).kind is StepKind.CANCEL
    nxt = s.step(20.2, grid, (1.0, 0.7))
    assert nxt.frontier.centroid != first.centroid
    assert s.snapshot()['blacklisted'] == 1


def test_R10_reached_frontier_that_stays_is_blacklisted_after_max_reaches():
    s, _ = make(max_reaches=2, max_blacklist_ratio=1.0)
    s.begin(0.0, 0.0)
    grid = room_with_unknown_strip()
    for i in range(2):
        assert s.step(i, grid, (1.0, 0.7)).kind is StepKind.SEND
        s.goal_result(i + 0.5, True)
    step = s.step(5.0, grid, (1.0, 0.7))
    assert step.kind is StepKind.DONE and step.success, 'the only frontier is blacklisted, none remains'
    assert step.blacklisted == 1 and step.done == 2


def test_R11_timeout_fails_with_partial_result_and_cancels_the_goal():
    s, _ = make(explore_timeout_s=100.0)
    s.begin(0.0, 0.0)
    grid = room_with_unknown_strip()
    s.step(1.0, grid, (1.0, 0.7))
    step = s.step(100.0, grid, (1.0, 0.7))
    assert step.kind is StepKind.DONE and not step.success and step.reason == 'timeout' and step.cancel_goal


def test_R11_goal_timeout_zero_means_the_parameter_and_a_positive_one_overrides_it():
    s, _ = make(explore_timeout_s=100.0)
    s.begin(0.0, 0.0)
    assert s.deadline == 100.0
    s.end()
    s.begin(0.0, 30.0)
    assert s.deadline == 30.0


def test_R11_blacklist_ratio_fails_and_reports_incomplete():
    s, log = make(max_blacklist_ratio=0.4)
    s.begin(0.0, 0.0)
    grid = two_frontiers()
    s.step(0.0, grid, (1.0, 0.7))
    s.goal_result(0.1, False)
    s.step(0.2, grid, (1.0, 0.7))
    s.goal_result(0.3, False)                       # 1 blacklisted of 2 seen = 0.5 > 0.4
    step = s.step(0.4, grid, (1.0, 0.7))
    assert step.kind is StepKind.DONE and not step.success and step.reason == 'blacklist_ratio'
    assert any('blacklist' in line for line in log)


def test_R11_ratio_at_the_limit_continues():
    s, _ = make(max_blacklist_ratio=0.5)
    s.begin(0.0, 0.0)
    grid = two_frontiers()
    s.step(0.0, grid, (1.0, 0.7))
    s.goal_result(0.1, False)
    s.step(0.2, grid, (1.0, 0.7))
    s.goal_result(0.3, False)                       # 1 of 2 = 0.5, not more than 0.5
    assert s.step(0.4, grid, (1.0, 0.7)).kind is StepKind.SEND


def test_R32_suspend_and_resume_keep_blacklist_counters_and_clock():
    s, _ = make(explore_timeout_s=100.0)
    s.begin(0.0, 0.0)
    grid = two_frontiers()
    first = s.step(0.0, grid, (1.0, 0.7)).frontier
    s.goal_result(1.0, False)
    s.step(1.5, grid, (1.0, 0.7))
    s.goal_result(2.0, False)                       # first frontier blacklisted
    s.step(2.5, grid, (1.0, 0.7))
    s.suspend(30.0)                                  # supervisor cancels explore for an escape
    assert s.begin(60.0, 500.0) is True              # re-sent: resumed, goal timeout ignored
    assert s.deadline == 100.0, 'the clock is not restarted'
    assert s.snapshot()['blacklisted'] == 1 and s.snapshot()['seen'] == 2
    nxt = s.step(61.0, grid, (1.0, 0.7))
    assert nxt.kind is StepKind.SEND and nxt.frontier.centroid != first.centroid
    assert s.step(100.0, grid, (1.0, 0.7)).reason == 'timeout'


def test_R32_end_resets_everything():
    s, _ = make()
    s.begin(0.0, 0.0)
    grid = two_frontiers()
    s.step(0.0, grid, (1.0, 0.7))
    s.goal_result(0.1, False)
    s.step(0.2, grid, (1.0, 0.7))
    s.goal_result(0.3, False)
    s.end()
    assert s.begin(1000.0, 0.0) is False
    assert s.snapshot()['blacklisted'] == 0 and s.deadline == 1000.0 + 900.0


def test_R32_done_clears_the_saved_session():
    s, _ = make()
    s.begin(0.0, 0.0)
    s.step(0.1, known_room(), (1.0, 1.0))
    s.suspend(0.2)                                   # a DONE session is not resumable
    assert s.begin(1.0, 0.0) is False


def test_outcome_of_a_cancelled_goal_is_ignored_after_suspend():
    s, _ = make()
    s.begin(0.0, 0.0)
    grid = two_frontiers()
    s.step(0.0, grid, (1.0, 0.7))
    s.suspend(0.5)
    s.begin(1.0, 0.0)
    s.goal_result(1.1, False)                        # late result of the cancelled goal: not an attempt
    assert s.snapshot()['blacklisted'] == 0


def test_config_validate_rejects_bad_values():
    for bad in ({'frontier_timeout_s': 0.0}, {'explore_timeout_s': -1.0}, {'max_blacklist_ratio': 1.5},
                {'max_reaches': 0}):
        with pytest.raises(ValueError):
            ExplorationConfig(**bad).validate()
