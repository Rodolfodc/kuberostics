"""CoverageRun: one CoverRoom goal from region to result (005-R16, R17, R20, R21, R32). Pure Python, no ROS."""
import math

import pytest

from coverage_fixtures import rect_room
from vacuum_coverage.core.coverage_run import ConfigError, CoverageConfig, CoverageRun
from vacuum_coverage.core.coverage_session import StepKind
from vacuum_coverage.core.region_builder import UnknownRoom


def drive(run, t=0.0, limit=2000):
    """Reports every waypoint as reached, feeding poses along the straight hop (free room, 0.2 m apart)."""
    last = None
    for _ in range(limit):
        step = run.step(t)
        if step.kind is StepKind.DONE:
            return step
        if step.kind is StepKind.SEND:
            goal = (step.waypoint.x, step.waypoint.y)
            start = last or goal
            n = max(1, int(math.hypot(goal[0] - start[0], goal[1] - start[1]) / 0.2))
            for i in range(n + 1):
                run.on_pose(start[0] + (goal[0] - start[0]) * i / n, start[1] + (goal[1] - start[1]) * i / n)
            last = goal
            run.goal_result(t, True)
        t += 0.1
    raise AssertionError('no result')


def test_R21_config_rejects_lane_spacing_wider_than_tool_diameter():
    with pytest.raises(ConfigError):
        CoverageConfig(lane_spacing_m=0.4, tool_radius_m=0.15).validate()


def test_R21_config_rejects_min_ratio_above_target():
    with pytest.raises(ConfigError):
        CoverageConfig(min_acceptable_ratio=0.9, target_ratio=0.8).validate()


def test_R21_defaults_are_valid():
    CoverageConfig().validate()


def test_R16_empty_room_id_without_room_topics_uses_connected_cells():
    run = CoverageRun(CoverageConfig())
    run.begin(0.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1)
    assert run.session.waypoints
    assert run.covered_grid().max() == 0


def test_R16_non_empty_room_id_without_room_topics_is_rejected():
    run = CoverageRun(CoverageConfig())
    with pytest.raises(UnknownRoom):
        run.begin(0.0, rect_room(2.0, 2.0), (1.0, 1.0), 'room_01', 1)


def test_R21_success_when_ratio_reaches_target():
    raster = rect_room(2.0, 2.0)
    run = CoverageRun(CoverageConfig())
    run.begin(0.0, raster, (1.0, 1.0), '', 1)
    res = drive(run)
    assert res.success and not res.degraded and res.covered_ratio >= 0.95


def test_R21_degraded_success_when_waypoints_run_out_between_min_and_target():
    run = CoverageRun(CoverageConfig(target_ratio=0.999, min_acceptable_ratio=0.01))
    run.begin(0.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1)
    step = None
    # reach only the waypoints, without marking the tool between them: ratio stays well below 1.0
    for _ in range(500):
        step = run.step(0.0)
        if step.kind is StepKind.DONE:
            break
        if step.kind is StepKind.SEND:
            run.on_pose(step.waypoint.x, step.waypoint.y)
            run.goal_result(0.0, True)
    assert step.kind is StepKind.DONE
    assert step.success is True and step.degraded is True
    assert 0.01 <= step.covered_ratio < 0.999


def test_R20_failure_below_min_ratio():
    run = CoverageRun(CoverageConfig(min_acceptable_ratio=0.7, max_skip_ratio=1.0))
    run.begin(0.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1)
    for _ in range(500):
        step = run.step(0.0)
        if step.kind is StepKind.DONE:
            break
        if step.kind is StepKind.SEND:
            run.goal_result(0.0, False)
    assert step.kind is StepKind.DONE and step.success is False


def test_R32_same_room_within_window_resumes_session_and_mask():
    run = CoverageRun(CoverageConfig(resume_window_s=120.0))
    run.begin(0.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1)
    first = run.step(0.0)
    run.on_pose(first.waypoint.x, first.waypoint.y)
    run.goal_result(0.0, True)
    run.step(1.0)                      # second waypoint in flight
    ratio = run.covered_ratio
    index = run.session.current_index
    run.suspend(2.0)
    assert run.begin(30.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1) is True
    assert run.session.current_index == index
    assert run.covered_ratio == pytest.approx(ratio)


def test_R32_resume_window_expiry_rebuilds():
    run = CoverageRun(CoverageConfig(resume_window_s=10.0))
    run.begin(0.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1)
    run.step(0.0)
    run.suspend(1.0)
    assert run.begin(100.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1) is False
    assert run.session.current_index == 0


def test_R32_state_cleared_after_goal_ends():
    run = CoverageRun(CoverageConfig())
    run.begin(0.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1)
    run.step(0.0)
    run.suspend(1.0)
    run.begin(2.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1)
    run.end()
    assert run.begin(3.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1) is False


def test_R21_pose_hops_longer_than_limit_are_not_interpolated():
    run = CoverageRun(CoverageConfig(), max_pose_gap_m=0.5)
    run.begin(0.0, rect_room(4.0, 1.0), (0.5, 0.5), '', 1)
    run.on_pose(0.3, 0.5)
    before = run.session.tracker.covered_count
    run.on_pose(3.5, 0.5)          # a jump (relocalisation): credits only the end pose
    gained = run.session.tracker.covered_count - before
    assert 0 < gained < 0.5 * run.session.tracker.coverable_count


def test_R21_pose_marking_interpolates_short_hops():
    run = CoverageRun(CoverageConfig(), max_pose_gap_m=1.0)
    run.begin(0.0, rect_room(4.0, 1.0), (0.5, 0.5), '', 1)
    run.on_pose(0.3, 0.5)
    run.on_pose(0.9, 0.5)
    assert run.session.tracker.covered_count > 0


def test_R32_rejected_goal_does_not_consume_the_saved_session():
    run = CoverageRun(CoverageConfig(resume_window_s=120.0))
    run.begin(0.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1)
    run.step(0.0)
    run.suspend(1.0)
    with pytest.raises(UnknownRoom):
        run.begin(2.0, rect_room(2.0, 2.0), (1.0, 1.0), 'room_99', 1)
    assert run.begin(3.0, rect_room(2.0, 2.0), (1.0, 1.0), '', 1) is True
