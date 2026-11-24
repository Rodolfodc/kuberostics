"""StuckCore (spec 005, T504): sliding-window stuck detection on a simulated clock. No ROS."""
import math

import pytest

from vacuum_coverage.core.stuck_core import StuckCore

DT = 0.1


def make(**kw):
    logs = []
    metrics = []
    core = StuckCore(logger=lambda lvl, msg: logs.append((lvl, msg)),
                     metrics=lambda name, value: metrics.append((name, value)), **kw)
    return core, logs, metrics


def run(core, seconds, cmd=(0.2, 0.0), pose=(0.0, 0.0, 0.0), t0=0.0, pose_fn=None, **kw):
    """Feeds the core every DT s for `seconds`; returns (list of (t, cause), last t)."""
    out = []
    n = int(round(seconds / DT))
    t = t0
    for i in range(n + 1):
        t = t0 + i * DT
        p = pose_fn(t - t0) if pose_fn else pose
        cause = core.update(t, cmd[0], cmd[1], p, **kw)
        if cause:
            out.append((round(t, 6), cause))
    return out, t


def test_005_R01_forward_command_without_displacement_emits_no_progress_after_window():
    core, _, _ = make()
    events, _ = run(core, 5.0)
    assert events and events[0][1] == 'NO_PROGRESS'
    assert events[0][0] == pytest.approx(3.0, abs=DT)       # window_s default 3.0


def test_005_R01_no_event_before_the_window_is_complete():
    core, _, _ = make()
    events, _ = run(core, 2.8)
    assert events == []


def test_005_R01_defaults_match_the_spec():
    core, _, _ = make()
    assert (core.cmd_linear_min_m_s, core.cmd_angular_min_rad_s) == (0.05, 0.20)
    assert (core.window_s, core.min_translation_m, core.min_rotation_rad) == (3.0, 0.03, 0.10)
    assert core.cooldown_s == 5.0


def test_005_R01_translation_above_threshold_is_not_stuck():
    core, _, _ = make()
    events, _ = run(core, 10.0, pose_fn=lambda t: (0.1 * t, 0.0, 0.0))
    assert events == []


def test_005_R01_slow_creep_below_min_translation_is_stuck():
    core, _, _ = make()
    # 0.005 m/s -> 0.015 m over 3 s, below 0.03 m
    events, _ = run(core, 5.0, pose_fn=lambda t: (0.005 * t, 0.0, 0.0))
    assert events


def test_005_R01_pure_rotation_with_yaw_change_is_not_stuck():
    core, _, _ = make()
    events, _ = run(core, 10.0, cmd=(0.0, 1.0), pose_fn=lambda t: (0.0, 0.0, 1.0 * t))
    assert events == []


def test_005_R01_rotation_command_without_yaw_change_is_stuck():
    core, _, _ = make()
    events, _ = run(core, 5.0, cmd=(0.0, 1.0))
    assert events and events[0][1] == 'NO_PROGRESS'


def test_005_R01_yaw_wrap_around_pi_counts_as_motion():
    core, _, _ = make()

    def yaw(t):
        a = 3.0 + 0.5 * t
        return (0.0, 0.0, math.atan2(math.sin(a), math.cos(a)))   # wraps at pi
    events, _ = run(core, 10.0, cmd=(0.0, 0.5), pose_fn=lambda t: yaw(t))
    assert events == []


def test_005_R01_command_exactly_at_threshold_is_active():
    core, _, _ = make()
    events, _ = run(core, 5.0, cmd=(0.05, 0.0))
    assert events


def test_005_R01_command_below_both_thresholds_is_inactive():
    core, _, _ = make()
    events, _ = run(core, 10.0, cmd=(0.049, 0.19))
    assert events == []


def test_005_R01_negative_commands_count_by_absolute_value():
    core, _, _ = make()
    events, _ = run(core, 5.0, cmd=(-0.2, 0.0))
    assert events


def test_005_R01_motion_in_the_second_half_slides_the_window():
    core, _, _ = make()
    # still for 2 s, then moving at 0.1 m/s: the window never holds 3 s of nothing
    events, _ = run(core, 10.0, pose_fn=lambda t: (0.0 if t < 2.0 else 0.1 * (t - 2.0), 0.0, 0.0))
    assert events == []


def test_005_R01_robot_stops_moving_mid_run_is_detected_after_window_from_the_stop():
    core, _, _ = make()
    events, _ = run(core, 10.0, pose_fn=lambda t: (0.1 * min(t, 2.0), 0.0, 0.0))
    assert events
    assert events[0][0] == pytest.approx(5.0, abs=0.35)     # stopped at 2 s, plus the 3 s window


def test_005_R01_configurable_thresholds():
    core, _, _ = make(window_s=1.0, cmd_linear_min_m_s=0.2)
    assert run(core, 3.0, cmd=(0.1, 0.0))[0] == []
    core, _, _ = make(window_s=1.0, cmd_linear_min_m_s=0.2)
    events, _ = run(core, 3.0, cmd=(0.25, 0.0))
    assert events and events[0][0] == pytest.approx(1.0, abs=DT)


def test_005_R02_zero_command_never_emits():
    core, _, _ = make()
    events, _ = run(core, 20.0, cmd=(0.0, 0.0))
    assert events == []


def test_005_R02_command_drop_resets_the_window():
    core, _, _ = make()
    events, t = run(core, 2.5)
    assert events == []
    events, t = run(core, 1.0, cmd=(0.0, 0.0), t0=t + DT)          # zero Twist of the stop lock (D-22)
    assert events == []
    events, _ = run(core, 2.5, t0=t + DT)                           # 2.5 s of command: window restarted
    assert events == []
    events, _ = run(core, 1.5, t0=t + DT)
    assert events


def test_005_R02_stale_command_is_treated_as_zero():
    core, _, _ = make()
    # the mux goes silent on input timeout instead of sending zero: the node reports the age of the last command
    events, _ = run(core, 20.0, cmd_age_s=2.0)
    assert events == []


def test_005_R02_fresh_command_age_is_still_active():
    core, _, _ = make()
    events, _ = run(core, 5.0, cmd_age_s=0.1)
    assert events


def test_005_R01_missing_pose_resets_the_window_and_never_emits():
    core, _, _ = make()
    events, _ = run(core, 20.0, pose=None)
    assert events == []


def test_005_R01_sample_gap_resets_the_window():
    core, _, _ = make()
    run(core, 2.0)
    # odometry silent for 2 s while the command stays active, then still: not 3 s of evidence
    assert core.update(4.1, 0.2, 0.0, (0.0, 0.0, 0.0)) is None
    events, _ = run(core, 2.5, t0=4.2)
    assert events == []


def test_005_R01_time_going_backwards_resets():
    core, _, _ = make()
    run(core, 2.5)
    assert core.update(0.0, 0.2, 0.0, (0, 0, 0)) is None
    events, _ = run(core, 2.5, t0=0.1)
    assert events == []


def test_005_R03_cooldown_suppresses_further_events_and_restarts_the_window():
    core, _, _ = make()
    events, _ = run(core, 15.0)
    times = [t for t, _ in events]
    assert len(times) >= 2
    # first at ~3 s; cooldown 5 s ends at ~8 s; the window restarts there, so the next is ~3 s later
    assert times[1] - times[0] == pytest.approx(5.0 + 3.0, abs=2 * DT)


def test_005_R03_no_event_inside_the_cooldown():
    core, _, _ = make()
    events, _ = run(core, 8.0)
    assert len(events) == 1


def test_005_R03_cooldown_configurable():
    core, _, _ = make(cooldown_s=1.0)
    events, _ = run(core, 12.0)
    assert events[1][0] - events[0][0] == pytest.approx(1.0 + 3.0, abs=2 * DT)


def test_005_R05_bumper_press_emits_bumper():
    core, _, _ = make()
    assert core.bumper_pressed(1.0) == 'BUMPER'


def test_005_R05_bumper_respects_cooldown():
    core, _, _ = make()
    assert core.bumper_pressed(1.0) == 'BUMPER'
    assert core.bumper_pressed(2.0) is None
    assert core.bumper_pressed(6.1) == 'BUMPER'


def test_005_R05_bumper_event_starts_the_cooldown_for_no_progress():
    core, _, _ = make()
    core.bumper_pressed(0.0)
    events, _ = run(core, 4.9, t0=0.1)
    assert events == []


def test_005_R01_logs_every_transition_and_counts_metrics():
    core, logs, metrics = make()
    run(core, 4.0)
    text = ' '.join(m for _, m in logs)
    assert 'monitoring' in text and 'stuck' in text
    assert ('stuck_events_total', 1) in metrics
    assert core.snapshot()['events'] == 1


def test_005_R02_logs_the_window_reset_on_command_drop():
    core, logs, _ = make()
    run(core, 1.0)
    core.update(1.2, 0.0, 0.0, (0, 0, 0))
    assert any('reset' in m for _, m in logs)


def test_005_R01_snapshot_reports_state():
    core, _, _ = make()
    assert core.snapshot()['state'] == 'idle'
    run(core, 1.0)
    assert core.snapshot()['state'] == 'monitoring'
    run(core, 4.0, t0=1.1)
    assert core.snapshot()['state'] == 'cooldown'


def test_005_R01_rejects_nonsense_configuration():
    with pytest.raises(ValueError):
        StuckCore(window_s=0.0)
    with pytest.raises(ValueError):
        StuckCore(cooldown_s=-1.0)
    with pytest.raises(ValueError):
        StuckCore(min_translation_m=-0.1)
