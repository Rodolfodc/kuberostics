"""EscapeScript (spec 005): the scripted escape on a simulated clock and a simulated robot. No ROS."""
import math

import pytest

from vacuum_coverage.core.escape_script import Command, EscapeConfig, EscapeScript, Result

DT = 0.05
A_MIN, A_INC = -math.pi, math.radians(1.0)
FREE = 3.0


def ranges(left=2.0, right=1.0):
    return [left if (A_MIN + i * A_INC) > 0 else right for i in range(360)]


class Sim:
    """Unicycle integrating the commands the script returns; the script sees the pose at the start of each tick."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, frozen=False):
        self.x, self.y, self.yaw, self.t, self.frozen = x, y, yaw, 0.0, frozen

    @property
    def pose(self):
        return (self.x, self.y, self.yaw)

    def apply(self, cmd):
        if not self.frozen:
            self.x += cmd.linear * math.cos(self.yaw) * DT
            self.y += cmd.linear * math.sin(self.yaw) * DT
            self.yaw += cmd.angular * DT
        self.t += DT


def drive(script, sim, front_fn=lambda sim, script: FREE, rear_fn=lambda sim, script: None, max_s=40.0,
          trace=None):
    """Runs the script to its Result; returns (result, commands). `trace` collects (t, state, pose)."""
    cmds = []
    while sim.t < max_s:
        out = script.step(sim.t, sim.pose, front_fn(sim, script), rear_fn(sim, script))
        if trace is not None:
            trace.append((sim.t, script.state, sim.pose))
        if isinstance(out, Result):
            return out, cmds
        cmds.append(out)
        sim.apply(out)
    raise AssertionError('script did not finish')


def started(config=None, left=2.0, right=1.0, pose=(0.0, 0.0, 0.0), **kw):
    logs = []
    s = EscapeScript(config or EscapeConfig(), logger=lambda lvl, msg: logs.append((lvl, msg)), **kw)
    s.start(ranges(left, right), pose, A_MIN, A_INC)
    s.logs = logs
    return s


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def test_005_R08_first_command_is_a_reverse_at_reverse_speed():
    s = started()
    cmd = s.step(0.0, (0, 0, 0), FREE, None)
    assert cmd == Command(-0.15, 0.0)


def test_005_R08_reverse_stops_after_reverse_m_measured_on_the_odometry():
    s, sim, trace = started(), Sim(), []
    drive(s, sim, trace=trace)
    t_turn = next(t for t, st, _ in trace if st == 'TURN')
    x_turn = next(p for t, st, p in trace if st == 'TURN')[0]
    assert x_turn == pytest.approx(-0.20, abs=0.02)
    assert t_turn == pytest.approx(0.20 / 0.15, abs=0.15)


def test_005_R08_rear_clearance_ends_the_reverse_early():
    s, sim, trace = started(), Sim(), []
    drive(s, sim, rear_fn=lambda sim, sc: 0.10 if sim.x < -0.05 else 1.0, trace=trace)
    x_turn = next(p for t, st, p in trace if st == 'TURN')[0]
    assert -0.12 < x_turn < -0.04


def test_005_R08_rear_none_means_scan_does_not_cover_the_rear_and_the_check_is_skipped():
    s, sim, trace = started(), Sim(), []
    drive(s, sim, rear_fn=lambda sim, sc: None, trace=trace)
    assert next(p for t, st, p in trace if st == 'TURN')[0] == pytest.approx(-0.20, abs=0.02)


def test_005_R08_turns_toward_the_freer_side_left():
    s, sim = started(left=3.0, right=1.0), Sim()
    result, cmds = drive(s, sim)
    assert result.success
    assert any(c.angular > 0 for c in cmds) and not any(c.angular < 0 for c in cmds)


def test_005_R08_turns_toward_the_freer_side_right():
    s, sim = started(left=1.0, right=3.0), Sim()
    result, cmds = drive(s, sim)
    assert result.success
    assert any(c.angular < 0 for c in cmds) and not any(c.angular > 0 for c in cmds)


def test_005_R09_tie_goes_left():
    s, sim = started(left=2.0, right=2.0), Sim()
    _, cmds = drive(s, sim)
    assert any(c.angular > 0 for c in cmds) and not any(c.angular < 0 for c in cmds)


def test_005_R08_first_turn_is_turn_deg_base_degrees_at_turn_speed():
    s, sim, trace = started(), Sim(), []
    result, cmds = drive(s, sim, trace=trace)
    turn = [c for c in cmds if c.linear == 0.0 and c.angular != 0.0]
    assert all(c.angular == pytest.approx(1.0) for c in turn)
    assert math.degrees(sim.yaw) == pytest.approx(90.0, abs=4.0)
    assert result.success and result.phase == 'primary'


def test_005_R09_determinism_same_input_same_command_sequence():
    runs = []
    for _ in range(2):
        s, sim = started(left=1.5, right=1.2), Sim()
        front = lambda sim, sc: FREE if sim.yaw > 2.0 else 0.2        # noqa: E731  forces several attempts
        runs.append(drive(s, sim, front_fn=front))
    assert runs[0] == runs[1]


def test_005_R10_front_blocked_runs_the_next_attempt_with_a_growing_turn():
    cfg = EscapeConfig(primary_budget_s=30.0, fallback_budget_s=11.5, default_timeout_s=60.0)
    s, sim, trace = started(cfg), Sim(), []
    front = lambda sim, sc: FREE if sc.snapshot()['attempt'] >= 2 else 0.1     # noqa: E731
    result, _ = drive(s, sim, front_fn=front, trace=trace)
    assert result.success and result.phase == 'primary'
    # turn phases: yaw change between entering TURN and leaving it
    turns, start = [], None
    prev_state, prev_pose = None, None
    for _, st, pose in trace:
        if st == 'TURN' and prev_state != 'TURN':
            start = pose[2]
        if prev_state == 'TURN' and st != 'TURN':
            turns.append(math.degrees(prev_pose[2] - start))
        prev_state, prev_pose = st, pose
    assert turns == pytest.approx([90.0, 135.0, 180.0], abs=7.0)


def test_005_R10_success_when_front_range_equals_front_clearance():
    s, sim = started(), Sim()
    result, _ = drive(s, sim, front_fn=lambda sim, sc: 0.40)
    assert result.success and result.phase == 'primary'


def test_005_R10_attempts_end_at_max_attempts_and_the_fallback_runs():
    cfg = EscapeConfig(primary_budget_s=60.0, default_timeout_s=100.0, max_attempts=2)
    s, sim, trace = started(cfg), Sim(), []
    result, _ = drive(s, sim, front_fn=lambda sim, sc: 0.1, trace=trace)
    states = [st for _, st, _ in trace]
    assert 'FALLBACK_SWEEP' in states
    assert states.count('TURN') > 0 and snapshot_attempts(s) == 2
    assert not result.success and result.phase == 'fallback'


def snapshot_attempts(script):
    return script.snapshot()['attempt']


def test_005_R10_primary_budget_abandons_the_attempt_in_progress():
    s, sim, trace = started(), Sim(), []
    drive(s, sim, front_fn=lambda sim, sc: 0.1, trace=trace)
    t_fb = next(t for t, st, _ in trace if st == 'FALLBACK_SWEEP')
    assert t_fb == pytest.approx(5.0, abs=2 * DT)           # primary_budget_s counted from the first step


def test_005_R11_fallback_sweeps_360_in_the_turn_direction():
    s, sim, trace = started(), Sim(), []
    drive(s, sim, front_fn=lambda sim, sc: 0.1, trace=trace)
    sweep = [(t, p[2]) for t, st, p in trace if st == 'FALLBACK_SWEEP']
    assert sweep[-1][0] - sweep[0][0] == pytest.approx(2 * math.pi / 1.0, abs=0.2)
    assert sweep[-1][1] - sweep[0][1] == pytest.approx(2 * math.pi, abs=0.15)


def test_005_R11_fallback_faces_the_largest_range_and_advances():
    s, sim, trace = started(), Sim(), []
    state = {}

    def front(sim, sc):
        if sc.state == 'FALLBACK_SWEEP' and 'base' not in state:
            state['base'] = sim.yaw
        if 'base' not in state:
            return 0.1
        rel = wrap(sim.yaw - state['base'] - math.radians(60))        # open window around +60 deg
        return 2.5 if abs(rel) < math.radians(10) else 0.2
    result, _ = drive(s, sim, front_fn=front, trace=trace)
    assert result.success and result.phase == 'fallback'
    t_adv = next(i for i, (_, st, _) in enumerate(trace) if st == 'FALLBACK_ADVANCE')
    face_yaw = trace[t_adv][2][2]
    assert math.degrees(wrap(face_yaw - state['base'])) == pytest.approx(60.0, abs=8.0)
    x0, y0 = trace[t_adv][2][:2]
    assert math.hypot(sim.x - x0, sim.y - y0) == pytest.approx(0.30, abs=0.03)


def fallback_scenario(offset_deg, cfg=None, **sim_kw):
    """Front clear only at `offset_deg` from the heading at the start of the sweep."""
    s, sim, trace = started(cfg), Sim(**sim_kw), []
    state = {}

    def front(sim, sc):
        if sc.state == 'FALLBACK_SWEEP' and 'base' not in state:
            state['base'] = sim.yaw
            state['t0'] = sim.t
        if 'base' not in state:
            return 0.1
        rel = wrap(sim.yaw - state['base'] - math.radians(offset_deg))
        return 2.5 if abs(rel) < math.radians(8) else 0.2
    result, _ = drive(s, sim, front_fn=front, trace=trace)
    return result, state, sim, trace


def test_005_R11_b_fallback_with_a_180_degree_facing_turn_fits_fallback_budget():
    result, state, sim, _ = fallback_scenario(180.0)
    assert result.success and result.phase == 'fallback'
    assert sim.t - state['t0'] <= EscapeConfig().fallback_budget_s
    assert sim.t - state['t0'] > 10.5            # it really is the worst case (about 11.4 s)


def test_005_R11_a_worst_case_reaches_the_fallback_before_default_timeout():
    s, sim, trace = started(), Sim(), []
    result, _ = drive(s, sim, front_fn=lambda sim, sc: 0.1, trace=trace)
    t_fb = next(t for t, st, _ in trace if st == 'FALLBACK_SWEEP')
    assert t_fb < EscapeConfig().default_timeout_s
    assert not result.success
    assert sim.t <= EscapeConfig().primary_budget_s + EscapeConfig().fallback_budget_s + 2 * DT
    assert sim.t < EscapeConfig().default_timeout_s


def test_005_R11_fallback_fails_when_the_front_is_blocked_after_facing():
    s, sim = started(), Sim()
    result, _ = drive(s, sim, front_fn=lambda sim, sc: 0.1)
    assert not result.success and 'front' in result.reason


def test_005_R11_fallback_fails_when_the_advance_meets_an_obstacle():
    s, sim, trace = started(), Sim(), []
    state = {}

    def front(sim, sc):
        if sc.state == 'FALLBACK_SWEEP' and 'base' not in state:
            state['base'] = sim.yaw
        if sc.state == 'FALLBACK_ADVANCE':
            return 0.1
        if 'base' not in state:
            return 0.1
        return 2.5 if abs(wrap(sim.yaw - state['base'] - math.radians(30))) < math.radians(8) else 0.2
    result, _ = drive(s, sim, front_fn=front, trace=trace)
    assert not result.success and 'advance' in result.reason


def test_005_R11_fallback_budget_stops_a_robot_that_does_not_move():
    s, sim, trace = started(), Sim(frozen=True), []
    result, _ = drive(s, sim, front_fn=lambda sim, sc: 0.1, trace=trace)
    t_fb = next(t for t, st, _ in trace if st == 'FALLBACK_SWEEP')
    assert not result.success and 'budget' in result.reason
    assert sim.t - t_fb == pytest.approx(11.5, abs=2 * DT)


def test_005_R11_frozen_odometry_during_reverse_is_bounded_by_the_primary_budget():
    s, sim, trace = started(), Sim(frozen=True), []
    drive(s, sim, front_fn=lambda sim, sc: 0.1, trace=trace)
    assert next(t for t, st, _ in trace if st == 'FALLBACK_SWEEP') == pytest.approx(5.0, abs=2 * DT)


def test_005_R11_c_default_configuration_is_valid():
    assert EscapeConfig().validate() == []
    assert EscapeConfig().fallback_worst_case_s() == pytest.approx(6.2832 + 3.1416 + 2.0, abs=1e-3)


def test_005_R11_c_slow_turn_speed_is_rejected_because_the_worst_case_does_not_fit():
    problems = EscapeConfig(turn_speed_rad_s=0.8).validate()
    assert any('fallback_budget_s' in p for p in problems)


def test_005_R11_c_speeds_above_the_wheel_bridge_limits_are_rejected():
    assert any('reverse_speed_m_s' in p for p in EscapeConfig(reverse_speed_m_s=0.5).validate())
    cfg = EscapeConfig(turn_speed_rad_s=2.0, max_angular_speed_rad_s=1.0)
    assert any('turn_speed_rad_s' in p for p in cfg.validate())


def test_005_R11_c_budgets_must_fit_default_timeout():
    cfg = EscapeConfig(default_timeout_s=16.0)
    assert any('default_timeout_s' in p for p in cfg.validate())


def test_005_R11_c_nonsense_values_are_rejected():
    assert EscapeConfig(reverse_speed_m_s=0.0).validate()
    assert EscapeConfig(max_attempts=0).validate()
    assert EscapeConfig(front_clearance_m=-1.0).validate()


def test_005_R11_c_script_refuses_an_invalid_configuration():
    with pytest.raises(ValueError):
        EscapeScript(EscapeConfig(turn_speed_rad_s=0.5))


def test_005_R08_step_before_start_is_an_error():
    with pytest.raises(RuntimeError):
        EscapeScript(EscapeConfig()).step(0.0, (0, 0, 0), 1.0, None)


def test_005_R10_after_the_result_every_step_returns_it():
    s, sim = started(), Sim()
    result, _ = drive(s, sim)
    assert s.step(99.0, sim.pose, FREE, None) == result


def test_005_R08_no_front_reading_is_not_a_clear_front():
    s, sim = started(), Sim()
    result, _ = drive(s, sim, front_fn=lambda sim, sc: None)
    assert not result.success


def test_005_R08_start_can_be_called_again_for_a_new_goal():
    s, sim = started(), Sim()
    drive(s, sim)
    s.start(ranges(), (0, 0, 0), A_MIN, A_INC)
    assert s.state == 'REVERSE'
    assert s.step(0.0, (0, 0, 0), FREE, None) == Command(-0.15, 0.0)


def test_005_R08_logs_every_transition_and_snapshot_reports_state():
    logs_s = started()
    sim = Sim()
    drive(logs_s, sim)
    text = ' '.join(m for _, m in logs_s.logs)
    for word in ('REVERSE', 'TURN', 'DONE'):
        assert word in text
    snap = logs_s.snapshot()
    assert snap['state'] == 'DONE' and snap['success'] is True


def test_005_R08_metrics_callback_receives_the_outcome():
    got = []
    s = started(metrics=lambda n, v: got.append((n, v)))
    drive(s, Sim())
    assert ('escape_success', 1) in got
