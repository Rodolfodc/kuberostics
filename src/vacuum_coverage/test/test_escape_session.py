"""EscapeSession (spec 005-R12, R14): timeout, outcomes and odometry loss around EscapeScript. No ROS."""
import math

from vacuum_coverage.core.escape_script import Command, EscapeConfig
from vacuum_coverage.core.escape_session import EscapeSession, Final, OutcomeStub

DT = 0.05
A_MIN, A_INC = -math.pi, math.radians(1.0)
RANGES = [2.0] * 360


def session(outcome='auto', config=None, stub=None):
    return EscapeSession(config or EscapeConfig(), outcome, stub or OutcomeStub())


def run(s, front=3.0, pose_fn=None, max_s=30.0, t0=0.0):
    """Ticks until Final; the robot is perfectly still unless `pose_fn(t)` says otherwise."""
    t = t0
    cmds = []
    while t - t0 < max_s:
        out = s.step(t, pose_fn(t) if pose_fn else (0.0, 0.0, 0.0), front, None)
        if isinstance(out, Final):
            return out, cmds
        cmds.append(out)
        t += DT
    raise AssertionError('no final')


def test_005_R12_timeout_ends_a_robot_that_never_moves():
    s = session()
    s.start(0.0, RANGES, (0.0, 0.0, 0.0), A_MIN, A_INC, timeout_s=3.0)
    final, cmds = run(s, front=0.1)
    assert not final.success and final.reason == 'timeout'
    assert 2.9 <= len(cmds) * DT <= 3.2          # a frozen robot is bounded by the timeout, not by the script


def test_005_R12_zero_goal_timeout_uses_the_default():
    s = session()
    s.start(0.0, RANGES, (0.0, 0.0, 0.0), A_MIN, A_INC, timeout_s=0.0)
    assert s.timeout_s == EscapeConfig().default_timeout_s


def test_005_R12_negative_goal_timeout_uses_the_default():
    s = session()
    s.start(0.0, RANGES, (0.0, 0.0, 0.0), A_MIN, A_INC, timeout_s=-1.0)
    assert s.timeout_s == EscapeConfig().default_timeout_s


def test_005_R10_free_front_finishes_in_the_primary_phase():
    s = session()
    s.start(0.0, RANGES, (0.0, 0.0, 0.0), A_MIN, A_INC, timeout_s=0.0)
    state = {'x': 0.0, 'yaw': 0.0, 't': 0.0}

    def pose(t):
        return (state['x'], 0.0, state['yaw'])
    t = 0.0
    while True:
        out = s.step(t, pose(t), 3.0, None)
        if isinstance(out, Final):
            break
        state['x'] += out.linear * DT
        state['yaw'] += out.angular * DT
        t += DT
        assert t < 20
    assert out.success and out.phase == 'primary'
    assert s.strategy == 'scripted'


def test_005_R12_no_odometry_fails_at_once_without_motion():
    s = session()
    s.start(0.0, RANGES, None, A_MIN, A_INC, timeout_s=0.0)
    out = s.step(0.0, None, 3.0, None)
    assert isinstance(out, Final) and not out.success and out.reason == 'no odometry'


def test_005_R12_odometry_lost_mid_escape_fails():
    s = session()
    s.start(0.0, RANGES, (0.0, 0.0, 0.0), A_MIN, A_INC, timeout_s=0.0)
    assert isinstance(s.step(0.0, (0.0, 0.0, 0.0), 3.0, None), Command)
    out = s.step(0.1, None, 3.0, None)
    assert isinstance(out, Final) and not out.success and out.reason == 'odometry lost'


def test_005_R14_succeed_outcome_runs_the_stub_script_then_succeeds():
    s = session('succeed', stub=OutcomeStub(reverse_s=0.5, turn_s=0.5))
    s.start(0.0, RANGES, (0.0, 0.0, 0.0), A_MIN, A_INC, timeout_s=0.0)
    final, cmds = run(s)
    assert final.success
    assert cmds[0].linear < 0 and cmds[-1].angular != 0.0
    assert 0.9 <= len(cmds) * DT <= 1.1


def test_005_R14_fail_outcome_finishes_with_failure():
    s = session('fail', stub=OutcomeStub(reverse_s=0.1, turn_s=0.1))
    s.start(0.0, RANGES, (0.0, 0.0, 0.0), A_MIN, A_INC, timeout_s=0.0)
    final, _ = run(s)
    assert not final.success and final.reason == 'outcome fail'


def test_005_R14_hang_outcome_holds_zero_until_the_timeout():
    s = session('hang', stub=OutcomeStub(reverse_s=0.1, turn_s=0.1))
    s.start(0.0, RANGES, (0.0, 0.0, 0.0), A_MIN, A_INC, timeout_s=2.0)
    final, cmds = run(s)
    assert final.reason == 'timeout' and not final.success
    assert cmds[-1] == Command(0.0, 0.0)


def test_005_R14_unknown_outcome_is_rejected():
    import pytest
    with pytest.raises(ValueError):
        session('explode')


def test_005_R11_invalid_config_is_rejected():
    import pytest
    with pytest.raises(ValueError):
        session(config=EscapeConfig(reverse_speed_m_s=5.0))


def test_005_R12_step_after_final_returns_the_same_final():
    s = session()
    s.start(0.0, RANGES, None, A_MIN, A_INC, timeout_s=0.0)
    first = s.step(0.0, None, 3.0, None)
    assert s.step(0.1, (0.0, 0.0, 0.0), 3.0, None) == first


def test_005_R12_logs_start_and_final():
    logs = []
    s = EscapeSession(EscapeConfig(), 'auto', OutcomeStub(), logger=lambda lvl, m: logs.append((lvl, m)))
    s.start(0.0, RANGES, None, A_MIN, A_INC, timeout_s=0.0)
    s.step(0.0, None, 3.0, None)
    assert any('no odometry' in m for _, m in logs)
