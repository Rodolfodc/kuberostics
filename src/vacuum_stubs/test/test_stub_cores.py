"""Pure-logic tests for the stub cores (no ROS). Requirements: 001-R06, 001-R07, 001-R16."""
import pytest

from vacuum_stubs.coverage_core import CoverageRamp
from vacuum_stubs.escape_core import EscapeScript
from vacuum_stubs.stuck_core import StuckStub


def script(outcome='succeed'):
    return EscapeScript(reverse_s=1.0, turn_s=1.0, reverse_linear_x=-0.1, turn_angular_z=0.8, outcome=outcome)


def test_001_R06_coverage_command_is_constant():
    r = CoverageRamp(linear_x=0.15, angular_z=0.2)
    assert r.command() == (0.15, 0.2)


def test_001_R06_coverage_ratio_grows_and_stays_below_one():
    r = CoverageRamp(linear_x=0.15, angular_z=0.0)
    assert r.ratio(0.0) == 0.0
    assert 0.0 < r.ratio(5.0) < r.ratio(50.0) <= 0.99
    assert r.ratio(1e6) <= 0.99          # a stub never claims the room is done


def test_001_R07_escape_reverses_then_turns_then_finishes():
    s = script()
    assert s.step(0.0).command == (-0.1, 0.0)
    assert s.step(0.99).command == (-0.1, 0.0)
    assert s.step(1.0).command == (0.0, 0.8)
    assert s.step(1.99).command == (0.0, 0.8)
    last = s.step(2.0)
    assert last.done and last.command is None


@pytest.mark.parametrize('outcome,expected', [('succeed', True), ('fail', False)])
def test_001_R16_escape_outcome_decides_success(outcome, expected):
    last = script(outcome).step(2.5)
    assert last.done and last.success is expected


def test_001_R16_escape_hang_never_finishes_and_publishes_zero():
    s = script('hang')
    for t in (2.0, 10.0, 1e5):
        st = s.step(t)
        assert not st.done and st.command == (0.0, 0.0)


def test_001_R16_escape_rejects_unknown_outcome():
    with pytest.raises(ValueError):
        script('explode')


def test_001_R07_escape_not_done_during_script():
    assert not script().step(1.5).done


def test_001_R07_stuck_stub_counts_triggers_and_uses_no_progress_cause():
    s = StuckStub()
    assert s.trigger() == 'NO_PROGRESS'
    s.trigger()
    assert s.snapshot()['events'] == 2
