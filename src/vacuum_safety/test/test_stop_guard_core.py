"""T017: StopGuard tests (spec 001, Q-08 / D-22). Pure Python, no ROS.

The guard turns the stop lock into an explicit zero-velocity command, because twist_mux publishes
nothing while locked (001-R10, 001-R19).
"""
from vacuum_safety.stop_guard_core import StopGuard


class FakeLogger:
    def __init__(self):
        self.records = []

    def __getattr__(self, level):
        return lambda msg: self.records.append((level, msg))


def test_001_R19_no_command_while_unlocked():
    assert StopGuard().command() is None


def test_001_R19_zero_velocity_while_locked():
    g = StopGuard()
    g.on_lock(True)
    assert g.command() == (0.0, 0.0)


def test_001_R19_command_stops_when_the_lock_is_released():
    g = StopGuard()
    g.on_lock(True)
    g.on_lock(False)
    assert g.command() is None


def test_001_R19_repeated_lock_messages_are_idempotent():
    g = StopGuard()
    g.on_lock(True)
    g.on_lock(True)
    assert g.snapshot()['lock_changes'] == 1


def test_001_R19_lock_is_latched_when_the_publisher_goes_silent():
    g = StopGuard()
    g.on_lock(True)
    # no further messages: a dead supervisor must never release the robot
    assert g.command() == (0.0, 0.0)


def test_D20_lock_changes_are_logged_and_counted():
    log = FakeLogger()
    g = StopGuard(logger=log)
    g.on_lock(True)
    g.command()
    g.command()
    g.on_lock(False)
    snap = g.snapshot()
    assert snap == {'locked': False, 'lock_changes': 2, 'zero_commands': 2}
    assert [lv for lv, _ in log.records] == ['warning', 'info']
