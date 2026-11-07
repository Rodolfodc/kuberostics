"""T006: SubmodeMachine tests, one row of the transition table per case.

Each test cites the requirement it verifies (Art. VIII). No ROS: the logic is pure Python.
"""
import pytest

from vacuum_supervisor.submode_machine import (
    COVERING, ESCAPE_FAILED, ESCAPING, NONE, Effect, SubmodeMachine,
)


def kinds(effects):
    return [e.kind for e in effects]


def machine_covering():
    m = SubmodeMachine()
    m.on_mode('CLEANING')
    return m


# --- 001-R05 ---------------------------------------------------------------
@pytest.mark.parametrize('mode', ['IDLE', 'CHARGING', 'CHARGED', 'UNKNOWN', ''])
def test_001_R05_mode_outside_cleaning_and_autodocking_gives_none(mode):
    m = SubmodeMachine()
    m.on_mode(mode)
    assert m.submode == NONE


# --- 001-R06 ---------------------------------------------------------------
def test_001_R06_cleaning_starts_covering_and_sends_cover_room():
    m = SubmodeMachine()
    fx = m.on_mode('CLEANING')
    assert m.submode == COVERING
    assert fx == [Effect('send_cover_room')]


def test_001_R06_republished_mode_is_idempotent():
    m = machine_covering()
    assert m.on_mode('CLEANING') == []
    assert m.submode == COVERING


def test_001_R05_leaving_cleaning_cancels_and_returns_to_none():
    m = machine_covering()
    fx = m.on_mode('IDLE')
    assert m.submode == NONE
    assert kinds(fx) == ['cancel_actions']


# --- 001-R07 ---------------------------------------------------------------
def test_001_R07_stuck_in_covering_cancels_and_escapes():
    m = machine_covering()
    fx = m.on_stuck()
    assert m.submode == ESCAPING
    assert kinds(fx) == ['cancel_actions', 'send_escape']


def test_001_R07_stuck_in_autodocking_escapes_and_resumes_none():
    m = SubmodeMachine()
    m.on_mode('AUTODOCKING')
    assert m.submode == NONE
    fx = m.on_stuck()
    assert m.submode == ESCAPING
    assert 'send_escape' in kinds(fx)
    m.on_escape_result(success=True)
    assert m.submode == NONE


@pytest.mark.parametrize('mode', ['IDLE', 'CHARGING', 'CHARGED'])
def test_001_R07_stuck_outside_covering_and_autodocking_is_ignored(mode):
    m = SubmodeMachine()
    m.on_mode(mode)
    assert m.on_stuck() == []
    assert m.submode == NONE


def test_001_R07_stuck_during_escaping_is_ignored():
    m = machine_covering()
    m.on_stuck()
    assert m.on_stuck() == []
    assert m.submode == ESCAPING


# --- 001-R08 ---------------------------------------------------------------
def test_001_R08_successful_escape_returns_to_covering_and_resends_cover_room():
    m = machine_covering()
    m.on_stuck()
    fx = m.on_escape_result(success=True)
    assert m.submode == COVERING
    assert fx == [Effect('send_cover_room')]


# --- 001-R09 ---------------------------------------------------------------
def test_001_R09_failed_escape_locks_and_notifies():
    m = machine_covering()
    m.on_stuck()
    fx = m.on_escape_result(success=False)
    assert m.submode == ESCAPE_FAILED
    assert Effect('lock_stop', True) in fx
    notes = [e for e in fx if e.kind == 'notify']
    assert len(notes) == 1
    assert notes[0].value == ('ERROR', 'ESCAPE_FAILED')


def test_001_R09_supervisor_escape_timeout_equals_failure():
    m = machine_covering()
    m.on_stuck()
    fx = m.on_escape_timeout()
    assert m.submode == ESCAPE_FAILED
    assert Effect('lock_stop', True) in fx


def test_001_R09_reset_releases_lock_and_returns_to_none():
    m = machine_covering()
    m.on_stuck()
    m.on_escape_result(success=False)
    fx = m.reset()
    assert m.submode == NONE
    assert fx == [Effect('lock_stop', False)]


def test_001_R09_reset_without_failure_does_nothing():
    m = machine_covering()
    assert m.reset() == []
    assert m.submode == COVERING


def test_001_R09_cleaning_does_not_restart_while_escape_failed():
    m = machine_covering()
    m.on_stuck()
    m.on_escape_result(success=False)
    assert m.on_mode('CLEANING') == []
    assert m.submode == ESCAPE_FAILED


# --- robustez (resultado atrasado depois de cancelar) ----------------------
def test_escape_result_outside_escaping_is_ignored():
    m = machine_covering()
    assert m.on_escape_result(success=False) == []
    assert m.submode == COVERING


def test_001_R05_leaving_cleaning_during_escaping_cancels_the_escape():
    m = machine_covering()
    m.on_stuck()
    fx = m.on_mode('IDLE')
    assert m.submode == NONE
    assert kinds(fx) == ['cancel_actions']
    assert m.on_escape_result(success=True) == []   # a late result revives nothing


# --- D-20: logs and telemetry (injected logger, metrics snapshot) ------------
class FakeLogger:
    def __init__(self):
        self.records = []

    def _add(self, level, msg):
        self.records.append((level, msg))

    def debug(self, msg):
        self._add('debug', msg)

    def info(self, msg):
        self._add('info', msg)

    def warning(self, msg):
        self._add('warning', msg)

    def error(self, msg):
        self._add('error', msg)

    def messages(self, level):
        return [m for lv, m in self.records if lv == level]


def test_D20_every_transition_is_logged_with_its_reason():
    log = FakeLogger()
    m = SubmodeMachine(logger=log)
    m.on_mode('CLEANING')
    assert any('NONE -> COVERING' in msg for msg in log.messages('info'))
    assert m.reason == 'mode CLEANING'


def test_D20_failed_escape_is_logged_as_error():
    log = FakeLogger()
    m = SubmodeMachine(logger=log)
    m.on_mode('CLEANING')
    m.on_stuck()
    m.on_escape_result(success=False)
    assert any('ESCAPING -> ESCAPE_FAILED' in msg for msg in log.messages('error'))


def test_D20_ignored_events_are_logged_at_debug():
    log = FakeLogger()
    m = SubmodeMachine(logger=log)
    m.on_stuck()
    assert log.messages('debug')


def test_D20_works_without_a_logger():
    m = SubmodeMachine()
    m.on_mode('CLEANING')
    m.on_stuck()
    assert m.submode == ESCAPING


def test_D20_metrics_count_events_and_outcomes():
    m = SubmodeMachine()
    m.on_mode('CLEANING')
    m.on_stuck()
    m.on_escape_result(success=True)
    m.on_stuck()
    m.on_escape_timeout()
    m.reset()
    snap = m.snapshot()
    assert snap['submode'] == NONE
    assert snap['stuck_events'] == 2
    assert snap['escapes_succeeded'] == 1
    assert snap['escapes_failed'] == 1
    assert snap['escape_timeouts'] == 1
    assert snap['resets'] == 1
    assert snap['transitions'] >= 5


# --- review round 1: B2 (lock must stay releasable) and B3 (mode changes) -----
def machine_escape_failed():
    m = machine_covering()
    m.on_stuck()
    m.on_escape_result(success=False)
    assert m.submode == ESCAPE_FAILED
    return m


def test_001_R22_reset_still_releases_the_lock_after_the_mode_left_cleaning():
    m = machine_escape_failed()
    m.on_mode('IDLE')
    assert m.submode == NONE
    assert m.snapshot()['locked'] is True          # the lock is independent of the submode
    assert m.reset() == [Effect('lock_stop', False)]
    assert m.snapshot()['locked'] is False


def test_001_R22_cleaning_while_locked_does_not_start_coverage_until_reset():
    m = machine_escape_failed()
    m.on_mode('IDLE')
    assert m.on_mode('CLEANING') == []
    assert m.submode == NONE
    m.reset()
    assert m.on_mode('CLEANING') == [Effect('send_cover_room')]


def test_001_R22_reset_without_a_lock_does_nothing():
    assert machine_covering().reset() == []


def test_001_R21_autodocking_while_covering_cancels_coverage():
    m = machine_covering()
    fx = m.on_mode('AUTODOCKING')
    assert m.submode == NONE
    assert kinds(fx) == ['cancel_actions']


def test_001_R21_mode_change_during_escaping_updates_where_to_resume():
    m = machine_covering()
    m.on_stuck()                                   # ESCAPING, resume COVERING
    assert m.on_mode('AUTODOCKING') == []          # the escape keeps running
    assert m.submode == ESCAPING
    assert m.on_escape_result(success=True) == []  # docking does not want coverage
    assert m.submode == NONE


def test_001_R21_autodocking_to_cleaning_during_escaping_resumes_coverage():
    m = SubmodeMachine()
    m.on_mode('AUTODOCKING')
    m.on_stuck()                                   # ESCAPING, resume NONE
    m.on_mode('CLEANING')
    assert kinds(m.on_escape_result(success=True)) == ['send_cover_room']
    assert m.submode == COVERING
