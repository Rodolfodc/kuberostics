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


# --- review round 2, remark 1: no escape while the stop lock is on -----------------------------
def test_001_R22_stuck_event_is_ignored_while_locked():
    m = SubmodeMachine()
    m.on_mode('AUTODOCKING')
    m.on_stuck()
    m.on_escape_result(success=False)               # ESCAPE_FAILED, locked
    m.on_mode('IDLE')                               # submode back to NONE, the lock stays
    m.on_mode('AUTODOCKING')
    assert m.submode == NONE and m.snapshot()['locked'] is True
    assert m.on_stuck() == []
    assert m.submode == NONE


# --- spec 005, supervisor rows (005-R30): active only with dock_after_coverage -------------------------------
def machine_dock(mode='CLEANING'):
    m = SubmodeMachine(dock_after_coverage=True)
    m.on_mode(mode)
    return m


def test_005_R30_cover_result_success_notifies_done_requests_docking_and_goes_none():
    m = machine_dock()
    effects = m.on_cover_result(success=True)
    assert m.submode == NONE
    assert m.coverage_blocked is True
    assert Effect('notify', ('INFO', 'COVERAGE_DONE')) in effects
    assert kinds(effects).count('request_dock') == 1
    assert kinds(effects).count('start_dock_timer') == 1


def test_005_R30_cover_result_failure_notifies_warn_and_still_requests_docking():
    m = machine_dock()
    effects = m.on_cover_result(success=False)
    assert m.submode == NONE
    assert m.coverage_blocked is True
    assert Effect('notify', ('WARN', 'COVERAGE_FAILED')) in effects
    assert 'request_dock' in kinds(effects)
    assert 'start_dock_timer' in kinds(effects)


def test_005_R30_cover_result_does_not_lock_the_robot():
    m = machine_dock()
    effects = m.on_cover_result(success=False)
    assert m.locked is False
    assert 'lock_stop' not in kinds(effects)


def test_005_R30_no_new_cover_room_while_cleaning_stays_blocked():
    m = machine_dock()
    m.on_cover_result(success=True)
    assert m.on_mode('CLEANING') == []          # the orchestrator republishes the mode every 0.2 s
    assert m.on_mode('CLEANING') == []
    assert m.submode == NONE
    assert m.coverage_blocked is True


def test_005_R30_stuck_event_while_blocked_is_ignored():
    m = machine_dock()
    m.on_cover_result(success=True)
    assert m.on_stuck() == []
    assert m.submode == NONE


def test_005_R30_mode_autodocking_after_the_request_keeps_none_and_stops_the_timer():
    m = machine_dock()
    m.on_cover_result(success=True)
    effects = m.on_mode('AUTODOCKING')
    assert m.submode == NONE
    assert kinds(effects) == ['cancel_dock_timer']      # docking is driven by the IA368II autodocking_server (D-05)
    assert m.coverage_blocked is False                  # the mode left CLEANING


@pytest.mark.parametrize('mode', ['AUTODOCKING', 'CHARGING', 'CHARGED', 'IDLE', ''])
def test_005_R30_flag_is_cleared_when_the_mode_leaves_cleaning(mode):
    m = machine_dock()
    m.on_cover_result(success=True)
    m.on_mode(mode)
    assert m.coverage_blocked is False


def test_005_R30_cleaning_covers_again_after_the_flag_was_cleared():
    m = machine_dock()
    m.on_cover_result(success=True)
    m.on_mode('CHARGING')
    effects = m.on_mode('CLEANING')                     # 001-R06 again: a new cleaning session
    assert kinds(effects) == ['send_cover_room']
    assert m.submode == COVERING


def test_005_R30_dock_timer_expiry_in_cleaning_notifies_error_and_sends_no_goal():
    m = machine_dock()
    m.on_cover_result(success=True)
    effects = m.on_dock_timeout()
    assert effects == [Effect('notify', ('ERROR', 'DOCK_REQUEST_FAILED'))]
    assert m.submode == NONE
    assert m.coverage_blocked is True                   # bounded (Art. VI.3): still no new cover_room
    assert m.on_mode('CLEANING') == []


def test_005_R30_dock_timer_expiry_after_the_mode_changed_is_ignored():
    m = machine_dock()
    m.on_cover_result(success=True)
    m.on_mode('AUTODOCKING')
    assert m.on_dock_timeout() == []


def test_005_R30_dock_timeout_is_reported_once():
    m = machine_dock()
    m.on_cover_result(success=True)
    assert len(m.on_dock_timeout()) == 1
    assert m.on_dock_timeout() == []


def test_005_R30_cover_result_outside_covering_is_ignored():
    m = machine_dock()
    m.on_stuck()                                        # ESCAPING: the cover goal was cancelled, late result
    assert m.on_cover_result(success=True) == []
    assert m.submode == ESCAPING
    assert m.coverage_blocked is False


def test_005_R30_cover_result_after_the_mode_left_cleaning_is_ignored():
    m = machine_dock()
    m.on_mode('IDLE')
    assert m.on_cover_result(success=True) == []
    assert m.coverage_blocked is False


def test_005_R30_escape_success_still_resumes_covering_with_the_rows_enabled():
    m = machine_dock()
    m.on_stuck()
    assert kinds(m.on_escape_result(success=True)) == ['send_cover_room']
    assert m.submode == COVERING


def test_005_R30_with_the_flag_off_the_001_behaviour_is_unchanged():
    m = SubmodeMachine()                                # dock_after_coverage defaults to false
    m.on_mode('CLEANING')
    assert m.on_cover_result(success=True) == []
    assert m.submode == COVERING
    assert m.coverage_blocked is False


def test_005_R30_new_cleaning_session_clears_a_pending_dock_timer():
    m = machine_dock()
    m.on_cover_result(success=True)
    effects = m.on_mode('CHARGED')
    assert 'cancel_dock_timer' in kinds(effects)


def test_D20_005_R30_metrics_and_logs_of_the_new_rows():
    log = FakeLogger()
    m = SubmodeMachine(logger=log, dock_after_coverage=True)
    m.on_mode('CLEANING')
    m.on_cover_result(success=True)
    m.on_dock_timeout()
    snap = m.snapshot()
    assert snap['coverage_blocked'] is True
    assert snap['coverage_results'] == 1
    assert snap['dock_requests'] == 1
    assert snap['dock_request_failures'] == 1
    assert any('COVERING -> NONE' in msg for msg in log.messages('info'))
    assert log.messages('error')                        # the dock request failure
