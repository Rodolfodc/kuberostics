"""EXPLORING rows of the SubmodeMachine (spec 003, plan "Supervisor: new rows"; 003-R14, 003-R32).

Each test cites the requirement it verifies (Art. VIII). No ROS: the logic is pure Python.
"""
from vacuum_supervisor.submode_machine import (
    COVERING, ESCAPE_FAILED, ESCAPING, EXPLORING, NONE, Effect, SubmodeMachine,
)


def kinds(effects):
    return [e.kind for e in effects]


def exploring(**kw):
    m = SubmodeMachine(explore_on_start=True, **kw)
    m.on_mode('CLEANING')
    return m


def test_003_R14_default_off_behaves_as_spec_001():
    m = SubmodeMachine()
    fx = m.on_mode('CLEANING')
    assert m.submode == COVERING and fx == [Effect('send_cover_room')]


def test_003_R14_cleaning_with_explore_on_start_sends_explore():
    m = SubmodeMachine(explore_on_start=True)
    fx = m.on_mode('CLEANING')
    assert m.submode == EXPLORING
    assert fx == [Effect('send_explore')]


def test_003_R14_explored_true_skips_exploration():
    m = SubmodeMachine(explore_on_start=True)
    m.on_explored(True)
    fx = m.on_mode('CLEANING')
    assert m.submode == COVERING and fx == [Effect('send_cover_room')]


def test_003_R14_republished_mode_is_idempotent_in_exploring():
    m = exploring()
    assert m.on_mode('CLEANING') == []
    assert m.submode == EXPLORING


def test_003_R14_explore_success_goes_to_covering():
    m = exploring()
    fx = m.on_explore_result(True)
    assert m.submode == COVERING and m.exploration_done
    assert fx == [Effect('send_cover_room')]


def test_003_R14_explore_failure_goes_to_covering_with_partial_map():
    m = exploring()
    fx = m.on_explore_result(False)
    assert m.submode == COVERING and m.exploration_done
    assert fx == [Effect('send_cover_room')]


def test_003_R14_exploration_done_prevents_a_loop_after_failure():
    m = exploring()
    m.on_explore_result(False)
    m.on_mode('IDLE')
    fx = m.on_mode('CLEANING')
    assert m.submode == COVERING and fx == [Effect('send_cover_room')]


def test_003_R14_late_explore_result_is_ignored():
    m = exploring()
    m.on_mode('IDLE')
    assert m.on_explore_result(True) == []
    assert m.submode == NONE and not m.exploration_done


def test_003_R14_explore_result_outside_exploring_is_ignored():
    m = SubmodeMachine(explore_on_start=True)
    assert m.on_explore_result(True) == []
    assert m.submode == NONE


def test_003_R14_stuck_in_exploring_cancels_and_escapes():
    m = exploring()
    fx = m.on_stuck()
    assert m.submode == ESCAPING and m.resume_submode == EXPLORING
    assert kinds(fx) == ['cancel_actions', 'send_escape']
    assert m.snapshot()['stuck_events'] == 1


def test_003_R14_escape_success_returns_to_exploring():
    m = exploring()
    m.on_stuck()
    fx = m.on_escape_result(True)
    assert m.submode == EXPLORING
    assert fx == [Effect('send_explore')]


def test_003_R14_exploration_done_stays_false_across_an_escape():
    m = exploring()
    m.on_stuck()
    m.on_escape_result(True)
    assert not m.exploration_done


def test_003_R14_republished_cleaning_does_not_change_the_resume_during_escape():
    m = exploring()
    m.on_stuck()
    for _ in range(3):                      # the orchestrator republishes the mode every 0.2 s
        m.on_mode('CLEANING')
    assert m.resume_submode == EXPLORING
    assert m.on_escape_result(True) == [Effect('send_explore')]


def test_003_R14_escape_failure_from_exploring_locks_as_in_001():
    m = exploring()
    m.on_stuck()
    fx = m.on_escape_result(False)
    assert m.submode == ESCAPE_FAILED and m.locked
    assert kinds(fx) == ['lock_stop', 'notify']


def test_003_R14_mode_leaving_cleaning_cancels_explore_and_keeps_the_lock_untouched():
    m = exploring()
    fx = m.on_mode('IDLE')
    assert m.submode == NONE and kinds(fx) == ['cancel_actions']
    assert not m.locked


def test_003_R14_autodocking_while_exploring_cancels_exploration():
    m = exploring()
    fx = m.on_mode('AUTODOCKING')
    assert m.submode == NONE and kinds(fx) == ['cancel_actions']


def test_003_R14_mode_change_during_escape_from_exploring_follows_001_R21():
    m = exploring()
    m.on_stuck()
    m.on_mode('AUTODOCKING')
    assert m.resume_submode == NONE
    m.on_mode('CLEANING')
    assert m.resume_submode == COVERING
    assert m.on_escape_result(True) == [Effect('send_cover_room')]


def test_003_R14_mode_leaving_cleaning_during_escape_resumes_none():
    m = exploring()
    m.on_stuck()
    m.on_mode('IDLE')
    assert m.resume_submode == NONE


def test_003_R14_stuck_with_lock_on_is_ignored_in_exploring():
    m = exploring()
    m.locked = True
    assert m.on_stuck() == []
    assert m.submode == EXPLORING


def test_003_R14_lock_on_blocks_the_start_of_exploration():
    m = SubmodeMachine(explore_on_start=True)
    m.locked = True
    assert m.on_mode('CLEANING') == []
    assert m.submode == NONE


def test_003_R14_explore_result_with_dock_after_coverage_does_not_request_docking():
    m = exploring(dock_after_coverage=True)
    fx = m.on_explore_result(True)
    assert kinds(fx) == ['send_cover_room']      # docking is requested only by the cover_room result (005-R30)


def test_003_R14_explored_flag_is_only_recorded_while_exploring():
    m = exploring()
    assert m.on_explored(True) == []
    assert m.submode == EXPLORING                # the Explore result decides, no double transition


def test_003_R14_snapshot_reports_exploration_state():
    m = exploring()
    m.on_explore_result(True)
    snap = m.snapshot()
    assert snap['exploration_done'] and snap['explore_results'] == 1
