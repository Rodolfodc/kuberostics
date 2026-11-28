"""Pure tests of the 005-R34 watchdog classes (no ROS)."""
from vacuum_supervisor.feedback_watchdog import (FEEDBACK_STALE, SERVER_NOT_READY, CancelWindow,
                                                 FeedbackWatchdog)


def test_005_R34_not_armed_never_reports_a_loss():
    w = FeedbackWatchdog(2.5)
    assert w.check(1000.0, server_ready=False) is None


def test_005_R34_silence_below_the_limit_is_alive_and_at_the_limit_is_lost_once():
    w = FeedbackWatchdog(2.5)
    w.arm(10.0)
    assert w.check(12.4, True) is None
    assert w.check(12.5, True) == FEEDBACK_STALE
    assert w.check(13.0, True) is None            # reported once, then disarmed
    assert not w.armed


def test_005_R34_feedback_resets_the_clock():
    w = FeedbackWatchdog(2.5)
    w.arm(0.0)
    w.feed(2.0)
    assert w.check(4.4, True) is None
    assert w.check(4.5, True) == FEEDBACK_STALE


def test_005_R34_server_not_ready_is_lost_at_once():
    w = FeedbackWatchdog(2.5)
    w.arm(0.0)
    assert w.check(0.1, server_ready=False) == SERVER_NOT_READY


def test_005_R34_feedback_while_disarmed_is_ignored_and_disarm_stops_the_watch():
    w = FeedbackWatchdog(1.0)
    w.feed(0.0)
    assert not w.armed
    w.arm(0.0)
    w.disarm()
    assert w.check(100.0, False) is None


def test_005_R34_snapshot_counts_losses():
    w = FeedbackWatchdog(1.0)
    w.arm(0.0)
    w.feed(0.5)
    w.check(2.0, True)
    snap = w.snapshot()
    assert snap['watchdog_losses'] == 1 and snap['watchdog_feedback'] == 1 and not snap['watchdog_armed']


def test_005_R34_cancel_window_retries_until_done_or_expired():
    c = CancelWindow(1.0)
    assert not c.should_try(0.0)
    c.start(10.0)
    assert c.should_try(10.5) and c.pending
    c.done()
    assert not c.should_try(10.6)
    c.start(20.0)
    assert c.should_try(21.0)
    assert not c.should_try(21.1)                  # expired: gives up
    assert not c.pending


# --- 003-R31, 005-R34: no new coverage goal while a cancel-all of /navigate_to_pose is being delivered ---------
def test_003_R31_cancel_window_holds_new_goals_until_done():
    w = CancelWindow(1.0)
    assert not w.holds(0.0)
    w.start(10.0)
    assert w.holds(10.0) and w.holds(10.9)
    w.done()
    assert not w.holds(10.2)


def test_003_R31_cancel_window_stops_holding_after_the_window_even_if_never_answered():
    w = CancelWindow(1.0)
    w.start(10.0)
    assert not w.holds(11.1)          # nav_cancel_s elapsed: the retry is over, the goal may go
