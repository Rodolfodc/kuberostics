"""T904: SelectorCore tests (circuit breaker table, age and watermark filters), simulated clock.

Each test cites the requirement it verifies (Art. VIII). No ROS, no real time.
"""
import math

import pytest

from vacuum_detection.selector_core import (
    CLOSED, CLOUD, HALF_OPEN, LOCAL, OPEN, SelectorCore,
)


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make(**kw):
    clock = Clock()
    events = []
    core = SelectorCore(clock, notify=lambda s, c: events.append((s, c)), **kw)
    return core, clock, events


def cloud(core, clock, age=0.1):
    """A cloud response whose photo is `age` seconds old right now."""
    return core.on_response(CLOUD, clock() - age)


def local(core, clock, age=0.1):
    return core.on_response(LOCAL, clock() - age)


def to_open(core, clock):
    clock.advance(1.6)
    core.tick()
    assert core.state == OPEN


def to_half_open(core, clock):
    to_open(core, clock)
    cloud(core, clock, age=0.2)
    assert core.state == HALF_OPEN


# --- 009a-R05: age filter -------------------------------------------------------------
def test_009a_R05_response_older_than_max_age_is_dropped_and_counted():
    core, clock, _ = make(max_age_s=0.67)
    assert cloud(core, clock, age=0.68) is False
    assert core.snapshot()['dropped_stale'] == 1
    assert core.snapshot()['forwarded'] == 0


def test_009a_R05_response_within_max_age_is_forwarded():
    core, clock, _ = make(max_age_s=0.75)
    assert cloud(core, clock, age=0.75) is True       # exactly representable; the limit is inclusive
    assert cloud(core, clock, age=0.75 + 2 ** -10) is False
    assert core.snapshot()['dropped_stale'] == 1


def test_009a_R05_age_uses_the_injected_robot_clock():
    core, clock, _ = make(max_age_s=0.67)
    stamp = clock() - 0.1
    clock.advance(1.0)                                # the response sat in a queue: age is now 1.1 s
    assert core.on_response(CLOUD, stamp) is False
    assert core.snapshot()['dropped_stale'] == 1


def test_009a_R05_age_filter_applies_in_every_state_to_the_local_source_too():
    core, clock, _ = make(max_age_s=0.67)
    to_open(core, clock)
    assert local(core, clock, age=0.9) is False
    assert core.snapshot()['dropped_stale'] == 1


# --- 009a-R06: watermark filter -------------------------------------------------------
def test_009a_R06_forwards_only_strictly_increasing_stamps_and_updates_watermark():
    core, clock, _ = make()
    s1 = clock() - 0.3
    assert core.on_response(CLOUD, s1) is True
    assert core.watermark == s1
    s2 = clock() - 0.1
    assert core.on_response(CLOUD, s2) is True
    assert core.watermark == s2


def test_009a_R06_repeated_stamp_is_dropped_as_watermark():
    core, clock, _ = make()
    s = clock() - 0.1
    assert core.on_response(CLOUD, s) is True
    assert core.on_response(CLOUD, s) is False
    assert core.snapshot()['dropped_watermark'] == 1


def test_009a_R06_out_of_order_stamp_is_dropped_and_watermark_kept():
    core, clock, _ = make()
    new, old = clock() - 0.1, clock() - 0.3
    core.on_response(CLOUD, new)
    assert core.on_response(CLOUD, old) is False
    assert core.watermark == new
    assert core.snapshot()['dropped_watermark'] == 1


def test_009a_R06_dropped_response_does_not_move_the_watermark():
    core, clock, _ = make(max_age_s=0.75)
    core.on_response(CLOUD, clock() - 0.25)
    wm = core.watermark
    assert core.on_response(CLOUD, clock() - 1.0) is False    # stale (and older than the watermark)
    assert core.watermark == wm


def test_009a_R06_watermark_is_shared_across_sources_and_switches():
    """After OPEN -> CLOSED, cloud responses older than what local already forwarded must not appear."""
    core, clock, _ = make()
    to_half_open(core, clock)
    for _ in range(5):                                       # 2.5 s of healthy cloud responses
        clock.advance(0.5)
        cloud(core, clock, age=0.2)
    clock.advance(0.4)
    assert local(core, clock, age=0.05) is True              # still HALF_OPEN: local forwarded
    wm = core.watermark
    clock.advance(0.1)                                       # 3.0 s after the window started
    assert cloud(core, clock, age=0.2) is False              # closes the breaker, but stamp < watermark
    assert core.state == CLOSED
    assert core.watermark == wm
    assert core.snapshot()['dropped_watermark'] == 1


# --- 009a-R07: source gate ------------------------------------------------------------
def test_009a_R07_closed_forwards_cloud_and_drops_local():
    core, clock, _ = make()
    assert local(core, clock) is False
    assert core.snapshot()['dropped_inactive'] == 1
    assert cloud(core, clock) is True


@pytest.mark.parametrize('prepare', [to_open, to_half_open], ids=['OPEN', 'HALF_OPEN'])
def test_009a_R07_open_and_half_open_forward_local_and_drop_cloud(prepare):
    core, clock, _ = make()
    prepare(core, clock)
    before = core.snapshot()['dropped_inactive']
    assert local(core, clock) is True
    # a cloud response with age > close threshold keeps/returns to OPEN, never forwarded
    assert cloud(core, clock, age=0.6) is False
    assert core.snapshot()['dropped_inactive'] == before + 1


def test_009a_R07_inactive_source_does_not_move_the_watermark():
    core, clock, _ = make()
    local(core, clock, age=0.01)
    assert core.watermark == float('-inf')


# --- 009a-R08: CLOSED -> OPEN ---------------------------------------------------------
def test_009a_R08_starts_closed():
    core, _, _ = make()
    assert core.state == CLOSED


def test_009a_R08_silence_longer_than_open_threshold_opens_and_warns_once():
    core, clock, events = make(open_threshold_s=1.5)
    clock.advance(1.51)
    core.tick()
    assert core.state == OPEN
    assert events == [('WARN', 'CLOUD_DETECTOR_DOWN')]
    clock.advance(5.0)
    core.tick()
    assert events == [('WARN', 'CLOUD_DETECTOR_DOWN')]


def test_009a_R08_silence_up_to_open_threshold_stays_closed():
    core, clock, events = make(open_threshold_s=1.5)
    clock.advance(1.5)
    core.tick()
    assert core.state == CLOSED
    assert events == []


def test_009a_R08_healthy_responses_keep_it_closed_indefinitely():
    core, clock, events = make()
    for _ in range(40):
        clock.advance(0.5)
        cloud(core, clock, age=0.2)
        core.tick()
    assert core.state == CLOSED
    assert events == []


def test_009a_R08_responses_older_than_open_threshold_do_not_count_as_healthy():
    core, clock, events = make(open_threshold_s=1.5)
    for _ in range(4):                                       # slow cloud: every answer is 2 s old
        clock.advance(0.5)
        cloud(core, clock, age=2.0)
    assert core.state == OPEN
    assert events == [('WARN', 'CLOUD_DETECTOR_DOWN')]


def test_009a_R08_dead_zone_closed_cloud_slower_than_max_age_opens_and_local_takes_over():
    """Age 1.0 s is between max_age_s (0.67) and open_threshold_s (1.5): every response is stale, so it
    cannot count as healthy; the breaker must open instead of dropping everything while CLOSED."""
    core, clock, events = make(open_threshold_s=1.5, max_age_s=0.67)
    for _ in range(3):                                       # 1.5 s, one response per 0.5 s
        clock.advance(0.5)
        assert cloud(core, clock, age=1.0) is False
    assert core.state == CLOSED                              # exactly open_threshold_s: not yet
    clock.advance(0.5)
    cloud(core, clock, age=1.0)                              # one tick period later
    assert core.state == OPEN
    assert events == [('WARN', 'CLOUD_DETECTOR_DOWN')]
    clock.advance(0.5)
    cloud(core, clock, age=1.0)
    core.tick()
    assert events == [('WARN', 'CLOUD_DETECTOR_DOWN')]       # once
    assert local(core, clock, age=0.05) is True


def test_009a_R08_response_exactly_at_max_age_counts_as_healthy():
    core, clock, _ = make(max_age_s=0.75, open_threshold_s=1.5)
    for _ in range(8):
        clock.advance(0.5)
        assert cloud(core, clock, age=0.75) is True
    assert core.state == CLOSED


def test_009a_R08_silence_is_evaluated_before_the_late_response_so_the_warning_is_not_lost():
    core, clock, events = make(open_threshold_s=1.5)
    clock.advance(1.6)
    cloud(core, clock, age=0.1)       # CLOSED -> OPEN (WARN) first, then this response lifts it to HALF_OPEN
    assert events == [('WARN', 'CLOUD_DETECTOR_DOWN')]
    assert core.state == HALF_OPEN


def test_009a_R08_transition_is_counted_and_logged():
    logs = []

    class Log:
        def info(self, m):
            logs.append(m)

        def debug(self, m):
            pass
    clock = Clock()
    core = SelectorCore(clock, logger=Log())
    clock.advance(2.0)
    core.tick()
    assert core.snapshot()['transitions'] == 1
    assert any('CLOSED -> OPEN' in m for m in logs)


# --- 009a-R09: OPEN -> HALF_OPEN ------------------------------------------------------
def test_009a_R09_cloud_response_within_close_threshold_goes_half_open():
    core, clock, events = make(close_threshold_s=0.5)
    to_open(core, clock)
    cloud(core, clock, age=0.5)                              # inclusive limit
    assert core.state == HALF_OPEN
    assert events == [('WARN', 'CLOUD_DETECTOR_DOWN')]       # no notification for this edge


def test_009a_R09_cloud_response_above_close_threshold_stays_open():
    core, clock, _ = make(close_threshold_s=0.5)
    to_open(core, clock)
    cloud(core, clock, age=0.51)
    assert core.state == OPEN


def test_009a_R09_local_responses_never_leave_open():
    core, clock, _ = make()
    to_open(core, clock)
    for _ in range(10):
        clock.advance(0.5)
        local(core, clock, age=0.05)
    assert core.state == OPEN


# --- 009a-R10: HALF_OPEN -> CLOSED / OPEN, hysteresis ---------------------------------
def feed_cloud(core, clock, seconds, period=0.5, age=0.2):
    for _ in range(int(round(seconds / period))):
        clock.advance(period)
        cloud(core, clock, age=age)


def test_009a_R10_stable_window_closes_and_notifies_recovered_once():
    core, clock, events = make(stable_window_s=3.0)
    to_half_open(core, clock)
    feed_cloud(core, clock, 3.0)
    assert core.state == CLOSED
    assert events == [('WARN', 'CLOUD_DETECTOR_DOWN'), ('INFO', 'CLOUD_DETECTOR_RECOVERED')]


def test_009a_R10_does_not_close_before_the_window_ends():
    core, clock, events = make(stable_window_s=3.0)
    to_half_open(core, clock)
    feed_cloud(core, clock, 2.5)
    assert core.state == HALF_OPEN
    assert ('INFO', 'CLOUD_DETECTOR_RECOVERED') not in events


def test_009a_R10_one_slow_response_returns_to_open_and_restarts_the_window():
    core, clock, events = make(stable_window_s=3.0)
    to_half_open(core, clock)
    feed_cloud(core, clock, 2.0)
    clock.advance(0.5)
    cloud(core, clock, age=0.6)                              # violation
    assert core.state == OPEN
    cloud(core, clock, age=0.2)                              # back to HALF_OPEN, window restarts here
    assert core.state == HALF_OPEN
    feed_cloud(core, clock, 2.5)
    assert core.state == HALF_OPEN                           # 2.5 s since the restart: still not enough
    feed_cloud(core, clock, 0.5)
    assert core.state == CLOSED
    assert events.count(('INFO', 'CLOUD_DETECTOR_RECOVERED')) == 1


def test_009a_R10_return_to_open_from_half_open_does_not_notify_down_again():
    core, clock, events = make()
    to_half_open(core, clock)
    clock.advance(0.5)
    cloud(core, clock, age=0.9)
    assert core.state == OPEN
    assert events == [('WARN', 'CLOUD_DETECTOR_DOWN')]


def test_009a_R10_cloud_silence_in_half_open_returns_to_open():
    core, clock, _ = make(open_threshold_s=1.5)
    to_half_open(core, clock)
    clock.advance(1.6)
    core.tick()
    assert core.state == OPEN


def test_009a_R10_hysteresis_age_between_close_and_open_thresholds_does_not_close_it():
    """Age 0.6 s is healthy to stay CLOSED (<= 0.67) but not good enough to leave OPEN (> 0.5)."""
    core, clock, _ = make()
    to_open(core, clock)
    for _ in range(10):
        clock.advance(0.5)
        cloud(core, clock, age=0.6)
    assert core.state == OPEN


def test_009a_R10_hysteresis_age_between_close_and_max_age_keeps_closed_once_closed():
    core, clock, _ = make()
    to_half_open(core, clock)
    feed_cloud(core, clock, 3.0)
    assert core.state == CLOSED
    feed_cloud(core, clock, 5.0, age=0.625)                  # worse than close (0.5), within max_age (0.67)
    assert core.state == CLOSED


def test_009a_R10_after_recovery_a_new_outage_opens_again():
    core, clock, events = make()
    to_half_open(core, clock)
    feed_cloud(core, clock, 3.0)
    assert core.state == CLOSED
    clock.advance(1.6)
    core.tick()
    assert core.state == OPEN
    assert events.count(('WARN', 'CLOUD_DETECTOR_DOWN')) == 2


def test_009a_R10_response_that_closes_the_breaker_is_forwarded_from_cloud():
    core, clock, _ = make()
    to_half_open(core, clock)
    feed_cloud(core, clock, 2.5)
    clock.advance(0.5)
    assert cloud(core, clock, age=0.2) is True
    assert core.state == CLOSED


# --- 009a-R10: HALF_OPEN silence --------------------------------------------------------
def test_009a_R10_silence_in_half_open_returns_to_open_without_notification():
    core, clock, events = make(open_threshold_s=1.5)
    to_half_open(core, clock)
    events.clear()
    clock.advance(1.5)
    core.tick()
    assert core.state == HALF_OPEN                           # exactly open_threshold_s: not yet
    clock.advance(0.25)
    core.tick()
    assert core.state == OPEN
    assert events == []


def test_009a_R10_response_after_a_long_gap_does_not_close_the_breaker():
    core, clock, events = make(stable_window_s=3.0)
    to_half_open(core, clock)
    events.clear()
    clock.advance(10.0)                                      # long silence, no tick in between
    cloud(core, clock, age=0.25)                             # R09 again: OPEN -> HALF_OPEN, window restarts
    assert core.state == HALF_OPEN
    assert events == []
    feed_cloud(core, clock, 2.5)
    assert core.state == HALF_OPEN
    feed_cloud(core, clock, 0.5)
    assert core.state == CLOSED
    assert events == [('INFO', 'CLOUD_DETECTOR_RECOVERED')]


def test_009a_R10_sparse_cloud_responses_in_half_open_never_close_the_breaker():
    """One response every 2 s: each gap exceeds open_threshold_s, so the window never completes."""
    core, clock, events = make(stable_window_s=3.0, open_threshold_s=1.5)
    to_half_open(core, clock)
    events.clear()
    for _ in range(6):
        clock.advance(2.0)
        cloud(core, clock, age=0.25)
        assert core.state != CLOSED
    assert ('INFO', 'CLOUD_DETECTOR_RECOVERED') not in events


def test_009a_R10_responses_spaced_within_open_threshold_can_close_it():
    core, clock, _ = make(stable_window_s=3.0, open_threshold_s=1.5)
    to_half_open(core, clock)
    for _ in range(3):
        clock.advance(1.0)
        cloud(core, clock, age=0.25)
    assert core.state == CLOSED


# --- counters (009a spec, "Selector processing order and counters") ----------------------
def total(core):
    m = core.snapshot()
    return m['forwarded'] + m['dropped_inactive'] + m['dropped_stale'] + m['dropped_watermark']


def test_009a_R07_stale_cloud_response_while_open_counts_only_as_inactive():
    core, clock, _ = make()
    to_open(core, clock)
    assert cloud(core, clock, age=5.0) is False
    m = core.snapshot()
    assert (m['dropped_inactive'], m['dropped_stale']) == (1, 0)


def test_009a_R05_stale_cloud_response_while_closed_counts_only_as_stale():
    core, clock, _ = make()
    assert cloud(core, clock, age=0.75) is False
    m = core.snapshot()
    assert (m['dropped_stale'], m['dropped_inactive']) == (1, 0)


def test_009a_R05_every_response_lands_in_exactly_one_counter():
    core, clock, _ = make()
    calls = 0
    for i in range(30):
        clock.advance(0.25)
        for src, age in ((CLOUD, 0.125), (LOCAL, 0.125), (CLOUD, 0.75), (LOCAL, 2.0), (CLOUD, 0.125)):
            core.on_response(src, clock() - age)
            calls += 1
        if i == 12:
            clock.advance(2.0)                               # outage: CLOSED -> OPEN -> ...
        assert total(core) == calls
    assert core.snapshot()['transitions'] >= 1


# --- constructor invariant: close_threshold_s < max_age_s <= open_threshold_s ------------
@pytest.mark.parametrize('kw', [
    dict(close_threshold_s=0.5, max_age_s=0.5),              # close == max
    dict(close_threshold_s=0.5, max_age_s=0.25),             # close > max
    dict(max_age_s=2.0, open_threshold_s=1.5),               # max > open
])
def test_009a_R08_invalid_threshold_ordering_is_rejected(kw):
    with pytest.raises(ValueError):
        SelectorCore(Clock(), **kw)


def test_009a_R08_max_age_equal_to_open_threshold_is_accepted():
    SelectorCore(Clock(), max_age_s=1.5, open_threshold_s=1.5)


def test_009a_R05_drops_are_logged_at_debug_level():
    msgs = []

    class Log:
        def info(self, m):
            pass

        def debug(self, m):
            msgs.append(m)
    clock = Clock()
    core = SelectorCore(clock, logger=Log())
    core.on_response(LOCAL, clock() - 0.125)
    core.on_response(CLOUD, clock() - 2.0)
    s = clock() - 0.125
    core.on_response(CLOUD, s)
    core.on_response(CLOUD, s)
    assert len(msgs) == 3
    assert math.isclose(core.watermark, s)
