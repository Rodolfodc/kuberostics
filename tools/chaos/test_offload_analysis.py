"""Tests of the pure chaos analysis. Run: scripts/test-chaos-tools.sh (Docker, python only, no ROS)."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import offload_analysis as oa  # noqa: E402


def st(t, state, fwd=0, ina=0, stale=0, wm=0, tr=0):
    return (t, state, fwd, ina, stale, wm, tr)


def test_009a_R15_time_to_open_uses_the_notification_when_present():
    statuses = [st(10, oa.CLOSED), st(11, oa.CLOSED), st(12, oa.OPEN), st(13, oa.OPEN)]
    rep = oa.analyze(statuses, [(11.9, oa.DOWN_CODE)], [], [], [(10.4, 10.9)])
    k = rep.kills[0]
    assert abs(k.time_to_open_s - 1.5) < 1e-9 and k.time_to_open_source == "notification"
    assert k.open_within_bound is True   # 1.5 <= 2.0


def test_009a_R15_falls_back_to_the_first_open_status_and_flags_a_late_open():
    statuses = [st(10, oa.CLOSED), st(11, oa.CLOSED), st(13, oa.OPEN)]
    k = oa.analyze(statuses, [], [], [], [(10.4, 11.0)]).kills[0]
    assert k.time_to_open_source == "status" and abs(k.time_to_open_s - 2.6) < 1e-9
    assert k.open_within_bound is False


def test_009a_R15_no_open_is_reported_not_hidden():
    k = oa.analyze([st(10, oa.CLOSED), st(20, oa.CLOSED)], [], [], [], [(10.5, 11.0)]).kills[0]
    assert k.time_to_open_s is None and k.open_within_bound is None and k.time_to_open_source == "none"


def test_009a_R15_mode_change_during_the_window_is_detected():
    statuses = [st(0, oa.CLOSED), st(3, oa.OPEN), st(9, oa.CLOSED)]
    ok = [(t, "CLEANING") for t in range(0, 10)]
    bad = ok[:5] + [(5, "IDLE")] + ok[5:]
    assert oa.analyze(statuses, [(2.0, oa.DOWN_CODE)], ok, [], [(1.0, 1.5)]).kills[0].mode_changed is False
    assert oa.analyze(statuses, [(2.0, oa.DOWN_CODE)], bad, [], [(1.0, 1.5)]).kills[0].mode_changed is True


def test_009a_R16_time_to_closed_and_counter_deltas():
    statuses = [st(0, oa.CLOSED, fwd=10), st(3, oa.OPEN, fwd=10, ina=4), st(8, oa.HALF_OPEN, fwd=12, ina=9),
                st(12, oa.CLOSED, fwd=12, ina=15, stale=2, wm=1)]
    k = oa.analyze(statuses, [(2.2, oa.DOWN_CODE)], [], [], [(1.0, 6.0)]).kills[0]
    assert k.time_to_closed_s == 11.0 and k.time_closed_after_ready_s == 6.0
    assert k.counter_deltas == {"forwarded": 2, "dropped_inactive": 15, "dropped_stale": 2, "dropped_watermark": 1}


def test_009a_R16_each_kill_is_confined_to_its_own_window():
    statuses = [st(0, oa.CLOSED), st(3, oa.OPEN), st(9, oa.CLOSED), st(13, oa.OPEN), st(19, oa.CLOSED)]
    rep = oa.analyze(statuses, [(2.5, oa.DOWN_CODE), (12.5, oa.DOWN_CODE)], [], [], [(1.0, 5.0), (11.0, 15.0)])
    assert [round(k.time_to_closed_s, 1) for k in rep.kills] == [8.0, 8.0]


def test_009a_R16_monotonic_violations():
    good = [(0, 1.0), (1, 1.5), (2, 2.0)]
    assert oa.monotonic_violations(good) == 0
    assert oa.monotonic_violations([(0, 1.0), (1, 1.0), (2, 0.5), (3, 2.0)]) == 2   # duplicate and out of order


def test_markdown_renders_all_kills():
    rep = oa.analyze([st(0, oa.CLOSED), st(3, oa.OPEN), st(9, oa.CLOSED)], [(2, oa.DOWN_CODE)], [], [(0, 1.0)], [(1, 5)])
    md = oa.to_markdown(rep)
    assert md.count("\n| 1 |") == 1 and "stamp violations (R16): 0" in md
