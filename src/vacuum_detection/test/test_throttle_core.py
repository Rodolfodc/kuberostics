"""ThrottleCore tests (rate limiting by photo stamp), simulated clock. No ROS.

009a-R03: the image sent to the cloud leaves at `image_rate_hz`. 009a-R11: the rate does not depend on the
breaker state, so the core has no notion of state at all (the probe keeps the same rate by construction).
"""
import pytest

from vacuum_detection.throttle_core import ThrottleCore


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def feed(core, rate_in_hz, seconds, t0=10.0):
    """Offers a stamped stream and returns the stamps that passed."""
    passed = []
    n = int(rate_in_hz * seconds)
    for i in range(n):
        stamp = t0 + i / rate_in_hz
        if core.offer(stamp):
            passed.append(stamp)
    return passed


def test_009a_R03_first_frame_passes_immediately():
    assert ThrottleCore(clock=Clock(), rate_hz=2.0).offer(10.0) is True


def test_009a_R03_30hz_stream_is_limited_to_2hz():
    core = ThrottleCore(clock=Clock(), rate_hz=2.0)
    passed = feed(core, 30.0, 10.0)
    assert len(passed) == pytest.approx(20, abs=1)


def test_009a_R03_spacing_between_passed_frames_is_at_least_the_period():
    core = ThrottleCore(clock=Clock(), rate_hz=2.0)
    passed = feed(core, 30.0, 5.0)
    gaps = [b - a for a, b in zip(passed, passed[1:])]
    assert min(gaps) >= 0.5 - 1e-9


def test_009a_R03_slow_input_is_never_delayed_or_dropped():
    core = ThrottleCore(clock=Clock(), rate_hz=2.0)
    assert len(feed(core, 1.0, 5.0)) == 5


def test_009a_R11_rate_is_independent_of_anything_but_the_stamp():
    # Same stream twice, with a different arrival clock: same frames pass.
    a = feed(ThrottleCore(clock=Clock(0.0), rate_hz=2.0), 30.0, 3.0)
    b = feed(ThrottleCore(clock=Clock(9999.0), rate_hz=2.0), 30.0, 3.0)
    assert a == b


def test_009a_R03_stamp_going_backwards_restarts_the_throttle_instead_of_freezing_it():
    core = ThrottleCore(clock=Clock(), rate_hz=2.0)
    assert core.offer(1000.0)
    assert core.offer(5.0)          # simulation reset: must not wait 995 s
    assert not core.offer(5.1)
    assert core.offer(5.5)


def test_009a_R03_unstamped_frame_falls_back_to_the_injected_clock():
    clock = Clock(50.0)
    core = ThrottleCore(clock=clock, rate_hz=2.0)
    assert core.offer(0.0)
    clock.t = 50.2
    assert not core.offer(0.0)
    clock.t = 50.6
    assert core.offer(0.0)


def test_009a_R03_counters_add_up_to_the_frames_offered():
    core = ThrottleCore(clock=Clock(), rate_hz=2.0)
    feed(core, 30.0, 2.0)
    snap = core.snapshot()
    assert snap['passed'] + snap['dropped_rate'] == 60
    assert snap['passed'] > 0


def test_009a_R03_non_positive_rate_is_rejected():
    with pytest.raises(ValueError):
        ThrottleCore(clock=Clock(), rate_hz=0.0)
