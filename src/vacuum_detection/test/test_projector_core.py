"""T905: ProjectorCore tests with a synthetic TF buffer (transform at the photo timestamp, drop without TF).

Each test cites the requirement it verifies (Art. VIII). No ROS.
"""
import math

import pytest

from vacuum_detection.projection_core import ProjectorCore, Transform, pixel_to_camera

IDENTITY = (0.0, 0.0, 0.0, 1.0)
K = dict(fx=100.0, fy=100.0, cx=50.0, cy=50.0)


class FakeTf:
    """Synthetic TF buffer: transforms stored by exact stamp; records every query."""

    def __init__(self, by_stamp):
        self.by_stamp = by_stamp
        self.queries = []

    def __call__(self, stamp, timeout_s):
        self.queries.append((stamp, timeout_s))
        return self.by_stamp.get(stamp)


def core_with(tf, **kw):
    return ProjectorCore(tf, **K, **kw)


# --- 009a-R12: the transform is the one at the photo stamp -----------------------------
def test_009a_R12_uses_the_transform_at_the_detection_stamp_not_the_latest():
    tf = FakeTf({1.0: Transform((1.0, 0.0, 0.0), IDENTITY), 2.0: Transform((5.0, 0.0, 0.0), IDENTITY)})
    core = core_with(tf)
    # the robot moved between t=1 and t=2; a detection of the t=1 photo that arrives "now" (t>=2)
    # must still land where the robot was at t=1
    p = core.project(1.0, u=50.0, v=50.0, depth=2.0)        # optical axis, 2 m ahead
    assert p == pytest.approx((1.0, 0.0, 2.0))
    p2 = core.project(2.0, u=50.0, v=50.0, depth=2.0)
    assert p2 == pytest.approx((5.0, 0.0, 2.0))
    assert [q[0] for q in tf.queries] == [1.0, 2.0]


def test_009a_R12_queries_tf_with_the_configured_timeout():
    tf = FakeTf({1.0: Transform((0, 0, 0), IDENTITY)})
    core_with(tf, tf_timeout_s=0.1).project(1.0, 50, 50, 1.0)
    assert tf.queries == [(1.0, 0.1)]
    tf2 = FakeTf({1.0: Transform((0, 0, 0), IDENTITY)})
    core_with(tf2, tf_timeout_s=0.25).project(1.0, 50, 50, 1.0)
    assert tf2.queries == [(1.0, 0.25)]


def test_009a_R12_never_uses_the_arrival_instant():
    """The core has no clock: the only time it can pass to TF is the detection stamp."""
    tf = FakeTf({7.25: Transform((0, 0, 0), IDENTITY)})
    core = core_with(tf)
    core.project(7.25, 50, 50, 1.0)
    core.project(7.30, 50, 50, 1.0)                          # no TF at that stamp: dropped, no fallback
    assert [q[0] for q in tf.queries] == [7.25, 7.30]
    assert core.snapshot()['projected'] == 1


def test_009a_R12_missing_tf_drops_the_detection_and_counts_it():
    core = core_with(FakeTf({}))
    assert core.project(1.0, 50, 50, 1.0) is None
    assert core.project(2.0, 50, 50, 1.0) is None
    snap = core.snapshot()
    assert snap['dropped_tf'] == 2
    assert snap['projected'] == 0


def test_009a_R12_missing_tf_logs_a_warning():
    msgs = []

    class Log:
        def warning(self, m):
            msgs.append(m)
    ProjectorCore(FakeTf({}), **K, logger=Log()).project(1.0, 50, 50, 1.0)
    assert len(msgs) == 1


def test_009a_R12_a_later_tf_does_not_resurrect_a_dropped_detection():
    tf = FakeTf({})
    core = core_with(tf)
    assert core.project(1.0, 50, 50, 1.0) is None
    tf.by_stamp[1.0] = Transform((0, 0, 0), IDENTITY)
    assert core.project(1.0, 50, 50, 1.0) is not None       # a new detection with that stamp works
    assert core.snapshot() == {'projected': 1, 'dropped_tf': 1, 'dropped_depth': 0}


# --- geometry of the projection (supports 009a-R12/R13) --------------------------------
def test_009a_R12_pixel_to_camera_pinhole():
    assert pixel_to_camera(50, 50, 3.0, **K) == pytest.approx((0.0, 0.0, 3.0))
    assert pixel_to_camera(100, 50, 2.0, **K) == pytest.approx((1.0, 0.0, 2.0))
    assert pixel_to_camera(50, 0, 2.0, **K) == pytest.approx((0.0, -1.0, 2.0))


def test_009a_R12_rotation_and_translation_are_applied():
    s = math.sqrt(0.5)
    yaw90 = (0.0, 0.0, s, s)                                 # +90 degrees about z
    tf = FakeTf({1.0: Transform((10.0, 20.0, 0.0), yaw90)})
    p = core_with(tf).project(1.0, u=100.0, v=50.0, depth=2.0)   # camera point (1, 0, 2)
    assert p == pytest.approx((10.0, 21.0, 2.0))              # (1,0,2) rotated to (0,1,2), then shifted


@pytest.mark.parametrize('depth', [0.0, -1.0, float('nan'), float('inf')])
def test_009a_R12_invalid_depth_is_dropped_without_querying_tf(depth):
    tf = FakeTf({1.0: Transform((0, 0, 0), IDENTITY)})
    core = core_with(tf)
    assert core.project(1.0, 50, 50, depth) is None
    assert tf.queries == []
    assert core.snapshot()['dropped_depth'] == 1
