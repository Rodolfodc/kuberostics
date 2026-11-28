"""Small pure helpers of the stuck_detector and escape_server nodes (spec 005-R04, R05, R08). No ROS."""
import math

from vacuum_coverage.core.sensor_cache import (
    RisingEdge, ScanCache, quaternion_to_yaw, scan_geometry_or_default, yaw_to_quaternion)


def test_005_R04_yaw_quaternion_roundtrip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert math.isclose(quaternion_to_yaw(*yaw_to_quaternion(yaw)), yaw, abs_tol=1e-9)


def test_005_R04_scan_cache_returns_fresh_ranges_only():
    c = ScanCache()
    assert c.fresh_ranges(0.0, 0.5) == []
    c.store(1.0, [1.0, 2.0], -1.0, 0.5)
    assert c.fresh_ranges(1.4, 0.5) == [1.0, 2.0]
    assert c.fresh_ranges(1.6, 0.5) == []


def test_005_R04_scan_cache_keeps_geometry():
    c = ScanCache()
    c.store(2.0, [1.0], -0.5, 0.1)
    assert c.geometry() == (-0.5, 0.1)


def test_005_R05_first_true_is_a_press_and_only_rising_edges_count():
    e = RisingEdge()
    assert [e.update(v) for v in (True, True, False, True, False, False, True)] == [
        True, False, False, True, False, False, True]


def test_005_R08_default_geometry_spans_the_full_circle():
    angle_min, inc = scan_geometry_or_default(None, 360)
    assert math.isclose(angle_min, -math.pi) and math.isclose(inc, 2 * math.pi / 360)
    assert scan_geometry_or_default((-1.0, 0.1), 360) == (-1.0, 0.1)
