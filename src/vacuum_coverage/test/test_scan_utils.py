"""Scan helpers used by EscapeScript and the escape node (spec 005): sector minimum and left/right means.

Convention (documented in scan_utils): angles are in the laser frame, 0 = straight ahead, positive = left
(counter-clockwise); a sample is at `angle_min + i * angle_increment`. No ROS."""
import math

import pytest

from vacuum_coverage.core.scan_utils import sector_min, side_means

INC = math.radians(1.0)


def scan(fn, n=360, angle_min=-math.pi):
    return [fn(angle_min + i * INC) for i in range(n)], angle_min


def test_005_R08_side_means_left_larger():
    ranges, a0 = scan(lambda a: 2.0 if a > 0 else 0.5)
    left, right = side_means(ranges, a0, INC)
    assert left == pytest.approx(2.0) and right == pytest.approx(0.5)


def test_005_R08_side_means_ignore_nan_and_nonpositive():
    ranges, a0 = scan(lambda a: float('nan') if a > 0 else 1.0)
    left, right = side_means(ranges, a0, INC)
    assert left == 0.0 and right == pytest.approx(1.0)
    ranges, a0 = scan(lambda a: -1.0 if a > 0 else 1.0)
    assert side_means(ranges, a0, INC)[0] == 0.0


def test_005_R08_side_means_cap_inf_deterministically():
    ranges, a0 = scan(lambda a: float('inf') if a > 0 else 1.0)
    left, right = side_means(ranges, a0, INC, range_cap_m=5.0)
    assert left == pytest.approx(5.0) and right == pytest.approx(1.0)


def test_005_R08_side_means_cap_finite_values_too():
    ranges, a0 = scan(lambda a: 30.0 if a > 0 else 1.0)
    assert side_means(ranges, a0, INC, range_cap_m=5.0)[0] == pytest.approx(5.0)


def test_005_R08_side_means_front_only_laser():
    ranges, a0 = scan(lambda a: 1.0 if a > 0 else 3.0, n=181, angle_min=-math.pi / 2)
    left, right = side_means(ranges, a0, INC)
    assert left == pytest.approx(1.0) and right == pytest.approx(3.0)


def test_005_R08_side_means_empty():
    assert side_means([], -1.0, INC) == (0.0, 0.0)


def test_005_R08_sector_min_front():
    ranges, a0 = scan(lambda a: 0.3 if abs(a) < math.radians(5) else 2.0)
    assert sector_min(ranges, a0, INC, 0.0, math.radians(20)) == pytest.approx(0.3)
    assert sector_min(ranges, a0, INC, math.pi, math.radians(20)) == pytest.approx(2.0)


def test_005_R08_sector_min_inf_is_free_space():
    ranges, a0 = scan(lambda a: float('inf'))
    assert sector_min(ranges, a0, INC, 0.0, math.radians(20)) == float('inf')


def test_005_R08_sector_min_none_when_the_scan_does_not_cover_the_sector():
    ranges, a0 = scan(lambda a: 1.0, n=181, angle_min=-math.pi / 2)     # front-only laser
    assert sector_min(ranges, a0, INC, math.pi, math.radians(20)) is None
    assert sector_min(ranges, a0, INC, 0.0, math.radians(20)) == pytest.approx(1.0)


def test_005_R08_sector_min_none_when_all_samples_invalid():
    ranges, a0 = scan(lambda a: float('nan'))
    assert sector_min(ranges, a0, INC, 0.0, math.radians(20)) is None


def test_005_R08_sector_min_wraps_around_pi():
    ranges, a0 = scan(lambda a: 0.2 if abs(abs(a) - math.pi) < math.radians(3) else 2.0)
    assert sector_min(ranges, a0, INC, math.pi, math.radians(10)) == pytest.approx(0.2)
