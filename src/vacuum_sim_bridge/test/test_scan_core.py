"""Scan conversion. Requirement basis: spec 000 section 4 (frozen /scan), D-07; robot_report.txt section 3.
Requirement: 001-R23."""
import math

from vacuum_sim_bridge.profile import RobotProfile
from vacuum_sim_bridge.scan_core import convert_scan

P = RobotProfile()


def test_misses_and_out_of_range_become_inf():
    raw = [0.0, 0.054, 0.055, 1.0, 10.0, 10.1, float('nan'), -1.0]
    scan = convert_scan(raw, P)
    assert scan.ranges == [math.inf, math.inf, 0.055, 1.0, 10.0, math.inf, math.inf, math.inf]
    assert not any(r == 0 for r in scan.ranges)


def test_angles_match_the_model_sweep():
    scan = convert_scan([1.0] * 452, P)
    assert scan.angle_min == -math.pi / 2
    assert abs(scan.angle_increment - math.pi / 450) < 1e-12
    assert abs(scan.angle_max - (scan.angle_min + 451 * scan.angle_increment)) < 1e-12
    assert abs(math.degrees(scan.angle_max) - 90.4) < 1e-9      # last beam of the model script
    assert (scan.range_min, scan.range_max) == (0.055, 10.0)
    assert scan.warning is None


def test_beam_count_comes_from_the_data_and_warns_on_mismatch():
    scan = convert_scan([1.0] * 362, P)
    assert len(scan.ranges) == 362
    assert abs(scan.angle_max - (scan.angle_min + 361 * P.scan_angle_increment)) < 1e-12
    assert '362' in scan.warning


def test_too_few_beams_returns_none():
    assert convert_scan([], P) is None
    assert convert_scan([1.0], P) is None
