"""Checks against a running CoppeliaSim with myRobotLaser.ttm loaded, GUI mode (local only, Art. V).
Run: VACUUM_SIM_LIVE=1 python3 -m pytest -m sim test/test_sim_live.py   (the simulation must be running;
ask the sim-operator). They exercise ZmqSimClient, the part the fake cannot cover."""
import math
import os

import numpy as np
import pytest

from vacuum_sim_bridge.camera_core import convert_depth
from vacuum_sim_bridge.odometry_core import GroundTruthOdometry, WheelOdometry
from vacuum_sim_bridge.profile import RobotProfile
from vacuum_sim_bridge.scan_core import convert_scan

pytestmark = [pytest.mark.sim,
              pytest.mark.skipif(os.environ.get('VACUUM_SIM_LIVE') != '1', reason='needs CoppeliaSim')]
P = RobotProfile()


@pytest.fixture(scope='module')
def sim():
    from vacuum_sim_bridge.sim_client import ZmqSimClient
    return ZmqSimClient()


def test_live_scan_has_452_beams_and_no_zero_ranges(sim):
    robot = sim.get_object(P.robot_path)
    scan = convert_scan(sim.get_float_array(robot, P.scan_signal), P)
    assert scan.warning is None and len(scan.ranges) == 452
    assert not any(r == 0 for r in scan.ranges)


def test_live_rgb_buffer_has_the_expected_size(sim):
    from vacuum_sim_bridge.camera_core import convert_rgb
    buf, w, h = sim.read_rgb(sim.get_object(P.rgb_sensor_path))
    assert (w, h) == (P.camera_width, P.camera_height)
    assert convert_rgb(buf, w, h, P) is not None


def test_live_depth_floor_is_nearer_at_the_bottom_of_the_image(sim):
    """Pins the row order: the camera is pitched down, so the lower rows see the floor close by. If this fails,
    flip camera_flip_vertical."""
    depth = sim.get_object(P.depth_sensor_path)
    buf, w, h = sim.read_depth(depth)
    metres = convert_depth(buf, w, h, P)
    top, bottom = np.nanmean(metres[: h // 3]), np.nanmean(metres[-h // 3:])
    assert bottom < top


def test_live_drive_matches_between_ground_truth_and_wheel_odometry(sim):
    """Drives forward through the wheel signals for ~2 s of simulation and compares both odometry sources
    (needs the simulation running in real time stepping, GUI mode)."""
    import time
    robot = sim.get_object(P.robot_path)
    left, right = sim.get_object(P.left_motor_path), sim.get_object(P.right_motor_path)
    gt, wh = GroundTruthOdometry(P), WheelOdometry(P)
    t0 = sim.get_sim_time()
    g = gt.update(sim.get_object_pose(robot), t0)
    w = wh.update(sim.get_joint_position(left), sim.get_joint_position(right), t0)
    deadline = time.monotonic() + 120.0
    while sim.get_sim_time() - t0 < 2.0 and time.monotonic() < deadline:
        sim.set_wheel_speeds(robot, 0.2, 0.2)
    sim.set_wheel_speeds(robot, 0.0, 0.0)
    t = sim.get_sim_time()
    g = gt.update(sim.get_object_pose(robot), t)
    w = wh.update(sim.get_joint_position(left), sim.get_joint_position(right), t)
    assert g.x > 0.1 and abs(g.y) < 0.05 and abs(g.yaw) < 0.05          # forward is +x of the odom frame
    assert math.isclose(g.x, w.x, rel_tol=0.1, abs_tol=0.03)
