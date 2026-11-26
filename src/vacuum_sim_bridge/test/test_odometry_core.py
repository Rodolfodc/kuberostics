"""Odometry integration. Basis: Q-03 (odometry source), robot_report.txt section 5 (the stock script is wrong:
forward is +Y, heading sign inverted, 25 percent long)."""
import math

from vacuum_sim_bridge.geometry import quaternion_from_yaw
from vacuum_sim_bridge.odometry_core import GroundTruthOdometry, WheelOdometry
from vacuum_sim_bridge.profile import RobotProfile

P = RobotProfile()


def joints(left_dist, right_dist):
    """Joint angles after the wheels travelled the given distances forward (forward = negative joint angle)."""
    return left_dist / P.wheel_radius * P.joint_sign, right_dist / P.wheel_radius * P.joint_sign


def test_wheels_forward_is_plus_x_with_the_right_length():
    o = WheelOdometry(P)
    o.update(*joints(0, 0), sim_time=0.0)
    for i in range(1, 11):                       # per-update travel must stay below half a joint turn
        s = o.update(*joints(0.1 * i, 0.1 * i), sim_time=0.5 * i)
    assert math.isclose(s.x, 1.0, abs_tol=1e-9) and abs(s.y) < 1e-9 and abs(s.yaw) < 1e-9
    assert math.isclose(s.vx, 0.2) and abs(s.wz) < 1e-12


def test_wheels_positive_rotation_increases_yaw_about_the_axle():
    o = WheelOdometry(P)
    o.update(*joints(0, 0), sim_time=0.0)
    quarter = math.pi / 2 * P.wheel_separation / 2           # each wheel travels this arc for 90 deg in place
    for i in range(1, 5):
        s = o.update(*joints(-quarter * i / 4, quarter * i / 4), sim_time=0.5 * i)
    assert math.isclose(s.yaw, math.pi / 2, abs_tol=1e-9)
    assert math.isclose(s.wz, math.pi / 4)
    # the axle centre stays put, so the origin (0.07 behind it) moves from (0,0) to (0.07, -0.07)
    assert math.isclose(s.x, P.axle_offset_x, abs_tol=1e-9) and math.isclose(s.y, -P.axle_offset_x, abs_tol=1e-9)
    assert math.isclose(s.vy, -s.wz * P.axle_offset_x)


def test_wheels_arc_matches_geometry():
    o = WheelOdometry(P)
    o.update(*joints(0, 0), sim_time=0.0)
    radius = 0.5                                               # arc of the axle centre, turning left
    dyaw = math.pi / 2
    dl, dr = (radius - 0.1) * dyaw, (radius + 0.1) * dyaw
    s = None
    n = 50
    for i in range(1, n + 1):
        s = o.update(*joints(dl * i / n, dr * i / n), sim_time=i * 0.1)
    ax = s.x + P.axle_offset_x * math.cos(s.yaw)
    ay = s.y + P.axle_offset_x * math.sin(s.yaw)
    assert math.isclose(ax, P.axle_offset_x + radius, abs_tol=1e-6) and math.isclose(ay, radius, abs_tol=1e-6)
    assert math.isclose(s.yaw, math.pi / 2, abs_tol=1e-9)


def test_wheels_joint_angle_wrap_is_unwrapped():
    o = WheelOdometry(P)
    o.update(math.pi - 0.1, math.pi - 0.1, sim_time=0.0)
    s = o.update(-math.pi + 0.1, -math.pi + 0.1, sim_time=0.1)      # +0.2 rad across the wrap, backwards travel
    assert math.isclose(s.x, 0.2 * P.joint_sign * P.wheel_radius, abs_tol=1e-9)


def pose(x, y, model_yaw):
    qx, qy, qz, qw = quaternion_from_yaw(model_yaw)
    return (x, y, 0.08, qx, qy, qz, qw)


def test_ground_truth_forward_along_model_plus_y():
    o = GroundTruthOdometry(P)
    # the model is loaded with yaw -90 deg, so forward (local +Y) is world +X
    s0 = o.update(pose(3.35, -1.75, -math.pi / 2), 0.0)
    assert (s0.x, s0.y, s0.yaw) == (0.0, 0.0, 0.0)
    s = o.update(pose(3.35 + 0.975, -1.75, -math.pi / 2), 5.0)
    assert math.isclose(s.x, 0.975, abs_tol=1e-9) and abs(s.y) < 1e-9 and abs(s.yaw) < 1e-9
    assert math.isclose(s.vx, 0.195) and abs(s.vy) < 1e-9


def test_ground_truth_counter_clockwise_turn_increases_yaw():
    o = GroundTruthOdometry(P)
    o.update(pose(0, 0, -math.pi / 2), 0.0)
    s = o.update(pose(0, 0, -math.pi / 2 + math.radians(85.4)), 2.0)
    assert math.isclose(math.degrees(s.yaw), 85.4, abs_tol=1e-6)
    assert s.wz > 0


def test_ground_truth_frame_follows_the_first_pose():
    o = GroundTruthOdometry(P)
    o.update(pose(1.0, 1.0, 0.0), 0.0)               # model yaw 0: forward (+Y) is world +Y
    s = o.update(pose(1.0, 3.0, 0.0), 1.0)
    assert math.isclose(s.x, 2.0, abs_tol=1e-9) and abs(s.y) < 1e-9
    s = o.update(pose(-0.5, 3.0, 0.0), 2.0)           # 1.5 m to world -X = robot-left
    assert math.isclose(s.y, 1.5, abs_tol=1e-9)


def test_velocity_is_zero_without_time_progress():
    o = GroundTruthOdometry(P)
    o.update(pose(0, 0, 0), 1.0)
    s = o.update(pose(0.1, 0, 0), 1.0)
    assert s.vx == 0.0
