"""Wheel mapping. Basis: spec 000 D-03/D-06 (cmd_vel is unstamped Twist), robot_report.txt (caps, separation)."""
import math

from vacuum_sim_bridge.profile import RobotProfile
from vacuum_sim_bridge.wheel_core import WheelCommandCore

P = RobotProfile()


def test_forward_and_counter_clockwise_turn():
    c = WheelCommandCore(P)
    c.command(0.2, 0.0, now=0.0)
    assert c.output(0.0) == (0.2, 0.2)
    c.command(0.0, 1.0, now=0.0)
    left, right = c.output(0.0)
    assert math.isclose(left, -0.1) and math.isclose(right, 0.1)      # separation 0.20, CCW: right faster


def test_clamped_to_the_controller_caps():
    c = WheelCommandCore(P)
    c.command(5.0, 0.0, now=0.0)
    assert c.output(0.0) == (0.5, 0.5)
    c.command(0.0, -50.0, now=0.0)
    left, right = c.output(0.0)
    assert math.isclose(right, -P.max_angular * 0.1) and math.isclose(left, P.max_angular * 0.1)


def test_command_expires_and_nothing_is_driven_before_the_first_command():
    c = WheelCommandCore(P)
    assert c.output(0.0) == (0.0, 0.0)
    c.command(0.3, 0.0, now=10.0)
    assert c.output(10.0 + P.cmd_timeout) == (0.3, 0.3)
    assert c.output(10.0 + P.cmd_timeout + 0.01) == (0.0, 0.0)


def test_non_finite_command_stops_the_robot():
    c = WheelCommandCore(P)
    c.command(float('nan'), 0.0, now=0.0)
    assert c.output(0.0) == (0.0, 0.0)
