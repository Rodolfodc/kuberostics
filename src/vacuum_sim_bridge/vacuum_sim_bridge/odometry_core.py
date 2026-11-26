"""Odometry integration (pure Python, no ROS). Two interchangeable sources, both returning an OdomState for the
base_link origin in ROS convention (x forward, y left, yaw counter-clockwise), starting at the origin of the odom
frame. Time is the simulation time (the simulator runs far slower than real time)."""
import math
from dataclasses import dataclass
from typing import Optional

from vacuum_sim_bridge.geometry import wrap_angle, yaw_from_quaternion
from vacuum_sim_bridge.profile import RobotProfile


@dataclass(frozen=True)
class OdomState:
    x: float
    y: float
    yaw: float
    vx: float      # body frame, base_link origin
    vy: float
    wz: float
    sim_time: float


class WheelOdometry:
    """Integrates the two wheel joint angles (differential drive, exact arcs). The integrated point is the axle
    centre; the reported pose is the base_link origin, `axle_offset_x` behind it."""

    def __init__(self, profile: RobotProfile):
        self._p = profile
        self._last = None          # (left_angle, right_angle, sim_time)
        self._ax, self._ay, self._yaw = profile.axle_offset_x, 0.0, 0.0   # origin of the odom frame = base_link
        self._vx = self._vy = self._wz = 0.0

    def update(self, left_angle: float, right_angle: float, sim_time: float) -> OdomState:
        p = self._p
        if self._last is not None:
            dl = wrap_angle(left_angle - self._last[0]) * p.joint_sign * p.wheel_radius
            dr = wrap_angle(right_angle - self._last[1]) * p.joint_sign * p.wheel_radius
            dt = sim_time - self._last[2]
            ds = (dl + dr) / 2.0
            dyaw = (dr - dl) / p.wheel_separation
            if abs(dyaw) < 1e-9:
                self._ax += ds * math.cos(self._yaw)
                self._ay += ds * math.sin(self._yaw)
            else:
                radius = ds / dyaw
                self._ax += radius * (math.sin(self._yaw + dyaw) - math.sin(self._yaw))
                self._ay -= radius * (math.cos(self._yaw + dyaw) - math.cos(self._yaw))
            self._yaw = wrap_angle(self._yaw + dyaw)
            if dt > 0.0:
                self._vx = ds / dt
                self._wz = dyaw / dt
                self._vy = -self._wz * p.axle_offset_x   # origin is behind the axle centre
        self._last = (left_angle, right_angle, sim_time)
        d = p.axle_offset_x
        return OdomState(self._ax - d * math.cos(self._yaw), self._ay - d * math.sin(self._yaw), self._yaw,
                         self._vx, self._vy, self._wz, sim_time)


class GroundTruthOdometry:
    """Converts the world pose of the model frame (x, y, z, qx, qy, qz, qw) into base_link odometry relative to
    the first sample. The ROS heading is the model yaw + 90 deg because the model drives along local +Y."""

    def __init__(self, profile: RobotProfile):
        self._p = profile
        self._origin = None        # (x, y, yaw) of the first sample
        self._last: Optional[OdomState] = None

    def update(self, pose, sim_time: float) -> OdomState:
        x, y, _z, qx, qy, qz, qw = pose
        yaw = wrap_angle(yaw_from_quaternion(qx, qy, qz, qw) + math.pi / 2.0)
        if self._origin is None:
            self._origin = (x, y, yaw)
        ox, oy, oyaw = self._origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        px, py = x - ox, y - oy
        lx, ly = c * px + s * py, -s * px + c * py       # into the frame of the first pose
        lyaw = wrap_angle(yaw - oyaw)
        vx = vy = wz = 0.0
        if self._last is not None:
            dt = sim_time - self._last.sim_time
            if dt > 0.0:
                dx, dy = lx - self._last.x, ly - self._last.y
                cy, sy = math.cos(lyaw), math.sin(lyaw)
                vx = (cy * dx + sy * dy) / dt
                vy = (-sy * dx + cy * dy) / dt
                wz = wrap_angle(lyaw - self._last.yaw) / dt
            else:
                vx, vy, wz = self._last.vx, self._last.vy, self._last.wz
        self._last = OdomState(lx, ly, lyaw, vx, vy, wz, sim_time)
        return self._last
