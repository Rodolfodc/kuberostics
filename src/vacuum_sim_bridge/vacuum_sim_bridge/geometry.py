"""Small pose helpers (pure Python)."""
import math


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quaternion_from_yaw(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
