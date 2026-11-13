"""Projection of a detection to the map frame (spec 009a-R12). Pure Python, no ROS (D-18).

The core never reads a clock: the transform is requested at the detection stamp, so the arrival instant
cannot be used by mistake. `lookup_tf(stamp, timeout_s)` is injected by the node (a tf2 buffer wrapper) and
returns a `Transform`, or None when it does not exist.
"""
import logging
import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class Transform:
    translation: Vec3
    rotation: Tuple[float, float, float, float]   # quaternion x, y, z, w

    def apply(self, p: Vec3) -> Vec3:
        qx, qy, qz, qw = self.rotation
        px, py, pz = p
        # v' = v + w*t + q x t, with t = 2 (q x v)
        tx = 2.0 * (qy * pz - qz * py)
        ty = 2.0 * (qz * px - qx * pz)
        tz = 2.0 * (qx * py - qy * px)
        rx = px + qw * tx + (qy * tz - qz * ty)
        ry = py + qw * ty + (qz * tx - qx * tz)
        rz = pz + qw * tz + (qx * ty - qy * tx)
        t = self.translation
        return (rx + t[0], ry + t[1], rz + t[2])


def pixel_to_camera(u: float, v: float, depth: float, fx: float, fy: float, cx: float, cy: float) -> Vec3:
    """Pinhole back-projection (optical frame: x right, y down, z forward)."""
    return ((u - cx) * depth / fx, (v - cy) * depth / fy, depth)


class ProjectorCore:
    def __init__(self, lookup_tf: Callable[[float, float], Optional[Transform]],
                 fx: float, fy: float, cx: float, cy: float,
                 tf_timeout_s: float = 0.1, logger: Optional[Any] = None):
        self._lookup = lookup_tf
        self._k = (fx, fy, cx, cy)
        self.tf_timeout_s = tf_timeout_s
        self._log = logger if logger is not None else logging.getLogger(__name__)
        self._m = {'projected': 0, 'dropped_tf': 0, 'dropped_depth': 0}

    def snapshot(self) -> Dict[str, Any]:
        return dict(self._m)

    def project(self, stamp: float, u: float, v: float, depth: float) -> Optional[Vec3]:
        """Point in the target frame, or None if the detection was dropped."""
        if not (math.isfinite(depth) and depth > 0.0):
            self._m['dropped_depth'] += 1
            return None
        tf = self._lookup(stamp, self.tf_timeout_s)
        if tf is None:
            self._m['dropped_tf'] += 1                              # 009a-R12
            self._log.warning(f'no TF at stamp {stamp:.3f}: detection dropped')
            return None
        self._m['projected'] += 1
        return tf.apply(pixel_to_camera(u, v, depth, *self._k))
