"""Vision buffers of the model -> ROS image arrays (numpy only, no ROS)."""
import numpy as np

from vacuum_sim_bridge.profile import RobotProfile


def _orient(arr: np.ndarray, flip_h: bool, flip_v: bool) -> np.ndarray:
    if flip_h:
        arr = arr[:, ::-1]
    if flip_v:
        arr = arr[::-1]
    return np.ascontiguousarray(arr)


def convert_rgb(buf: bytes, width: int, height: int, profile: RobotProfile):
    """Raw rgb buffer -> (height, width, 3) uint8, or None when the size does not match."""
    arr = np.frombuffer(buf, dtype=np.uint8)
    if width <= 0 or height <= 0 or arr.size != width * height * 3:
        return None
    return _orient(arr.reshape(height, width, 3), profile.camera_flip_horizontal, profile.camera_flip_vertical)


def convert_depth(buf: bytes, width: int, height: int, profile: RobotProfile):
    """Normalised depth (float32, 0..1 of [near, far]) -> (height, width) float32 metres. A value at or beyond
    the far plane means no return and becomes NaN. Same orientation as the RGB image."""
    arr = np.frombuffer(buf, dtype=np.float32)
    if width <= 0 or height <= 0 or arr.size != width * height:
        return None
    near, far = profile.camera_near, profile.camera_far
    metres = near + arr.astype(np.float32) * np.float32(far - near)
    metres = np.where(arr >= 0.999, np.float32('nan'), metres).astype(np.float32)
    return _orient(metres.reshape(height, width), profile.camera_flip_horizontal, profile.camera_flip_vertical)
