"""JPEG encoding of a raw `sensor_msgs/Image` buffer (spec 009a-R03). No ROS imports (D-18).

A toolbox function: bytes in, bytes out. It avoids cv_bridge so the same code runs on Jazzy and Lyrical
without a distro-specific binding (Art. X).
"""
import cv2
import numpy as np

_CHANNELS = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}


def check_quality(quality: int) -> None:
    """Raises ValueError unless 1 <= quality <= 100 (OpenCV would silently clamp or misbehave)."""
    if not 1 <= quality <= 100:
        raise ValueError(f'jpeg quality must be in 1..100, got {quality}')


def encode_jpeg(data: bytes, height: int, width: int, step: int, encoding: str, quality: int) -> bytes:
    """Encodes a raw image buffer as JPEG. `step` is the row length in bytes (rows may be padded).

    Every failure is a ValueError (including OpenCV's own), so callers handle one exception type.
    """
    if encoding not in _CHANNELS:
        raise ValueError(f'unsupported encoding {encoding!r}, expected one of {sorted(_CHANNELS)}')
    channels = _CHANNELS[encoding]
    try:
        rows = np.frombuffer(data, dtype=np.uint8, count=height * step).reshape(height, step)
        img = rows[:, :width * channels].reshape(height, width, channels)
        if encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)      # OpenCV writes BGR
        ok, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    except cv2.error as exc:
        raise ValueError(f'OpenCV failed: {exc}') from exc
    if not ok:
        raise ValueError('JPEG encoding failed')
    return jpeg.tobytes()
