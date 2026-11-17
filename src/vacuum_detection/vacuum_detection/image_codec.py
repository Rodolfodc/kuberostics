"""JPEG encoding of a raw `sensor_msgs/Image` buffer (spec 009a-R03). No ROS imports (D-18).

A toolbox function: bytes in, bytes out. It avoids cv_bridge so the same code runs on Jazzy and Lyrical
without a distro-specific binding (Art. X).
"""
import cv2
import numpy as np

_CHANNELS = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}


def encode_jpeg(data: bytes, height: int, width: int, step: int, encoding: str, quality: int) -> bytes:
    """Encodes a raw image buffer as JPEG. `step` is the row length in bytes (rows may be padded)."""
    if encoding not in _CHANNELS:
        raise ValueError(f'unsupported encoding {encoding!r}, expected one of {sorted(_CHANNELS)}')
    channels = _CHANNELS[encoding]
    rows = np.frombuffer(data, dtype=np.uint8, count=height * step).reshape(height, step)
    img = rows[:, :width * channels].reshape(height, width, channels)
    if encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)      # OpenCV writes BGR
    ok, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return jpeg.tobytes()
