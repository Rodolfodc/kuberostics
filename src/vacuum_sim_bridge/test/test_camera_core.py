"""Camera buffers. Basis: spec 000 section 4 (frozen /camera, /camera_depth), robot_report.txt section 1.
Requirement: 001-R23."""
import math
from dataclasses import replace

import numpy as np

from vacuum_sim_bridge.camera_core import convert_depth, convert_rgb
from vacuum_sim_bridge.profile import RobotProfile

P = RobotProfile()


def test_rgb_default_flips_rows_only():
    """The Remote API returns rows bottom to top (verified live), so the default flips vertically only."""
    img = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    out = convert_rgb(img.tobytes(), 3, 2, P)
    assert np.array_equal(out, img[::-1])


def test_rgb_flips_are_parameters():
    img = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    out = convert_rgb(img.tobytes(), 3, 2, replace(P, camera_flip_vertical=False, camera_flip_horizontal=True))
    assert np.array_equal(out, img[:, ::-1])


def test_rgb_size_mismatch_returns_none():
    assert convert_rgb(b'\x00' * 10, 3, 2, P) is None


def test_depth_normalised_to_metres_and_far_plane_is_nan():
    norm = np.array([[0.0, 0.5, 1.0], [0.25, 0.75, 0.999]], dtype=np.float32)
    out = convert_depth(norm.tobytes(), 3, 2, replace(P, camera_flip_vertical=False))
    assert out.dtype == np.float32
    assert math.isclose(out[0, 0], 0.01, abs_tol=1e-6)
    assert math.isclose(out[0, 1], 0.01 + 0.5 * 3.49, abs_tol=1e-5)
    assert math.isnan(out[0, 2]) and math.isnan(out[1, 2])
    assert math.isclose(out[1, 0], 0.01 + 0.25 * 3.49, abs_tol=1e-5)


def test_depth_and_rgb_share_the_orientation():
    norm = np.array([[0.1, 0.2], [0.3, 0.4]], dtype=np.float32)
    out = convert_depth(norm.tobytes(), 2, 2, P)
    assert out[0, 0] > out[1, 0]                      # rows reversed like the rgb image


def test_depth_size_mismatch_returns_none():
    assert convert_depth(b'\x00' * 8, 3, 2, P) is None
