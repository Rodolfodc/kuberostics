"""JPEG encoding of raw image buffers (009a-R03). No ROS; needs numpy and OpenCV (skipped otherwise)."""
import pytest

np = pytest.importorskip('numpy')
cv2 = pytest.importorskip('cv2')

from vacuum_detection.image_codec import check_quality, encode_jpeg   # noqa: E402


def solid(h, w, rgb):
    return np.tile(np.array(rgb, dtype=np.uint8), (h, w, 1))


def decode(jpeg):
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)


def test_009a_R03_rgb8_buffer_becomes_a_decodable_jpeg_with_the_same_size_and_colours():
    img = solid(48, 64, (255, 0, 0))                       # pure red in RGB
    out = decode(encode_jpeg(img.tobytes(), 48, 64, 64 * 3, 'rgb8', quality=90))
    assert out.shape == (48, 64, 3)
    b, g, r = out[24, 32]
    assert r > 200 and g < 60 and b < 60                   # OpenCV decodes BGR: red stays red


def test_009a_R03_bgr8_buffer_keeps_its_colours():
    img = solid(48, 64, (255, 0, 0))                       # pure blue in BGR
    out = decode(encode_jpeg(img.tobytes(), 48, 64, 64 * 3, 'bgr8', quality=90))
    b, g, r = out[24, 32]
    assert b > 200 and g < 60 and r < 60


def test_009a_R03_row_padding_in_step_is_ignored():
    row = np.zeros((4, 64 * 3 + 8), dtype=np.uint8)        # step larger than width * 3
    out = decode(encode_jpeg(row.tobytes(), 4, 64, 64 * 3 + 8, 'rgb8', quality=90))
    assert out.shape == (4, 64, 3)


def test_009a_R03_lower_quality_gives_a_smaller_payload():
    rng = np.random.default_rng(0)
    img = rng.integers(0, 255, (96, 128, 3), dtype=np.uint8)
    big = encode_jpeg(img.tobytes(), 96, 128, 128 * 3, 'rgb8', quality=95)
    small = encode_jpeg(img.tobytes(), 96, 128, 128 * 3, 'rgb8', quality=20)
    assert len(small) < len(big)


def test_009a_R03_unsupported_encoding_is_rejected():
    with pytest.raises(ValueError):
        encode_jpeg(b'\x00' * 16, 2, 2, 8, '32FC1', quality=80)


def test_009a_R03_empty_image_raises_value_error_not_a_cv2_error():
    with pytest.raises(ValueError):
        encode_jpeg(b'', 0, 0, 0, 'rgb8', quality=80)


@pytest.mark.parametrize('quality', [0, 101, -5])
def test_009a_R03_quality_outside_1_to_100_is_rejected(quality):
    with pytest.raises(ValueError):
        check_quality(quality)


@pytest.mark.parametrize('quality', [1, 80, 100])
def test_009a_R03_quality_inside_1_to_100_is_accepted(quality):
    check_quality(quality)
