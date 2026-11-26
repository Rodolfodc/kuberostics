"""T906: raw and compressed image decoding used by the detector (RGB order matters for YOLO)."""
import pytest

np = pytest.importorskip('numpy')
pytest.importorskip('cv2')

from vacuum_detection.image_codec import decode_compressed, decode_raw, encode_jpeg   # noqa: E402


def test_009a_R02_decode_raw_rgb_and_bgr_give_rgb():
    px = bytes([255, 0, 0])                                    # one red pixel in rgb8
    assert decode_raw(px, 1, 1, 3, 'rgb8')[0, 0].tolist() == [255, 0, 0]
    assert decode_raw(bytes([0, 0, 255]), 1, 1, 3, 'bgr8')[0, 0].tolist() == [255, 0, 0]


def test_009a_R02_decode_raw_honours_row_padding_and_mono():
    data = bytes([1, 2, 3, 99, 4, 5, 6, 99])                    # 2 rows, width 3, step 4 (1 padding byte)
    img = decode_raw(data, 2, 3, 4, 'mono8')
    assert img.shape == (2, 3, 3) and img[1, 2].tolist() == [6, 6, 6]


def test_009a_R02_decode_raw_rejects_bad_input():
    with pytest.raises(ValueError):
        decode_raw(bytes(3), 4, 4, 12, 'rgb8')
    with pytest.raises(ValueError):
        decode_raw(bytes(48), 4, 4, 12, 'rgba8')


def test_009a_R02_decode_compressed_roundtrip_and_error():
    jpeg = encode_jpeg(bytes([200, 10, 10] * 64), 8, 8, 24, 'rgb8', 95)
    img = decode_compressed(jpeg)
    assert img.shape == (8, 8, 3) and img[0, 0, 0] > 150 > img[0, 0, 2]     # still red, in RGB order
    with pytest.raises(ValueError):
        decode_compressed(b'not a jpeg')
