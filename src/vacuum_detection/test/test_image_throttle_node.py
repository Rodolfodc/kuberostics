"""T909: smoke test of the image_throttle node glue (rate logic: test_throttle_core.py, JPEG: test_image_codec.py).

Needs rclpy, numpy and OpenCV (skipped otherwise). In-process, no launch.
"""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cv2')
from rclpy.executors import SingleThreadedExecutor            # noqa: E402
from rclpy.parameter import Parameter                          # noqa: E402
from rclpy.qos import qos_profile_sensor_data                  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image             # noqa: E402

from vacuum_detection.image_throttle_node import ImageThrottleNode   # noqa: E402


@pytest.fixture()
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_until(executor, predicate, timeout_s=5.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end and not predicate():
        executor.spin_once(timeout_sec=0.05)
    return predicate()


def test_009a_R03_node_rejects_non_positive_rate_on_configure(ros):
    from rclpy.lifecycle import TransitionCallbackReturn
    node = ImageThrottleNode(parameter_overrides=[Parameter('image_rate_hz', value=0.0)])
    assert node.on_configure(None) == TransitionCallbackReturn.FAILURE


def test_009a_R03_node_sends_one_jpeg_with_the_image_header_and_throttles_the_rest(ros):
    node = ImageThrottleNode(parameter_overrides=[Parameter('image_rate_hz', value=2.0)])
    helper = rclpy.create_node('throttle_test_helper')
    out = []
    helper.create_subscription(CompressedImage, '/myRobot/perception/rgb_throttled/compressed', out.append,
                               qos_profile_sensor_data)
    pub = helper.create_publisher(Image, '/camera', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    node.trigger_configure()
    node.trigger_activate()
    assert spin_until(executor, lambda: pub.get_subscription_count() > 0)

    def frame(sec, nanosec):
        msg = Image(height=4, width=4, encoding='rgb8', step=12, data=bytes(48))
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
        msg.header.frame_id = 'camera'
        return msg

    pub.publish(frame(10, 0))
    pub.publish(frame(10, 100_000_000))                   # 0.1 s later: throttled
    assert spin_until(executor, lambda: len(out) >= 1)
    spin_until(executor, lambda: False, timeout_s=0.5)    # give a wrongly forwarded second frame time to arrive
    assert len(out) == 1
    assert out[0].format == 'jpeg'
    assert (out[0].header.stamp.sec, out[0].header.frame_id) == (10, 'camera')
    node.destroy_node()
    helper.destroy_node()
