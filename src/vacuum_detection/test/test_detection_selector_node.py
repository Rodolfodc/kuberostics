"""T907: smoke tests of the detection_selector node glue (the logic is tested in test_selector_core.py).

Needs rclpy (skipped otherwise). In-process, system clock, no launch.
"""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor            # noqa: E402
from rclpy.parameter import Parameter                          # noqa: E402
from vacuum_interfaces.msg import OffloadStatus                # noqa: E402
from vision_msgs.msg import Detection2DArray                   # noqa: E402

from vacuum_detection.detection_selector_node import DetectionSelectorNode   # noqa: E402


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


def activate(node):
    node.trigger_configure()
    node.trigger_activate()


def test_009a_R09_node_rejects_invalid_threshold_ordering_on_configure(ros):
    from rclpy.lifecycle import TransitionCallbackReturn
    node = DetectionSelectorNode(parameter_overrides=[Parameter('close_threshold_s', value=0.9)])
    result = node.on_configure(None)
    assert result == TransitionCallbackReturn.FAILURE


def test_009a_R14_R06_node_forwards_fresh_cloud_response_and_publishes_status(ros):
    node = DetectionSelectorNode(parameter_overrides=[Parameter('status_rate_hz', value=10.0)])
    helper = rclpy.create_node('selector_test_helper')
    out, status = [], []
    helper.create_subscription(Detection2DArray, '/myRobot/perception/detections', out.append, 10)
    helper.create_subscription(OffloadStatus, '/myRobot/perception/offload_status', status.append, 10)
    pub = helper.create_publisher(Detection2DArray, '/myRobot/perception/detections/cloud', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    activate(node)

    msg = Detection2DArray()
    msg.header.stamp = node.get_clock().now().to_msg()
    assert spin_until(executor, lambda: pub.get_subscription_count() > 0)
    pub.publish(msg)
    assert spin_until(executor, lambda: out and status and status[-1].forwarded == 1)
    assert out[0].header.stamp == msg.header.stamp
    assert status[-1].state == OffloadStatus.CLOSED
    node.destroy_node()
    helper.destroy_node()
