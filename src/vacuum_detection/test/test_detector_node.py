"""T906: smoke tests of the detector node glue with an injected fake model (the logic: test_detector_core.py).

Needs rclpy, numpy and OpenCV (skipped otherwise). In-process, no launch, no weights.
"""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cv2')
from diagnostic_msgs.msg import DiagnosticArray                # noqa: E402
from rclpy.executors import SingleThreadedExecutor            # noqa: E402
from rclpy.lifecycle import TransitionCallbackReturn          # noqa: E402
from rclpy.parameter import Parameter                          # noqa: E402
from rclpy.qos import qos_profile_sensor_data                  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image             # noqa: E402
from vision_msgs.msg import Detection2DArray                   # noqa: E402

from vacuum_detection.detector_core import Detection          # noqa: E402
from vacuum_detection.detector_node import DetectorNode       # noqa: E402
from vacuum_detection.image_codec import encode_jpeg          # noqa: E402


class FakeModel:
    def __init__(self, result=(), error=None):
        self.result, self.error, self.shapes = list(result), error, []

    def predict(self, image):
        self.shapes.append(image.shape)
        if self.error:
            raise self.error
        return self.result


@pytest.fixture()
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_until(executor, predicate, timeout_s=10.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end and not predicate():
        executor.spin_once(timeout_sec=0.05)
    return predicate()


def frame(sec=10, nanosec=123):
    msg = Image(height=4, width=6, encoding='rgb8', step=18, data=bytes(72))
    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
    msg.header.frame_id = 'camera'
    return msg


def setup_graph(node, in_type=Image, in_topic='/camera', out_topic='/myRobot/perception/detections/local'):
    helper = rclpy.create_node('detector_test_helper')
    out = []
    helper.create_subscription(Detection2DArray, out_topic, out.append, 10)
    pub = helper.create_publisher(in_type, in_topic, qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    node.trigger_configure()
    node.trigger_activate()
    assert spin_until(executor, lambda: pub.get_subscription_count() > 0)
    # the output subscription must be matched too, or an early publish is lost
    assert spin_until(executor, lambda: node._pub.get_subscription_count() > 0)
    return helper, executor, pub, out


def test_009a_R02_invalid_backend_fails_configure(ros):
    node = DetectorNode(parameter_overrides=[Parameter('backend', value='nope')])
    assert node.on_configure(None) == TransitionCallbackReturn.FAILURE


def test_009a_R02_missing_dependency_fails_configure_with_no_crash(ros):
    try:
        import ultralytics  # noqa: F401
        pytest.skip('ultralytics is installed here')
    except ImportError:
        pass
    node = DetectorNode()                                     # backend ultralytics by default
    assert node.on_configure(None) == TransitionCallbackReturn.FAILURE


def test_009a_R02_publishes_detections_with_the_image_header(ros):
    model = FakeModel([Detection('cup', 0.9, 3.0, 2.0, 2.0, 1.0)])
    node = DetectorNode(model=model)
    helper, executor, pub, out = setup_graph(node)
    pub.publish(frame(10, 123))
    assert spin_until(executor, lambda: len(out) == 1)
    msg = out[0]
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id) == (10, 123, 'camera')
    assert len(msg.detections) == 1
    assert msg.detections[0].results[0].hypothesis.class_id == 'cup'
    assert (msg.detections[0].header.stamp.sec, msg.detections[0].header.stamp.nanosec) == (10, 123)
    assert model.shapes == [(4, 6, 3)]


def test_009a_R02_publishes_an_empty_array_when_nothing_is_detected(ros):
    node = DetectorNode(model=FakeModel([]))
    helper, executor, pub, out = setup_graph(node)
    pub.publish(frame(11, 7))
    assert spin_until(executor, lambda: len(out) == 1)
    assert len(out[0].detections) == 0 and out[0].header.stamp.sec == 11


def test_009a_R02_inference_failure_publishes_nothing(ros):
    model = FakeModel(error=RuntimeError('boom'))
    node = DetectorNode(model=model)
    helper, executor, pub, out = setup_graph(node)
    pub.publish(frame())
    assert spin_until(executor, lambda: len(model.shapes) == 1)
    spin_until(executor, lambda: False, timeout_s=0.3)
    assert out == []


def test_009a_R02_compressed_input_is_decoded_and_the_header_echoed(ros):
    node = DetectorNode(model=FakeModel([]), parameter_overrides=[
        Parameter('input_type', value='compressed'),
        Parameter('input_topic', value='/myRobot/perception/rgb_throttled/compressed'),
        Parameter('output_topic', value='/myRobot/perception/detections/cloud')])
    helper, executor, pub, out = setup_graph(
        node, CompressedImage, '/myRobot/perception/rgb_throttled/compressed',
        '/myRobot/perception/detections/cloud')
    msg = CompressedImage(format='jpeg', data=encode_jpeg(bytes(72), 4, 6, 18, 'rgb8', 80))
    msg.header.stamp.sec, msg.header.stamp.nanosec = 20, 5
    pub.publish(msg)
    assert spin_until(executor, lambda: len(out) == 1)
    assert (out[0].header.stamp.sec, out[0].header.stamp.nanosec) == (20, 5)


def test_009a_R02_telemetry_carries_the_inference_time(ros):
    node = DetectorNode(model=FakeModel([]), parameter_overrides=[Parameter('telemetry_rate_hz', value=20.0)])
    helper, executor, pub, out = setup_graph(node)
    diag = []
    helper.create_subscription(DiagnosticArray, '/myRobot/telemetry/detector', diag.append, 10)
    assert spin_until(executor, lambda: node._pub_telemetry.get_subscription_count() > 0)
    pub.publish(frame())
    assert spin_until(executor, lambda: any(
        {kv.key for kv in d.status[0].values} >= {'last_inference_ms', 'processed'} and
        dict((kv.key, kv.value) for kv in d.status[0].values)['processed'] == '1' for d in diag))


def test_009a_R02_full_lifecycle_cycle_leaves_nothing_behind(ros):
    node = DetectorNode(model=FakeModel([]))
    assert node.trigger_configure().label == 'inactive'
    assert node.trigger_activate().label == 'active'
    assert node.trigger_deactivate().label == 'inactive'
    assert node.trigger_cleanup().label == 'unconfigured'
    assert node._timer is None and node._core is None
    assert node.trigger_configure().label == 'inactive'          # and it can be configured again
