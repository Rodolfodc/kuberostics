"""Smoke test of the projector node glue (decisions: test_projector_markers.py, test_projector_core.py).

Needs rclpy, numpy, tf2_ros (skipped otherwise). In-process, no launch. The TF buffer of the node is filled
directly, so no broadcaster is needed.
"""
import struct
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
from diagnostic_msgs.msg import DiagnosticArray                # noqa: E402
from geometry_msgs.msg import TransformStamped                  # noqa: E402
from nav_msgs.msg import OccupancyGrid                          # noqa: E402
from rclpy.executors import SingleThreadedExecutor              # noqa: E402
from rclpy.parameter import Parameter                           # noqa: E402
from sensor_msgs.msg import Image                               # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray          # noqa: E402
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose   # noqa: E402

from vacuum_detection import marker_spec                                              # noqa: E402
from vacuum_detection.projector_node import ProjectorNode                              # noqa: E402


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


def test_009a_R13_marker_constants_match_ros():
    assert (marker_spec.CUBE, marker_spec.TEXT_VIEW_FACING, marker_spec.ADD) == (
        Marker.CUBE, Marker.TEXT_VIEW_FACING, Marker.ADD)


def test_009a_R13_node_rejects_invalid_period_on_configure(ros):
    from rclpy.lifecycle import TransitionCallbackReturn
    node = ProjectorNode(parameter_overrides=[Parameter('republish_period_s', value=0.0)])
    assert node.on_configure(None) == TransitionCallbackReturn.FAILURE


def _detection(label, score, u, v, stamp_sec):
    msg = Detection2DArray()
    msg.header.stamp.sec = stamp_sec
    msg.header.frame_id = 'camera'
    det = Detection2D()
    det.bbox.center.position.x, det.bbox.center.position.y = float(u), float(v)
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id, hyp.hypothesis.score = label, score
    det.results = [hyp]
    msg.detections = [det]
    return msg


def test_009a_R12_R13_node_projects_at_photo_stamp_and_republishes_the_whole_set(ros):
    node = ProjectorNode(parameter_overrides=[
        Parameter('republish_period_s', value=0.2), Parameter('telemetry_rate_hz', value=5.0),
        Parameter('fx', value=100.0), Parameter('fy', value=100.0),
        Parameter('cx', value=50.0), Parameter('cy', value=50.0)])
    helper = rclpy.create_node('projector_test_helper')
    markers, diags = [], []
    helper.create_subscription(MarkerArray, '/detections_markers', markers.append, 10)
    helper.create_subscription(DiagnosticArray, '/myRobot/telemetry/projector', diags.append, 10)
    pub_det = helper.create_publisher(Detection2DArray, '/myRobot/perception/detections', 10)
    pub_depth = helper.create_publisher(Image, '/camera_depth', 10)
    pub_map = helper.create_publisher(OccupancyGrid, '/topologic_map', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    node.trigger_configure()
    node.trigger_activate()

    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'map', 'camera'
    tf.header.stamp.sec = 10
    tf.transform.translation.x, tf.transform.translation.y = 2.0, 3.0
    tf.transform.rotation.w = 1.0
    node.tf_buffer.set_transform(tf, 'test')

    grid = OccupancyGrid()
    grid.info.resolution, grid.info.width, grid.info.height = 1.0, 10, 10
    grid.data = [3] * 100
    depth = Image(height=100, width=100, encoding='32FC1', step=400, data=struct.pack('<f', 1.0) * 10000)
    depth.header.stamp.sec = 10

    assert spin_until(executor, lambda: pub_det.get_subscription_count() > 0
                      and pub_depth.get_subscription_count() > 0 and pub_map.get_subscription_count() > 0)
    pub_map.publish(grid)
    pub_depth.publish(depth)
    spin_until(executor, lambda: False, timeout_s=0.3)
    pub_det.publish(_detection('Cup', 0.9, 50, 50, 10))
    pub_det.publish(_detection('cup', 0.3, 50, 50, 10))           # below min_confidence

    assert spin_until(executor, lambda: len(markers) >= 2)         # republished at least twice
    texts = [m for m in markers[-1].markers if m.ns == 'detections_text']
    assert len(markers[-1].markers) == 2 and len(texts) == 1
    t = texts[0]
    assert (t.header.frame_id, t.text.split('(')[0].strip().lower()) == ('map', 'cup')
    assert (t.pose.position.x, t.pose.position.y) == pytest.approx((2.0, 3.0))
    assert spin_until(executor, lambda: any(
        {kv.key: kv.value for kv in d.status[0].values}.get('dropped_confidence') == '1' for d in diags))
    node.destroy_node()
    helper.destroy_node()
