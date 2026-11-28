"""`projector` LifecycleNode (spec 009a-R12, R13, D-16): thin glue over the pure projector classes.

Detections (2D, from the selector) + depth cache + TF at the photo stamp -> persistent set -> the WHOLE set
republished on `/detections_markers` every `republish_period_s` for the unchanged 2025S2 semantic node.
Decisions live in `ProjectionPipeline`, `DetectionFilter`, `DetectionSet`, `marker_spec` (all pure).
"""
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
# RCLError is not re-exported by a public rclpy module on Jazzy; this path works on Jazzy and Lyrical.
from rclpy._rclpy_pybind11 import RCLError
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

from vacuum_detection.depth_sampling import depth_at
from vacuum_detection.detection_filter import DetectionFilter, GridArea
from vacuum_detection.detection_set import DetectionSet
from vacuum_detection.marker_spec import FRAME, MarkerSpec
from vacuum_detection.projection_core import ProjectorCore, Transform
from vacuum_detection.projector_pipeline import ProjectionPipeline
from vacuum_detection.ros_common import QOS_SENSOR, build_diagnostics, stamp_to_sec
from vacuum_detection.stamp_cache import StampCache


def depth_image_to_meters(msg: Image) -> np.ndarray:
    """32FC1 (metres) or 16UC1 (millimetres) to a float32 array in metres; ValueError otherwise."""
    if msg.encoding == '32FC1':
        dtype, scale = np.float32, 1.0
    elif msg.encoding == '16UC1':
        dtype, scale = np.uint16, 0.001
    else:
        raise ValueError(f'unsupported depth encoding {msg.encoding!r}')
    item = np.dtype(dtype).itemsize
    arr = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.step // item)[:, :msg.width]
    return arr.astype(np.float32) * scale


def _bbox_center(det):
    """Pixel centre of a vision_msgs bounding box (`center.position` on Jazzy; `center` itself in older msgs)."""
    c = det.bbox.center
    c = getattr(c, 'position', c)
    return float(c.x), float(c.y)


def marker_from_spec(spec: MarkerSpec) -> Marker:
    m = Marker()
    m.header.frame_id = spec.frame_id
    m.header.stamp.sec = int(spec.stamp)
    m.header.stamp.nanosec = int((spec.stamp - int(spec.stamp)) * 1e9)
    m.ns, m.id, m.type, m.action = spec.ns, spec.id, spec.type, spec.action
    m.pose.position.x, m.pose.position.y, m.pose.position.z = spec.x, spec.y, spec.z
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = spec.scale
    m.color.r, m.color.g, m.color.b, m.color.a = spec.color
    m.text = spec.text
    return m


class ProjectorNode(LifecycleNode):
    def __init__(self, node_name: str = 'projector', **kwargs):
        super().__init__(node_name, **kwargs)
        # Defaults and meaning: specs/000-system/contracts/topics.yaml (parameters: projector).
        self.declare_parameter('detections_topic', '/myRobot/perception/detections')
        self.declare_parameter('depth_topic', '/camera_depth')
        self.declare_parameter('map_topic', '/topologic_map')
        self.declare_parameter('output_topic', '/detections_markers')
        self.declare_parameter('target_frame', FRAME)
        self.declare_parameter('camera_frame', '')              # empty: the detection header frame_id
        self.declare_parameter('fx', 589.4)       # model camera_fx (vacuum_sim_bridge profile)
        self.declare_parameter('fy', 589.4)
        self.declare_parameter('cx', 319.5)
        self.declare_parameter('cy', 239.5)
        self.declare_parameter('tf_timeout_s', 0.1)             # 009a-R12
        self.declare_parameter('republish_period_s', 2.0)       # 009a-R13
        self.declare_parameter('min_confidence', 0.7)
        self.declare_parameter('allowed_labels', [''])          # [''] or empty strings only: every label
        self.declare_parameter('check_mapped_area', True)
        self.declare_parameter('merge_distance_m', 0.5)
        self.declare_parameter('depth_cache_size', 8)
        self.declare_parameter('depth_cache_max_age_s', 2.0)
        self.declare_parameter('depth_tolerance_s', 0.05)
        self.declare_parameter('depth_window_px', 2)
        self.declare_parameter('telemetry_rate_hz', 1.0)        # D-20
        self._pipeline = None
        self._area = None
        self._frame = ''
        self._active = False
        self._timers = []
        self.tf_buffer = None

    def on_configure(self, state) -> TransitionCallbackReturn:
        log = self.get_logger()
        p = lambda name: self.get_parameter(name).value          # noqa: E731
        try:
            self._depths = StampCache(int(p('depth_cache_size')), float(p('depth_cache_max_age_s')))
            allowed = [s for s in p('allowed_labels') if s.strip()]
            check_area = bool(p('check_mapped_area'))
            policy = DetectionFilter(float(p('min_confidence')), allowed,
                                     self._is_mapped if check_area else None)
            core = ProjectorCore(self._lookup_tf, p('fx'), p('fy'), p('cx'), p('cy'),
                                 float(p('tf_timeout_s')), logger=log)
            self._pipeline = ProjectionPipeline(core, policy, DetectionSet(float(p('merge_distance_m'))), log)
            if float(p('republish_period_s')) <= 0.0 or float(p('telemetry_rate_hz')) <= 0.0:
                raise ValueError('republish_period_s and telemetry_rate_hz must be > 0')
        except ValueError as exc:
            log.error(f'invalid configuration: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._target = p('target_frame')
        self._tolerance = float(p('depth_tolerance_s'))
        self._window = int(p('depth_window_px'))
        self.tf_buffer = Buffer()
        # node=None: the listener owns a private node and spins it in its own thread. Passing `self` with
        # spin_thread=True put the lifecycle node in two executors (the listener's and rclpy.spin in main), and
        # on_activate, which creates timers, then hung with the lifecycle services unanswered.
        self._tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)
        self._pub = self.create_lifecycle_publisher(MarkerArray, p('output_topic'), 10)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/projector', 10)
        self._subs = [
            self.create_subscription(Detection2DArray, p('detections_topic'), self._on_detections, 10),
            self.create_subscription(Image, p('depth_topic'), self._on_depth, QOS_SENSOR),
            self.create_subscription(OccupancyGrid, p('map_topic'), self._on_map, 10),
        ]
        log.info('configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._active = True
        self._timers = [
            self.create_timer(float(self.get_parameter('republish_period_s').value), self._republish),
            self.create_timer(1.0 / float(self.get_parameter('telemetry_rate_hz').value), self._telemetry),
        ]
        self.get_logger().info('activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False
        for t in self._timers:
            self.destroy_timer(t)
        self._timers = []
        super().on_deactivate(state)
        self.get_logger().info('deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        for s in self._subs:
            self.destroy_subscription(s)
        self.destroy_publisher(self._pub)
        self.destroy_publisher(self._pub_telemetry)
        self._stop_tf_listener()
        self.tf_buffer = None
        self._pipeline = None
        return TransitionCallbackReturn.SUCCESS

    def _stop_tf_listener(self) -> None:
        listener, self._tf_listener = getattr(self, '_tf_listener', None), None
        if listener is None:
            return
        listener.executor.shutdown()
        listener.dedicated_listener_thread.join(timeout=5.0)
        listener.unregister()
        listener.node.destroy_node()

    # --- inputs -------------------------------------------------------------------------------
    def _is_mapped(self, x: float, y: float) -> bool:
        return self._area is not None and self._area(x, y)

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        try:
            self._area = GridArea(info.origin.position.x, info.origin.position.y, info.resolution,
                                  info.width, info.height, np.asarray(msg.data, dtype=np.int16))
        except ValueError as exc:
            self.get_logger().warning(f'map ignored: {exc}')

    def _on_depth(self, msg: Image) -> None:
        try:
            self._depths.put(stamp_to_sec(msg.header.stamp), depth_image_to_meters(msg))
        except ValueError as exc:
            self.get_logger().warning(f'depth image ignored: {exc}', throttle_duration_sec=5.0)

    def _lookup_tf(self, stamp: float, timeout_s: float):
        """tf2 wrapped for ProjectorCore: transform camera frame -> target frame AT the detection stamp."""
        override = self.get_parameter('camera_frame').value
        source = override or self._frame
        try:
            ts = self.tf_buffer.lookup_transform(
                self._target, source, Time(seconds=int(stamp), nanoseconds=int((stamp % 1.0) * 1e9)),
                timeout=Duration(seconds=timeout_s))
        except TransformException:
            return None
        t, q = ts.transform.translation, ts.transform.rotation
        return Transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def _on_detections(self, msg: Detection2DArray) -> None:
        if not self._active:
            return
        stamp = stamp_to_sec(msg.header.stamp)
        self._frame = msg.header.frame_id
        depth_img = self._depths.get(stamp, self._tolerance)
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            u, v = _bbox_center(det)
            depth = float('nan') if depth_img is None else depth_at(depth_img, u, v, self._window)
            self._pipeline.process(stamp, hyp.class_id, float(hyp.score), u, v, depth)

    # --- outputs ------------------------------------------------------------------------------
    def _republish(self) -> None:
        specs = self._pipeline.markers()
        if specs:
            self._pub.publish(MarkerArray(markers=[marker_from_spec(s) for s in specs]))

    def _telemetry(self) -> None:
        self._pub_telemetry.publish(build_diagnostics(self, 'projecting', self._pipeline.snapshot()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProjectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except RCLError:                         # the context was already invalidated by SIGINT
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
