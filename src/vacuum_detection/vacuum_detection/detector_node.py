"""`detector` LifecycleNode (T906, spec 009a-R02, R04): thin glue over `DetectorCore` and a `DetectorModel`.

The same executable is the local detector (hot standby, `yolo11n`, raw `/camera`) and the cloud one (`yolo11x`,
compressed throttled image); only the parameters and the node name differ. The output `header` is the input
image header unchanged (the timestamp echo the selector and the projector rely on, R02), and an empty
`Detection2DArray` is published when nothing is detected. If the inference itself fails nothing is published
(silence is what the selector's health check measures; an empty array would pass as "saw nothing").
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
# RCLError is not re-exported by a public rclpy module on Jazzy; this path works on Jazzy and Lyrical.
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from vacuum_detection.detector_core import DetectorCore, NullModel
from vacuum_detection.image_codec import decode_compressed, decode_raw
from vacuum_detection.ros_common import QOS_SENSOR, build_diagnostics
from vacuum_detection.yolo_model import DetectorDependencyError, load_yolo

BACKENDS = ('ultralytics', 'null')
INPUT_TYPES = ('image', 'compressed')


def set_bbox(bbox, cx: float, cy: float, w: float, h: float) -> None:
    """Fills a BoundingBox2D. Its centre is `Pose2D` with `.position` on newer vision_msgs and `.x/.y` on older."""
    center = bbox.center
    target = center.position if hasattr(center, 'position') else center
    target.x, target.y = float(cx), float(cy)
    bbox.size_x, bbox.size_y = float(w), float(h)


def build_array(header, detections) -> Detection2DArray:
    """Detection2DArray whose header (and each Detection2D header) is a copy of the image header (R02)."""
    msg = Detection2DArray()
    msg.header = header
    for d in detections:
        det = Detection2D()
        det.header = header
        set_bbox(det.bbox, d.cx, d.cy, d.w, d.h)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = d.label
        hyp.hypothesis.score = d.score
        det.results.append(hyp)
        msg.detections.append(det)
    return msg


class DetectorNode(LifecycleNode):
    def __init__(self, node_name: str = 'detector', model=None, **kwargs):
        super().__init__(node_name, **kwargs)
        self._injected_model = model             # a DetectorModel handed in by the caller (tests); skips the backend
        # Defaults are those of the local detector. Public parameters: see the README of the package.
        self.declare_parameter('backend', 'ultralytics')      # ultralytics | null (null: detects nothing)
        self.declare_parameter('weights', 'yolo11n.pt')       # yolo11n.pt locally, yolo11x.pt in the cluster
        self.declare_parameter('device', 'cpu')               # cpu, cuda:0, ...
        self.declare_parameter('score_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('max_detections', 100)
        self.declare_parameter('input_type', 'image')         # image (raw /camera) | compressed (cloud)
        self.declare_parameter('input_topic', '/camera')
        self.declare_parameter('output_topic', '/myRobot/perception/detections/local')
        self.declare_parameter('telemetry_rate_hz', 1.0)
        self._core = None
        self._active = False
        self._timer = None
        self._decode_errors = 0

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state) -> TransitionCallbackReturn:
        log = self.get_logger()
        p = self.get_parameter
        backend, input_type = p('backend').value, p('input_type').value
        try:
            if backend not in BACKENDS:
                raise ValueError(f'backend must be one of {BACKENDS}, got {backend!r}')
            if input_type not in INPUT_TYPES:
                raise ValueError(f'input_type must be one of {INPUT_TYPES}, got {input_type!r}')
            if not p('telemetry_rate_hz').value > 0.0:
                raise ValueError('telemetry_rate_hz must be positive')
            if self._injected_model is not None:
                model = self._injected_model
            elif backend == 'null':
                model = NullModel()
            else:
                t0 = time.monotonic()
                model = load_yolo(p('weights').value, p('device').value, p('iou_threshold').value,
                                  int(p('image_size').value), p('score_threshold').value)
                log.info(f"model {p('weights').value} loaded on {p('device').value} "
                         f'in {time.monotonic() - t0:.1f} s')
            self._core = DetectorCore(model, clock=time.monotonic, score_threshold=p('score_threshold').value,
                                      max_detections=int(p('max_detections').value), logger=log)
        except (ValueError, DetectorDependencyError) as exc:
            log.error(f'invalid configuration or model: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._input_type = input_type
        self._pub = self.create_lifecycle_publisher(Detection2DArray, p('output_topic').value, 10)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, f'/myRobot/telemetry/{self.get_name()}', 10)
        msg_type = Image if input_type == 'image' else CompressedImage
        self._sub = self.create_subscription(msg_type, p('input_topic').value, self._on_image, QOS_SENSOR)
        log.info(f'configured: backend={backend} input={input_type}:{p("input_topic").value} '
                 f'output={p("output_topic").value}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._active = True
        self._timer = self.create_timer(1.0 / self.get_parameter('telemetry_rate_hz').value, self._telemetry)
        self.get_logger().info('activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        super().on_deactivate(state)
        self.get_logger().info('deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        if self._timer is not None:               # normally gone since deactivate; cleanup must not leak it
            self.destroy_timer(self._timer)
            self._timer = None
        self.destroy_subscription(self._sub)
        self.destroy_publisher(self._pub)
        self.destroy_publisher(self._pub_telemetry)
        self._core = None
        return TransitionCallbackReturn.SUCCESS

    # ---------------------------------------------------------------- data flow
    def _on_image(self, msg) -> None:
        if not self._active:
            return
        try:
            if self._input_type == 'image':
                image = decode_raw(bytes(msg.data), msg.height, msg.width, msg.step, msg.encoding)
            else:
                image = decode_compressed(bytes(msg.data))
        except ValueError as exc:
            self._decode_errors += 1
            self.get_logger().warning(f'dropping an undecodable image: {exc}')
            return
        detections = self._core.process(image)
        if detections is not None:                # None: inference failed, stay silent (see module doc)
            self._pub.publish(build_array(msg.header, detections))

    def _telemetry(self) -> None:
        snap = self._core.snapshot()
        snap['decode_errors'] = self._decode_errors
        self._pub_telemetry.publish(build_diagnostics(self, f"{snap['processed']} processed", snap))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()
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
