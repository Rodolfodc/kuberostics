"""`detection_selector` LifecycleNode (T907, spec 009a): thin glue over `SelectorCore`.

All decisions (circuit breaker, age and watermark filters) live in the pure core. This node only converts
messages to (source, stamp), republishes what the core forwards, publishes `OffloadStatus` (R14) and the
notifications, and ticks the core so that cloud silence is detected without a response (R08).
"""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
# RCLError is not re-exported by a public rclpy module on Jazzy; this path works on Jazzy and Lyrical.
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from vacuum_interfaces.msg import Notification, OffloadStatus
from vision_msgs.msg import Detection2DArray

from vacuum_detection.ros_common import (
    QOS_IA368II, build_diagnostics, node_now_sec, stamp_to_sec,
)
from vacuum_detection.selector_core import CLOSED, CLOUD, HALF_OPEN, LOCAL, OPEN, SelectorCore

STATE_CODE = {CLOSED: OffloadStatus.CLOSED, OPEN: OffloadStatus.OPEN, HALF_OPEN: OffloadStatus.HALF_OPEN}
SEVERITY = {'INFO': Notification.INFO, 'WARN': Notification.WARN, 'ERROR': Notification.ERROR}


class DetectionSelectorNode(LifecycleNode):
    def __init__(self, node_name: str = 'detection_selector', **kwargs):
        super().__init__(node_name, **kwargs)
        # Defaults and meaning: specs/000-system/contracts/topics.yaml (parameters: detection_selector).
        self.declare_parameter('max_age_s', 0.67)             # 009a-R05 (= tolerance / v_max)
        self.declare_parameter('open_threshold_s', 1.5)       # 009a-R08
        self.declare_parameter('close_threshold_s', 0.5)      # 009a-R09
        self.declare_parameter('stable_window_s', 3.0)        # 009a-R10
        self.declare_parameter('status_rate_hz', 1.0)         # 009a-R14
        self.declare_parameter('tick_rate_hz', 10.0)          # silence check, 009a-R08
        self.declare_parameter('cloud_topic', '/myRobot/perception/detections/cloud')
        self.declare_parameter('local_topic', '/myRobot/perception/detections/local')
        self.declare_parameter('output_topic', '/myRobot/perception/detections')
        self._core = None
        self._active = False
        self._timers = []

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state) -> TransitionCallbackReturn:
        log = self.get_logger()
        try:
            self._core = SelectorCore(
                clock=lambda: node_now_sec(self), notify=self._notify, logger=log,
                max_age_s=self.get_parameter('max_age_s').value,
                open_threshold_s=self.get_parameter('open_threshold_s').value,
                close_threshold_s=self.get_parameter('close_threshold_s').value,
                stable_window_s=self.get_parameter('stable_window_s').value)
        except ValueError as exc:                             # 009a-R09: reject an invalid configuration
            log.error(f'invalid thresholds: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._pub_out = self.create_lifecycle_publisher(
            Detection2DArray, self.get_parameter('output_topic').value, 10)
        self._pub_status = self.create_lifecycle_publisher(
            OffloadStatus, '/myRobot/perception/offload_status', QOS_IA368II)
        self._pub_notify = self.create_lifecycle_publisher(Notification, '/myRobot/notifications', 10)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/detection_selector', 10)
        self._subs = [
            self.create_subscription(Detection2DArray, self.get_parameter('cloud_topic').value,
                                     lambda m: self._on_detections(CLOUD, m), 10),
            self.create_subscription(Detection2DArray, self.get_parameter('local_topic').value,
                                     lambda m: self._on_detections(LOCAL, m), 10),
        ]
        log.info('configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._active = True
        self._core.restart_grace()               # configure may have happened long before (009a-R08)
        self._timers = [
            self.create_timer(1.0 / self.get_parameter('tick_rate_hz').value, self._core.tick),
            self.create_timer(1.0 / self.get_parameter('status_rate_hz').value, self._publish_status),
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
        for sub in self._subs:
            self.destroy_subscription(sub)
        for pub in (self._pub_out, self._pub_status, self._pub_notify, self._pub_telemetry):
            self.destroy_publisher(pub)
        self._core = None
        return TransitionCallbackReturn.SUCCESS

    # ---------------------------------------------------------------- data flow
    def _on_detections(self, source: str, msg: Detection2DArray) -> None:
        if self._active and self._core.on_response(source, stamp_to_sec(msg.header.stamp)):
            self._pub_out.publish(msg)

    def _notify(self, severity: str, code: str) -> None:
        msg = Notification(severity=SEVERITY[severity], source=self.get_name(), code=code, text=code)
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_notify.publish(msg)

    def _publish_status(self) -> None:
        snap = self._core.snapshot()
        msg = OffloadStatus(
            state=STATE_CODE[snap['state']], last_cloud_age_s=snap['last_cloud_age_s'],
            forwarded=snap['forwarded'], dropped_inactive=snap['dropped_inactive'],
            dropped_stale=snap['dropped_stale'], dropped_watermark=snap['dropped_watermark'],
            transitions=snap['transitions'])
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_status.publish(msg)
        self._pub_telemetry.publish(build_diagnostics(self, snap['state'], snap))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionSelectorNode()
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
