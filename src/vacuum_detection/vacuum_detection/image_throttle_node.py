"""`image_throttle` LifecycleNode (T909, spec 009a-R03, R11): thin glue over `ThrottleCore` and `encode_jpeg`.

RGB image in, JPEG `CompressedImage` out at `image_rate_hz`, header copied (it is the timestamp echo the selector
and the projector rely on). Depth never goes through here. The rate does not depend on the breaker state (R11).
"""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
# RCLError is not re-exported by a public rclpy module on Jazzy; this path works on Jazzy and Lyrical.
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import CompressedImage, Image

from vacuum_detection.image_codec import check_quality, encode_jpeg
from vacuum_detection.ros_common import QOS_SENSOR, build_diagnostics, node_now_sec, stamp_to_sec
from vacuum_detection.throttle_core import ThrottleCore


class ImageThrottleNode(LifecycleNode):
    def __init__(self, node_name: str = 'image_throttle', **kwargs):
        super().__init__(node_name, **kwargs)
        # Defaults and meaning: specs/000-system/contracts/topics.yaml (parameters: image_throttle).
        self.declare_parameter('image_rate_hz', 2.0)          # 009a-R03
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('telemetry_rate_hz', 1.0)      # D-20
        self.declare_parameter('input_topic', '/camera')
        self.declare_parameter('output_topic', '/myRobot/perception/rgb_throttled/compressed')
        self._core = None
        self._active = False
        self._encode_errors = 0
        self._timer = None

    def on_configure(self, state) -> TransitionCallbackReturn:
        log = self.get_logger()
        try:
            self._core = ThrottleCore(clock=lambda: node_now_sec(self),
                                      rate_hz=self.get_parameter('image_rate_hz').value, logger=log)
            self._quality = int(self.get_parameter('jpeg_quality').value)
            check_quality(self._quality)
        except ValueError as exc:
            log.error(f'invalid configuration: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._pub = self.create_lifecycle_publisher(
            CompressedImage, self.get_parameter('output_topic').value, QOS_SENSOR)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/image_throttle', 10)
        self._sub = self.create_subscription(
            Image, self.get_parameter('input_topic').value, self._on_image, 10)
        log.info('configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._active = True
        self._timer = self.create_timer(1.0 / self.get_parameter('telemetry_rate_hz').value, self._telemetry)
        self.get_logger().info('activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False
        self.destroy_timer(self._timer)
        self._timer = None
        super().on_deactivate(state)
        self.get_logger().info('deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.destroy_subscription(self._sub)
        self.destroy_publisher(self._pub)
        self.destroy_publisher(self._pub_telemetry)
        self._core = None
        return TransitionCallbackReturn.SUCCESS

    def _on_image(self, msg: Image) -> None:
        if not (self._active and self._core.offer(stamp_to_sec(msg.header.stamp))):
            return
        try:
            jpeg = encode_jpeg(bytes(msg.data), msg.height, msg.width, msg.step, msg.encoding, self._quality)
        except ValueError as exc:
            self._encode_errors += 1
            self.get_logger().warning(f'frame not sent: {exc}')
            return
        out = CompressedImage(format='jpeg', data=jpeg)
        out.header = msg.header                      # timestamp echo
        self._pub.publish(out)

    def _telemetry(self) -> None:
        values = {**self._core.snapshot(), 'encode_errors': self._encode_errors}
        self._pub_telemetry.publish(build_diagnostics(self, 'throttling', values))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageThrottleNode()
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
