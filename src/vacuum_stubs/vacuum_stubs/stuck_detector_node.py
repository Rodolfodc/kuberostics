"""`stuck_detector` stub LifecycleNode (T012, 001-R07): each /myRobot/stuck_detector/trigger call publishes one
StuckEvent on /myRobot/stuck. Real detection is spec 005."""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray
from vacuum_interfaces.msg import StuckEvent

from vacuum_stubs.stuck_core import StuckStub
from vacuum_stubs.telemetry import build_diagnostics


class StuckDetectorNode(LifecycleNode):
    def __init__(self, node_name: str = 'stuck_detector', **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('telemetry_rate_hz', 1.0)
        self._core = None
        self._timer = None
        self._active = False

    def on_configure(self, state) -> TransitionCallbackReturn:
        self._core = StuckStub()
        self._pub = self.create_lifecycle_publisher(StuckEvent, '/myRobot/stuck', 10)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/stuck_detector', 10)
        self._srv = self.create_service(Trigger, '/myRobot/stuck_detector/trigger', self._on_trigger)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._active = True
        self._timer = self.create_timer(1.0 / self.get_parameter('telemetry_rate_hz').value, self._telemetry)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.destroy_timer(self._timer)
        self._timer = None
        self._active = False
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.destroy_service(self._srv)
        self.destroy_publisher(self._pub)
        self.destroy_publisher(self._pub_telemetry)
        self._core = None
        return TransitionCallbackReturn.SUCCESS

    def _on_trigger(self, request, response):
        if self._core is None or not self._active:
            response.success = False
            response.message = 'not active'
            return response
        event = StuckEvent(cause=self._core.trigger())
        event.stamp = self.get_clock().now().to_msg()
        event.pose.orientation.w = 1.0      # valid identity quaternion; position stays zero
        self._pub.publish(event)
        self.get_logger().info(f'stuck event published (cause={event.cause})')
        response.success = True
        response.message = event.cause
        return response

    def _telemetry(self) -> None:
        self._pub_telemetry.publish(build_diagnostics(self, 'ok', self._core.snapshot()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StuckDetectorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
