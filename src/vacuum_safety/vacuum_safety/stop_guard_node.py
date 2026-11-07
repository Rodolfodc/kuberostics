"""`stop_guard` LifecycleNode (T018, D-22): publishes zero velocity on /myRobot/mux/stop while the stop lock is on.

It is an actuator in the sense of Art. III: it publishes only to its own twist_mux input. All logic is in
`StopGuard` (pure Python); this node wires it to the topics (D-18) and publishes telemetry (D-20).
"""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from vacuum_safety.stop_guard_core import StopGuard

# QoS of the IA368II contract (profile ia368ii in specs/000-system/contracts/topics.yaml).
QOS_IA368II = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)


class StopGuardNode(LifecycleNode):
    def __init__(self, node_name: str = 'stop_guard', **kwargs):
        super().__init__(node_name, **kwargs)
        # Must stay well above 1 / timeout of the `stop` mux input (0.5 s): at 10 Hz the input never lapses.
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('telemetry_rate_hz', 1.0)
        self._guard = None
        self._timers = []

    def on_configure(self, state) -> TransitionCallbackReturn:
        self._guard = StopGuard(logger=self.get_logger())
        self._pub_stop = self.create_lifecycle_publisher(Twist, '/myRobot/mux/stop', QOS_IA368II)
        self._pub_telemetry = self.create_lifecycle_publisher(DiagnosticArray, '/myRobot/telemetry/stop_guard', 10)
        self._sub_lock = self.create_subscription(Bool, '/myRobot/mux/lock_stop', self._on_lock, QOS_IA368II)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._timers = [
            self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self._tick),
            self.create_timer(1.0 / self.get_parameter('telemetry_rate_hz').value, self._publish_telemetry),
        ]
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        for t in self._timers:
            self.destroy_timer(t)
        self._timers = []
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.destroy_publisher(self._pub_stop)
        self.destroy_publisher(self._pub_telemetry)
        self.destroy_subscription(self._sub_lock)
        self._guard = None
        return TransitionCallbackReturn.SUCCESS

    def _on_lock(self, msg: Bool) -> None:
        self._guard.on_lock(msg.data)

    def _tick(self) -> None:
        cmd = self._guard.command()
        if cmd is not None:
            self._pub_stop.publish(Twist())      # an all-zero Twist; the tuple is (0.0, 0.0) by contract

    def _publish_telemetry(self) -> None:
        snap = self._guard.snapshot()
        status = DiagnosticStatus(
            name=self.get_name(), hardware_id='vacuum',
            level=DiagnosticStatus.WARN if snap['locked'] else DiagnosticStatus.OK,
            message='LOCKED' if snap['locked'] else 'released',
            values=[KeyValue(key=k, value=str(v)) for k, v in snap.items()])
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self._pub_telemetry.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StopGuardNode()
    executor = SingleThreadedExecutor()
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
