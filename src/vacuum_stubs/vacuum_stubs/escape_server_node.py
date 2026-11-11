"""`escape_server` stub LifecycleNode (T011, 001-R07, 001-R16): Escape action server on /myRobot/escape.

Runs a scripted escape (reverse, then turn) on /myRobot/mux/escape (its own mux input only) and returns the
result chosen by the `outcome` parameter (succeed, fail, hang). Logic in `EscapeScript` (pure Python, D-18).
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy._rclpy_pybind11 import RCLError
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from vacuum_interfaces.action import Escape

from vacuum_stubs.escape_core import EscapeScript
from vacuum_stubs.telemetry import QOS_IA368II, build_diagnostics


class EscapeServerNode(LifecycleNode):
    def __init__(self, node_name: str = 'escape_server', **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('outcome', 'succeed')
        self.declare_parameter('reverse_s', 1.0)
        self.declare_parameter('turn_s', 1.0)
        self.declare_parameter('reverse_linear_x', -0.1)
        self.declare_parameter('turn_angular_z', 0.8)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('telemetry_rate_hz', 1.0)
        self._active = False
        self._generation = 0      # bumped on every activate/deactivate: goals of an old generation abort
        self._busy = False        # a goal is accepted and not yet finished
        self._goals = 0
        self._running = 0
        self._last_result = 'none'
        self._timer = None

    def _new_script(self) -> EscapeScript:
        p = self.get_parameter
        return EscapeScript(p('reverse_s').value, p('turn_s').value, p('reverse_linear_x').value,
                            p('turn_angular_z').value, p('outcome').value)

    def on_configure(self, state) -> TransitionCallbackReturn:
        try:
            self._new_script()        # validates `outcome` early
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return TransitionCallbackReturn.FAILURE
        self._pub = self.create_lifecycle_publisher(Twist, '/myRobot/mux/escape', QOS_IA368II)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/escape_server', 10)
        self._server = ActionServer(
            self, Escape, '/myRobot/escape', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._generation += 1
        self._active = True
        self._timer = self.create_timer(1.0 / self.get_parameter('telemetry_rate_hz').value, self._telemetry)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False
        self._generation += 1
        self.destroy_timer(self._timer)
        self._timer = None
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._server.destroy()
        self.destroy_publisher(self._pub)
        self.destroy_publisher(self._pub_telemetry)
        return TransitionCallbackReturn.SUCCESS

    def _on_goal(self, goal) -> GoalResponse:
        if not self._active:
            self.get_logger().warning('goal rejected: node not active')
            return GoalResponse.REJECT
        if self._busy:
            self.get_logger().warning('goal rejected: another goal is active')
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _twist(linear_x: float, angular_z: float) -> Twist:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def _execute(self, goal_handle):
        self._goals += 1
        self._running += 1
        generation = self._generation
        # goal.timeout_s is deliberately ignored: bounding a hung escape is the supervisor's job
        # (escape_timeout_s, 001-R09); the stub must stay hangable to exercise that path (001-R16).
        script = self._new_script()
        outcome = self.get_parameter('outcome').value
        self.get_logger().info(f'Escape goal accepted (outcome={outcome})')
        period = 1.0 / self.get_parameter('publish_rate_hz').value
        start = time.monotonic()
        result = Escape.Result(success=False, duration_s=0.0, strategy='scripted')
        try:
            while rclpy.ok():
                elapsed = time.monotonic() - start
                result.duration_s = float(elapsed)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._last_result = 'cancelled'
                    self.get_logger().info('Escape goal cancelled')
                    return result
                if not self._active or generation != self._generation:
                    goal_handle.abort()
                    self._last_result = 'aborted'
                    return result
                step = script.step(elapsed)
                if step.done:
                    result.success = bool(step.success)
                    goal_handle.succeed()
                    self._last_result = 'success' if result.success else 'failure'
                    self.get_logger().info(f'Escape finished: {self._last_result} in {elapsed:.2f} s')
                    return result
                self._pub.publish(self._twist(*step.command))
                goal_handle.publish_feedback(Escape.Feedback(elapsed_s=float(elapsed)))
                time.sleep(period)
            self._last_result = 'aborted'
            goal_handle.abort()
            return result
        except RCLError:
            if rclpy.ok():
                raise
            # SIGINT/shutdown invalidated the context while the goal was publishing: not an error, just the
            # end of the process. Nothing can be published or logged any more (T021).
            self._last_result = 'aborted'
            return result
        finally:
            self._running -= 1
            self._busy = False

    def _telemetry(self) -> None:
        self._pub_telemetry.publish(build_diagnostics(
            self, 'escaping' if self._running else 'idle',
            {'goals': self._goals, 'running': self._running, 'outcome': self.get_parameter('outcome').value,
             'last_result': self._last_result}))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EscapeServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # SIGINT already invalidated the context: stop the executor threads first (a running goal sees
        # rclpy.ok() == False and returns), then tear down tolerating an already-invalid context (T021).
        executor.shutdown()
        try:
            node.destroy_node()
        except RCLError:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
