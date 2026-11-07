"""`coverage_server` stub LifecycleNode (T010, 001-R06): CoverRoom action server on /myRobot/cover_room.

While a goal is active it publishes a constant Twist on /myRobot/mux/coverage (its own mux input only); it runs
until cancelled and publishes nothing when idle. Logic in `CoverageRamp` (pure Python, D-18).
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from vacuum_interfaces.action import CoverRoom

from vacuum_stubs.coverage_core import CoverageRamp
from vacuum_stubs.telemetry import QOS_IA368II, build_diagnostics


class CoverageServerNode(LifecycleNode):
    def __init__(self, node_name: str = 'coverage_server', **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('linear_x', 0.15)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('telemetry_rate_hz', 1.0)
        self._active = False
        self._generation = 0      # bumped on every activate/deactivate: goals of an old generation abort
        self._busy = False        # a goal is accepted and not yet finished
        self._goals = 0
        self._running = 0
        self._ratio = 0.0
        self._timer = None

    def on_configure(self, state) -> TransitionCallbackReturn:
        self._ramp = CoverageRamp(self.get_parameter('linear_x').value, self.get_parameter('angular_z').value)
        self._pub = self.create_lifecycle_publisher(Twist, '/myRobot/mux/coverage', QOS_IA368II)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/coverage_server', 10)
        self._server = ActionServer(
            self, CoverRoom, '/myRobot/cover_room', execute_callback=self._execute,
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
        self._active = False          # running goals abort on their next iteration
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

    def _execute(self, goal_handle):
        self._goals += 1
        self._running += 1
        generation = self._generation
        self.get_logger().info(f'CoverRoom goal accepted (room_id={goal_handle.request.room_id!r})')
        period = 1.0 / self.get_parameter('publish_rate_hz').value
        start = time.monotonic()
        result = CoverRoom.Result(success=False, covered_ratio=0.0)
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('CoverRoom goal cancelled')
                    return result
                if not self._active or generation != self._generation:
                    goal_handle.abort()
                    return result
                linear_x, angular_z = self._ramp.command()
                self._pub.publish(self._twist(linear_x, angular_z))
                self._ratio = self._ramp.ratio(time.monotonic() - start)
                goal_handle.publish_feedback(CoverRoom.Feedback(covered_ratio=float(self._ratio)))
                time.sleep(period)
            goal_handle.abort()
            return result
        finally:
            self._running -= 1
            self._busy = False
            self._ratio = 0.0

    @staticmethod
    def _twist(linear_x: float, angular_z: float) -> Twist:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def _telemetry(self) -> None:
        self._pub_telemetry.publish(build_diagnostics(
            self, 'covering' if self._running else 'idle',
            {'goals': self._goals, 'running': self._running, 'covered_ratio': round(self._ratio, 3)}))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageServerNode()
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
