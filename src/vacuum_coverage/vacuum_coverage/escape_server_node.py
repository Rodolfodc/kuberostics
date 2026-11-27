"""Real `escape_server` LifecycleNode (spec 005-R08 to R14, R26, R27): thin glue over `EscapeSession` and
`EscapeScript`. Action /myRobot/escape; an actuator that publishes ONLY on /myRobot/mux/escape (Art. VI).

The decisions are in the pure classes. This node reads the odometry (`odom_topic`) and the laser (`scan_topic`),
turns them into the pose and the front and rear clearances, and publishes the command at `publish_rate_hz`.

Stop behaviour (005-R12, D-22): when a goal ends (success, failure, timeout, cancel, deactivation, shutdown) the
node publishes ONE zero Twist on mux/escape and then stays silent; twist_mux releases the input at its 0.5 s
timeout. The zero Twist (priority 80) only brakes: it never moves the robot.

Goal status: success and failure are both `succeed()` with `result.success` set (as the supervisor reads it);
a cancel request ends in `canceled()`; deactivation, shutdown and a rejected-late goal in `abort()`.
`strategy` is always "scripted"; the RL policy of spec 008 is out of scope.
"""
import math
import signal
import threading
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# RCLError is not re-exported by a public rclpy module on Jazzy; this path works on Jazzy and Lyrical.
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from vacuum_interfaces.action import Escape

from vacuum_coverage.core.escape_script import Command, EscapeConfig
from vacuum_coverage.core.escape_session import OUTCOMES, EscapeSession, Final, OutcomeStub
from vacuum_coverage.core.scan_utils import sector_min
from vacuum_coverage.core.sensor_cache import ScanCache, quaternion_to_yaw, scan_geometry_or_default
from vacuum_coverage.node_support import (
    QOS_DEFAULT, QOS_IA368II, QOS_SCAN, build_diagnostics, guarded_change_state)

ODOM_MAX_AGE_S = 1.0       # odometry older than this is "lost": the escape fails instead of driving blind
SCAN_MAX_AGE_S = 1.0       # a laser older than this gives no clearance (None = not clear)

CONFIG_PARAMS = ('reverse_m', 'rear_clearance_m', 'turn_deg_base', 'turn_deg_step', 'front_clearance_m',
                 'max_attempts', 'fallback_forward_m', 'reverse_speed_m_s', 'turn_speed_rad_s',
                 'primary_budget_s', 'fallback_budget_s', 'default_timeout_s', 'front_stop_m',
                 'angle_tolerance_rad', 'distance_tolerance_m', 'range_cap_m', 'max_linear_speed_m_s',
                 'max_angular_speed_rad_s')
DEFAULTS = {f: getattr(EscapeConfig, f) for f in CONFIG_PARAMS}


class EscapeServerNode(LifecycleNode):
    def __init__(self, node_name: str = 'escape_server', **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('outcome', 'auto')
        for name, default in (('reverse_s', 1.0), ('turn_s', 1.0), ('reverse_linear_x', -0.1),
                              ('turn_angular_z', 0.8), ('publish_rate_hz', 10.0), ('telemetry_rate_hz', 1.0),
                              ('front_sector_half_width_rad', 0.35), ('rear_sector_half_width_rad', 0.35)):
            self.declare_parameter(name, default)
        for name in CONFIG_PARAMS:
            self.declare_parameter(name, DEFAULTS[name])
        self.declare_parameter('odom_topic', '/myrobot/odom')
        self.declare_parameter('scan_topic', '/scan')
        self._lock = threading.Lock()
        self._active = False
        self._generation = 0      # bumped on every activate/deactivate: goals of an old generation abort
        self._busy = False        # a goal is accepted and not yet finished
        self._goals = 0
        self._running = 0
        self._counters = {'succeeded': 0, 'failed': 0, 'cancelled': 0, 'aborted': 0, 'rejected': 0}
        self._last_result = 'none'
        self._last_reason = ''
        self._phase = 'idle'
        self._odom = None         # (t, pose)
        self._scan = ScanCache()
        self._timer = None
        self._stopping = False

    # ---- lifecycle ----
    def _LifecycleNodeMixin__on_change_state(self, request, response):  # noqa: N802 - overrides the rclpy callback
        return guarded_change_state(self, super()._LifecycleNodeMixin__on_change_state, request, response)

    def begin_shutdown(self) -> None:
        self._stopping = True

    def goals_running(self) -> int:
        return self._running

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _log(self, level: str, message: str) -> None:
        # one call site per severity: rclpy refuses a severity change between calls from the same line
        logger = self.get_logger()
        if level == 'WARN':
            logger.warning(message)
        elif level == 'ERROR':
            logger.error(message)
        else:
            logger.info(message)

    def _config(self) -> EscapeConfig:
        values = {name: self.get_parameter(name).value for name in CONFIG_PARAMS}
        values['max_attempts'] = int(values['max_attempts'])
        return EscapeConfig(**values)

    def _stub(self) -> OutcomeStub:
        p = self.get_parameter
        return OutcomeStub(p('reverse_s').value, p('turn_s').value, p('reverse_linear_x').value,
                           p('turn_angular_z').value)

    def _new_session(self) -> EscapeSession:
        return EscapeSession(self._config(), self.get_parameter('outcome').value, self._stub(), logger=self._log)

    def on_configure(self, state) -> TransitionCallbackReturn:
        try:
            self._new_session()       # validates `outcome` and the speeds against the limits (005-R11 c)
        except ValueError as exc:
            self.get_logger().error(f'invalid configuration: {exc}')
            return TransitionCallbackReturn.FAILURE
        if self.get_parameter('publish_rate_hz').value <= 0.0:
            self.get_logger().error('publish_rate_hz must be > 0')
            return TransitionCallbackReturn.FAILURE
        self._pub = self.create_lifecycle_publisher(Twist, '/myRobot/mux/escape', QOS_IA368II)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/escape_server', QOS_DEFAULT)
        sensors = MutuallyExclusiveCallbackGroup()
        p = self.get_parameter
        self._subs = [
            self.create_subscription(Odometry, p('odom_topic').value, self._on_odom, QOS_DEFAULT,
                                     callback_group=sensors),
            self.create_subscription(LaserScan, p('scan_topic').value, self._on_scan, QOS_SCAN,
                                     callback_group=sensors)]
        self._server = ActionServer(
            self, Escape, '/myRobot/escape', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'configured: outcome={p("outcome").value}, odom {p("odom_topic").value}, '
                               f'scan {p("scan_topic").value}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._generation += 1
        self._active = True
        self._timer = self.create_timer(1.0 / self.get_parameter('telemetry_rate_hz').value, self._telemetry)
        self.get_logger().info('active')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False          # a running goal sees this, publishes its zero Twist and aborts
        self._generation += 1
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        super().on_deactivate(state)
        self.get_logger().info('inactive')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._server.destroy()
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        self.destroy_publisher(self._pub)
        self.destroy_publisher(self._pub_telemetry)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._active = False
        return TransitionCallbackReturn.SUCCESS

    # ---- inputs ----
    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
        with self._lock:
            self._odom = (self._now_s(), pose)

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._scan.store(self._now_s(), msg.ranges, msg.angle_min, msg.angle_increment)

    def _sense(self, t: float):
        """(pose or None, front, rear) from the latest odometry and laser, each None when missing or stale."""
        p = self.get_parameter
        with self._lock:
            odom = self._odom
            scan_age = self._scan.age(t)
            ranges, geometry = self._scan.ranges(), self._scan.geometry()
        pose = odom[1] if odom is not None and t - odom[0] <= ODOM_MAX_AGE_S else None
        front = rear = None
        if geometry is not None and scan_age <= SCAN_MAX_AGE_S:
            front = sector_min(ranges, geometry[0], geometry[1], 0.0, p('front_sector_half_width_rad').value)
            rear = sector_min(ranges, geometry[0], geometry[1], math.pi, p('rear_sector_half_width_rad').value)
        return pose, front, rear

    # ---- action ----
    def _on_goal(self, goal) -> GoalResponse:
        reason = None
        if not self._active:
            reason = 'node not active'
        elif self._busy:
            reason = 'another goal is active'
        if reason:
            self._counters['rejected'] += 1
            self.get_logger().warning(f'goal rejected: {reason}')
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('cancel requested')
        return CancelResponse.ACCEPT

    @staticmethod
    def _twist(command: Command) -> Twist:
        msg = Twist()
        msg.linear.x = float(command.linear)
        msg.angular.z = float(command.angular)
        return msg

    def _publish_zero(self) -> None:
        try:
            self._pub.publish(Twist())        # one brake command, then silence (see the module docstring)
        except RCLError:
            pass

    def _execute(self, goal_handle):
        self._goals += 1
        self._running += 1
        generation = self._generation
        goal = goal_handle.request
        period = 1.0 / self.get_parameter('publish_rate_hz').value
        start = self._now_s()
        result = Escape.Result(success=False, duration_s=0.0, strategy='scripted')
        moved = False
        try:
            session = self._new_session()
            pose, _, _ = self._sense(start)
            geometry = scan_geometry_or_default(self._scan.geometry(), len(goal.event.scan_ranges))
            session.start(start, list(goal.event.scan_ranges), pose, geometry[0], geometry[1],
                          float(goal.timeout_s))
            self._phase = 'escaping'
            self.get_logger().info(f'Escape goal accepted (cause={goal.event.cause}, outcome={session.outcome}, '
                                   f'timeout {session.timeout_s:.1f} s)')
            while True:
                t = self._now_s()
                result.duration_s = float(t - start)
                if goal_handle.is_cancel_requested:
                    if moved:
                        self._publish_zero()
                    goal_handle.canceled()
                    return self._ended(result, 'cancelled', 'goal cancelled')
                if self._stopping or not self.context.ok() or not self._active or generation != self._generation:
                    if moved:
                        self._publish_zero()
                    goal_handle.abort()
                    return self._ended(result, 'aborted', 'node deactivated or shutting down')
                pose, front, rear = self._sense(t)
                out = session.step(t, pose, front, rear)
                if isinstance(out, Final):
                    if moved:
                        self._publish_zero()
                    result.success = out.success
                    goal_handle.succeed()
                    return self._ended(result, 'success' if out.success else 'failure', out.reason)
                self._pub.publish(self._twist(out))
                moved = True
                goal_handle.publish_feedback(Escape.Feedback(elapsed_s=float(t - start)))
                time.sleep(period)
        except RCLError:
            if self.context.ok():
                raise
            # SIGINT/shutdown invalidated the context while the goal was publishing: not an error, just the
            # end of the process. Nothing can be published or logged any more.
            self._last_result = 'aborted'
            return result
        finally:
            self._running -= 1
            self._busy = False
            self._phase = 'idle'

    def _ended(self, result, kind: str, reason: str):
        key = {'success': 'succeeded', 'failure': 'failed', 'cancelled': 'cancelled', 'aborted': 'aborted'}[kind]
        self._counters[key] += 1
        self._last_result, self._last_reason = kind, reason
        self.get_logger().info(f'Escape finished: {kind} ({reason}) in {result.duration_s:.2f} s')
        return result

    def _telemetry(self) -> None:
        values = {'goals': self._goals, 'running': self._running, 'outcome': self.get_parameter('outcome').value,
                  'last_result': self._last_result, 'last_reason': self._last_reason, **self._counters}
        self._pub_telemetry.publish(build_diagnostics(self, self._phase, values))


def main(args=None) -> None:
    # rclpy's own SIGINT handler invalidates the context at once, while a goal may still be publishing. The signals
    # are handled here instead (same recipe as the stub): flag the goal to stop (context still valid, one zero
    # Twist), wait for it, then shut everything down in order.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = EscapeServerNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        while not stop.is_set() and rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                raise
            except Exception as exc:  # noqa: BLE001 - one bad callback must not end the process
                if not rclpy.ok():
                    break
                node.get_logger().error(f'callback raised {type(exc).__name__}: {exc}; spinning on')
        node.begin_shutdown()
        deadline = time.monotonic() + 3.0
        while node.goals_running() and time.monotonic() < deadline:
            time.sleep(0.05)
    except ExternalShutdownException:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except RCLError:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
