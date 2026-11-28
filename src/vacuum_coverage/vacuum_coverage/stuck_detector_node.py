"""Real `stuck_detector` LifecycleNode (spec 005-R01 to R06, R26, R27): thin glue over `StuckCore`.

A service in the sense of Art. III.4: it perceives and never moves the robot. Inputs: the command on
/myRobot/cmd_vel (the output of twist_mux, ia368ii QoS), the odometry (`odom_topic`), the laser (`scan_topic`), TF
`map_frame -> base_frame` and, when `bumper_topic` is set, a Bool. Output: `StuckEvent` on /myRobot/stuck and
telemetry on /myRobot/telemetry/stuck_detector at `telemetry_rate_hz`. The test hook /myRobot/stuck_detector/trigger
of the stub is kept (005-R06).
"""
import math
import threading
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from vacuum_interfaces.msg import StuckEvent

from vacuum_coverage.core.sensor_cache import RisingEdge, ScanCache, quaternion_to_yaw
from vacuum_coverage.core.stuck_core import NO_PROGRESS, StuckCore
from vacuum_coverage.node_support import (
    QOS_DEFAULT, QOS_IA368II, QOS_SCAN, build_diagnostics, guarded_change_state)

CMD_TOPIC = '/myRobot/cmd_vel'       # fixed by the contract (005-R01), not a parameter
CORE_PARAMS = ('cmd_linear_min_m_s', 'cmd_angular_min_rad_s', 'window_s', 'min_translation_m',
               'min_rotation_rad', 'cooldown_s', 'cmd_timeout_s', 'max_sample_gap_s')


class StuckDetectorNode(LifecycleNode):
    def __init__(self, node_name: str = 'stuck_detector', **kwargs):
        super().__init__(node_name, **kwargs)
        for name, default in (('telemetry_rate_hz', 1.0), ('cmd_linear_min_m_s', 0.05),
                              ('cmd_angular_min_rad_s', 0.20), ('window_s', 3.0), ('min_translation_m', 0.03),
                              ('min_rotation_rad', 0.10), ('cooldown_s', 5.0), ('cmd_timeout_s', 0.5),
                              ('max_sample_gap_s', 0.5), ('scan_max_age_s', 0.5)):
            self.declare_parameter(name, default)
        for name, default in (('map_frame', 'map'), ('base_frame', 'base_link'), ('odom_topic', '/myrobot/odom'),
                              ('scan_topic', '/scan'), ('bumper_topic', '')):
            self.declare_parameter(name, default)
        self._lock = threading.RLock()            # the core and the caches are touched by several callbacks
        self._core: Optional[StuckCore] = None
        self._active = False
        self._scan = ScanCache()
        self._edge = RisingEdge()
        self._cmd = (0.0, 0.0)
        self._cmd_t: Optional[float] = None
        self._odom_pose = None
        self._odom_warned = False
        self._tf_warned = False
        self._published = 0
        self._tf_fallbacks = 0
        self._timer = None

    # ---- lifecycle ----
    def _LifecycleNodeMixin__on_change_state(self, request, response):  # noqa: N802 - overrides the rclpy callback
        return guarded_change_state(self, super()._LifecycleNodeMixin__on_change_state, request, response)

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

    def on_configure(self, state) -> TransitionCallbackReturn:
        p = self.get_parameter
        try:
            self._core = StuckCore(**{name: p(name).value for name in CORE_PARAMS}, logger=self._log)
        except ValueError as exc:
            self.get_logger().error(f'invalid parameters: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._edge = RisingEdge()
        self._cmd_t = None
        self._odom_pose = None
        self._pub = self.create_lifecycle_publisher(StuckEvent, '/myRobot/stuck', QOS_DEFAULT)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/stuck_detector', QOS_DEFAULT)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        # one mutually exclusive group for every input: the callbacks are short and share the core
        group = MutuallyExclusiveCallbackGroup()
        self._subs = [
            self.create_subscription(Twist, CMD_TOPIC, self._on_cmd, QOS_IA368II, callback_group=group),
            self.create_subscription(Odometry, p('odom_topic').value, self._on_odom, QOS_DEFAULT,
                                     callback_group=group),
            self.create_subscription(LaserScan, p('scan_topic').value, self._on_scan, QOS_SCAN,
                                     callback_group=group)]
        if p('bumper_topic').value:
            self._subs.append(self.create_subscription(Bool, p('bumper_topic').value, self._on_bumper,
                                                       QOS_DEFAULT, callback_group=group))
        self._srv = self.create_service(Trigger, '/myRobot/stuck_detector/trigger', self._on_trigger,
                                        callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'configured: cmd {CMD_TOPIC}, odom {p("odom_topic").value}, '
                               f'scan {p("scan_topic").value}, bumper {p("bumper_topic").value or "disabled"}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._active = True
        self._timer = self.create_timer(1.0 / self.get_parameter('telemetry_rate_hz').value, self._telemetry)
        self.get_logger().info('active')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        super().on_deactivate(state)
        self.get_logger().info('inactive')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        self.destroy_service(self._srv)
        self.destroy_publisher(self._pub)
        self.destroy_publisher(self._pub_telemetry)
        self._tf_listener = None
        self._tf_buffer = None
        self._core = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._active = False
        return TransitionCallbackReturn.SUCCESS

    # ---- inputs ----
    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._cmd = (msg.linear.x, msg.angular.z)
            self._cmd_t = self._now_s()

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._scan.store(self._now_s(), msg.ranges, msg.angle_min, msg.angle_increment)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
        with self._lock:
            self._odom_pose = pose
            if self._core is None or not self._active:
                return
            t = self._now_s()
            age = math.inf if self._cmd_t is None else t - self._cmd_t
            cause = self._core.update(t, self._cmd[0], self._cmd[1], pose, cmd_age_s=age)
            if cause:
                self._publish(cause, t)

    def _on_bumper(self, msg: Bool) -> None:
        with self._lock:
            if self._core is None or not self._active or not self._edge.update(msg.data):
                return
            t = self._now_s()
            cause = self._core.bumper_pressed(t)
            if cause:
                self._publish(cause, t)

    def _on_trigger(self, request, response):
        with self._lock:
            if self._core is None or not self._active:
                response.success = False
                response.message = 'not active'
                return response
            self._publish(NO_PROGRESS, self._now_s())          # test hook: bypasses the core on purpose (005-R06)
            response.success = True
            response.message = NO_PROGRESS
            return response

    # ---- output ----
    def _event_pose(self, event: StuckEvent) -> None:
        p = self.get_parameter
        try:
            tf = self._tf_buffer.lookup_transform(p('map_frame').value, p('base_frame').value, Time())
            event.pose.position.x = tf.transform.translation.x
            event.pose.position.y = tf.transform.translation.y
            event.pose.orientation = tf.transform.rotation
            self._tf_warned = False
            return
        except Exception as exc:  # noqa: BLE001 - any TF failure falls back to the odometry (005-R04)
            self._tf_fallbacks += 1
            if not self._tf_warned:
                self.get_logger().warning(f'TF {p("map_frame").value} -> {p("base_frame").value} unavailable '
                                          f'({type(exc).__name__}): using the odometry pose')
                self._tf_warned = True
        event.pose.orientation.w = 1.0
        if self._odom_pose is not None:
            x, y, yaw = self._odom_pose
            event.pose.position.x, event.pose.position.y = x, y
            event.pose.orientation.z, event.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _publish(self, cause: str, t: float) -> None:
        event = StuckEvent(cause=cause)
        event.stamp = self.get_clock().now().to_msg()
        self._event_pose(event)
        event.scan_ranges = [float(r) for r in self._scan.fresh_ranges(t, self.get_parameter('scan_max_age_s').value)]
        self._pub.publish(event)
        self._published += 1
        self.get_logger().info(f'stuck event published (cause={cause}, scan_ranges={len(event.scan_ranges)})')

    def _telemetry(self) -> None:
        with self._lock:
            snap = self._core.snapshot() if self._core is not None else {}
            snap.update({'published': self._published, 'tf_fallbacks': self._tf_fallbacks})
            self._pub_telemetry.publish(build_diagnostics(self, snap.get('state', 'idle'), snap))


def spin_resilient(executor, node) -> None:
    while rclpy.ok():
        try:
            executor.spin()
            return
        except KeyboardInterrupt:
            return
        except Exception as exc:  # noqa: BLE001
            if not rclpy.ok():
                return
            node.get_logger().error(f'callback raised {type(exc).__name__}: {exc}; spinning on')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StuckDetectorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        spin_resilient(executor, node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
