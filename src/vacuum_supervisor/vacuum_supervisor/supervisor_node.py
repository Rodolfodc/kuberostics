"""`supervisor` LifecycleNode (T009, spec 001): thin glue between ROS and `SubmodeMachine`.

All decisions live in `SubmodeMachine` (pure Python). This node only wires topics, actions and the
service to the machine and executes the `Effect`s it returns (D-18). Logs go through the node logger
and a DiagnosticArray with the machine metrics is published periodically (D-20).
"""
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.action import ActionClient
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from vacuum_interfaces.action import CoverRoom, Escape
from vacuum_interfaces.msg import Notification, StuckEvent, Submode

from vacuum_supervisor.submode_machine import Effect, SubmodeMachine

# QoS of the IA368II contract (specs/000-system/contracts/topics.yaml, profile ia368ii).
QOS_IA368II = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
SEVERITY = {'INFO': Notification.INFO, 'WARN': Notification.WARN, 'ERROR': Notification.ERROR}


class SupervisorNode(LifecycleNode):
    def __init__(self, node_name: str = 'supervisor', **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('escape_timeout_s', 20.0)      # 001-R09
        # How long a stuck event waits for a not yet discovered escape server before it counts as a failed
        # escape (T021). The escape_timeout_s clock (001-R09) starts at the stuck event, not after this wait.
        self.declare_parameter('escape_server_wait_s', 2.0)
        self.declare_parameter('publish_rate_hz', 5.0)        # submode and lock, plan 001
        self.declare_parameter('telemetry_rate_hz', 1.0)      # D-20
        self.declare_parameter('mode_topic', '/myRobot/behaviour/mode')
        self.declare_parameter('cover_room_action', '/myRobot/cover_room')
        self.declare_parameter('escape_action', '/myRobot/escape')
        self._active = False
        # Created once, not per configure: the stop lock must survive a deactivate/activate or
        # cleanup/configure cycle (001-R22). Only a process restart starts unlocked.
        self._machine = SubmodeMachine(logger=self.get_logger())
        self._last_stuck: Optional[StuckEvent] = None
        self._cover_handle = None
        self._escape_handle = None
        self._escape_seq = 0               # identifies the escape goal whose result still matters
        self._cover_seq = 0                # same for cover_room: a late acceptance must not outlive a cancel
        self._escape_timer = None
        self._pending_cover = False        # goal to send as soon as the server is up
        self._pending_escape_deadline: Optional[float] = None   # monotonic; set while waiting for the server
        self._timers = []

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state) -> TransitionCallbackReturn:
        log = self.get_logger()
        self._pub_submode = self.create_lifecycle_publisher(Submode, '/myRobot/behaviour/submode', QOS_IA368II)
        self._pub_lock = self.create_lifecycle_publisher(Bool, '/myRobot/mux/lock_stop', QOS_IA368II)
        self._pub_notify = self.create_lifecycle_publisher(Notification, '/myRobot/notifications', 10)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/supervisor', 10)
        self._sub_mode = self.create_subscription(
            String, self.get_parameter('mode_topic').value, self._on_mode, QOS_IA368II)
        self._sub_stuck = self.create_subscription(StuckEvent, '/myRobot/stuck', self._on_stuck, 10)
        self._cover_client = ActionClient(self, CoverRoom, self.get_parameter('cover_room_action').value)
        self._escape_client = ActionClient(self, Escape, self.get_parameter('escape_action').value)
        self._srv_reset = self.create_service(Trigger, '/myRobot/supervisor/reset', self._on_reset)
        log.info('configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._active = True
        self._timers = [
            self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self._tick),
            self.create_timer(1.0 / self.get_parameter('telemetry_rate_hz').value, self._publish_telemetry),
        ]
        self.get_logger().info('activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False
        self._cancel_actions()
        self._machine.on_mode('')          # back to NONE; the lock is kept
        for t in self._timers:
            self.destroy_timer(t)
        self._timers = []
        super().on_deactivate(state)
        self.get_logger().info('deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        for pub in (self._pub_submode, self._pub_lock, self._pub_notify, self._pub_telemetry):
            self.destroy_publisher(pub)
        self.destroy_subscription(self._sub_mode)
        self.destroy_subscription(self._sub_stuck)
        self.destroy_service(self._srv_reset)
        self._cover_client.destroy()
        self._escape_client.destroy()
        return TransitionCallbackReturn.SUCCESS

    # ---------------------------------------------------------------- inputs
    def _on_mode(self, msg: String) -> None:
        if self._active:
            self._apply(self._machine.on_mode(msg.data))

    def _on_stuck(self, msg: StuckEvent) -> None:
        if self._active:
            self._last_stuck = msg
            self._apply(self._machine.on_stuck())

    def _on_reset(self, request, response):
        effects = self._machine.reset() if self._active else []
        self._apply(effects)
        response.success = bool(effects)
        response.message = 'lock released' if effects else f'nothing to reset (submode {self._machine.submode})'
        self.get_logger().info(f'reset requested: {response.message}')
        return response

    # ---------------------------------------------------------------- effects
    def _apply(self, effects) -> None:
        for e in effects:
            self._execute(e)

    def _execute(self, effect: Effect) -> None:
        if effect.kind == 'cancel_actions':
            self._cancel_actions()
        elif effect.kind == 'send_cover_room':
            self._pending_cover = True
            self._try_send_cover_room()
        elif effect.kind == 'send_escape':
            self._send_escape()
        elif effect.kind == 'lock_stop':
            self.get_logger().warning(f'lock_stop = {self._machine.locked}')
            self._pub_lock.publish(Bool(data=self._machine.locked))
        elif effect.kind == 'notify':
            severity, code = effect.value
            self._notify(severity, code, f'{code}: submode {self._machine.submode} ({self._machine.reason})')
        else:
            self.get_logger().error(f'unknown effect {effect.kind}')

    def _notify(self, severity: str, code: str, text: str) -> None:
        msg = Notification(severity=SEVERITY[severity], source=self.get_name(), code=code, text=text)
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_notify.publish(msg)
        self.get_logger().warning(f'notification {severity} {code}')

    def _cancel_actions(self) -> None:
        self._pending_cover = False
        self._pending_escape_deadline = None
        self._cover_seq += 1               # any callback of the goals cancelled below is stale
        self._escape_seq += 1
        self._stop_escape_timer()
        for handle in (self._cover_handle, self._escape_handle):
            if handle is not None:
                handle.cancel_goal_async()
        self._cover_handle = self._escape_handle = None

    # ---------------------------------------------------------------- cover_room
    def _try_send_cover_room(self) -> None:
        if not self._pending_cover:
            return
        if not self._cover_client.server_is_ready():
            self.get_logger().warning('cover_room server not ready, will retry')
            return
        self._pending_cover = False
        self._cover_seq += 1
        seq = self._cover_seq
        goal = CoverRoom.Goal(room_id='', passes=1)
        self.get_logger().info('sending cover_room goal')
        self._cover_client.send_goal_async(goal).add_done_callback(
            lambda fut, s=seq: self._on_cover_accepted(fut, s))

    def _on_cover_accepted(self, future, seq: int) -> None:
        handle = future.result()
        if seq != self._cover_seq:                    # cancelled (mode change, stuck) before being accepted
            self.get_logger().info('cover_room goal accepted after a cancel: cancelling it')
            handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error('cover_room goal rejected')
            return
        self._cover_handle = handle
        handle.get_result_async().add_done_callback(lambda fut, s=seq: self._on_cover_result(fut, s))

    def _on_cover_result(self, future, seq: int) -> None:
        status = future.result().status
        self.get_logger().info(f'cover_room finished with status {status}')
        if seq == self._cover_seq:
            self._cover_handle = None

    # ---------------------------------------------------------------- escape
    def _send_escape(self) -> None:
        timeout_s = float(self.get_parameter('escape_timeout_s').value)
        self._escape_seq += 1
        self._stop_escape_timer()
        # 001-R09: the escape_timeout_s clock starts now, also while waiting for the server to be discovered.
        self._escape_timer = self.create_timer(timeout_s, self._on_escape_timeout)
        wait_s = float(self.get_parameter('escape_server_wait_s').value)
        if not self._escape_client.server_is_ready():
            self.get_logger().warning(f'escape server not discovered yet: waiting up to {wait_s:.1f} s')
        self._pending_escape_deadline = time.monotonic() + wait_s
        self._try_send_escape()

    def _try_send_escape(self) -> None:
        """Sends the escape goal once the server is discovered; gives up after `escape_server_wait_s` (T021).

        Called right away and then from the periodic tick, so the executor is never blocked.
        """
        if self._pending_escape_deadline is None:
            return
        if not self._escape_client.server_is_ready():
            if time.monotonic() < self._pending_escape_deadline:
                return
            self.get_logger().error('escape server not discovered within escape_server_wait_s: '
                                    'treating as a failed escape')
            self._pending_escape_deadline = None
            self._stop_escape_timer()
            self._apply(self._machine.on_escape_result(success=False))
            return
        self._pending_escape_deadline = None
        seq = self._escape_seq
        timeout_s = float(self.get_parameter('escape_timeout_s').value)
        goal = Escape.Goal(event=self._last_stuck or StuckEvent(), timeout_s=timeout_s)
        self.get_logger().info(f'sending escape goal (timeout {timeout_s:.1f} s)')
        self._escape_client.send_goal_async(goal).add_done_callback(
            lambda fut, s=seq: self._on_escape_accepted(fut, s))

    def _on_escape_accepted(self, future, seq: int) -> None:
        handle = future.result()
        if seq != self._escape_seq:                    # cancelled before being accepted
            handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error('escape goal rejected')
            self._stop_escape_timer()
            self._apply(self._machine.on_escape_result(success=False))
            return
        self._escape_handle = handle
        handle.get_result_async().add_done_callback(lambda fut, s=seq: self._on_escape_result(fut, s))

    def _on_escape_result(self, future, seq: int) -> None:
        if seq != self._escape_seq:
            self.get_logger().debug('stale escape result dropped')
            return
        wrapped = future.result()
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success
        self.get_logger().info(f'escape finished (status {wrapped.status}, success {ok}, '
                               f'strategy {wrapped.result.strategy!r}, {wrapped.result.duration_s:.2f} s)')
        self._stop_escape_timer()
        self._escape_handle = None
        self._apply(self._machine.on_escape_result(success=ok))

    def _on_escape_timeout(self) -> None:
        self.get_logger().error('escape exceeded escape_timeout_s')
        self._stop_escape_timer()
        self._apply(self._machine.on_escape_timeout())

    def _stop_escape_timer(self) -> None:
        if self._escape_timer is not None:
            self.destroy_timer(self._escape_timer)
            self._escape_timer = None

    # ---------------------------------------------------------------- periodic outputs
    def _tick(self) -> None:
        m = self._machine
        msg = Submode(mode=m.mode, submode=m.submode, reason=m.reason)
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_submode.publish(msg)
        self._pub_lock.publish(Bool(data=self._machine.locked))       # republished: twist_mux may start later
        self._try_send_cover_room()
        self._try_send_escape()

    def _publish_telemetry(self) -> None:
        snap = self._machine.snapshot()
        status = DiagnosticStatus(
            name=self.get_name(), hardware_id='vacuum',
            level=DiagnosticStatus.ERROR if snap['locked'] else DiagnosticStatus.OK,
            message=snap['submode'],
            values=[KeyValue(key=k, value=str(v)) for k, v in snap.items()])
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self._pub_telemetry.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupervisorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except RCLError:                         # the context was already invalidated by SIGINT (T021)
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
