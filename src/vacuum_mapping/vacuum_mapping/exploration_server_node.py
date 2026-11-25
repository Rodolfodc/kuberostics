"""`exploration_server` LifecycleNode (spec 003, optional and late): Explore action server on
/myRobot/explore that drives the robot to frontiers through Nav2 `NavigateToPose` goals.

Thin glue over `ExplorationSession` (pure Python, D-18) and `FrontierFinder`. Requirements:
- 003-R10, R11: frontier loop, retry, blacklist, success/failure, WARN `EXPLORATION_INCOMPLETE`.
- 003-R12: on cancel the Nav2 goal is cancelled at once (inside the cancel callback); the node publishes NO
  velocity (Art. III.4: Nav2's controller_server drives on /myRobot/mux/nav).
- 003-R13: sole publisher of /myRobot/map/explored (latched Bool): false at configure, true when a goal
  succeeds or when the parameter `map_loaded` is true.
- 003-R26: LifecycleNode, logs every transition and event, telemetry at 1 Hz on
  /myRobot/telemetry/exploration_server (D-20).
- 003-R31: deactivate and shutdown cancel the Nav2 goal in flight before the node stops; every frontier goal
  is bounded by `frontier_timeout_s`; feedback is published every 0.5 s (the supervisor watchdog of 005-R34
  treats a server without feedback for `feedback_stale_s` as lost).
- 003-R32: a cancel keeps the session (blacklist, counters, clock origin); the re-sent goal resumes it.

Fixed by design, not parameters (the contract lists none): frames `map` and `base_link`, topics `/map` and
`/myRobot/notifications`, rates (loop 0.05 s, feedback 0.5 s, telemetry 1 Hz).
"""
import math
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from vacuum_grid.grid_view import GridView
from vacuum_interfaces.action import Explore
from vacuum_interfaces.msg import Notification

from vacuum_mapping.core.exploration_session import ExplorationConfig, ExplorationSession, StepKind
from vacuum_mapping.core.types import classify_occupancy

MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
LOOP_PERIOD_S = 0.05
FEEDBACK_PERIOD_S = 0.5          # 003-R31: 1 Hz or faster
TELEMETRY_PERIOD_S = 1.0
NAV_CANCEL_WAIT_S = 0.5

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def grid_view_from_msg(msg: OccupancyGrid) -> GridView:
    values = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    return GridView(classify_occupancy(values), float(msg.info.resolution),
                    float(msg.info.origin.position.x), float(msg.info.origin.position.y))


class ExplorationServerNode(LifecycleNode):
    def __init__(self, node_name: str = 'exploration_server', **kwargs):
        super().__init__(node_name, **kwargs)
        # Defaults and meaning: specs/000-system/contracts/topics.yaml (parameters: exploration_server).
        self.declare_parameter('min_frontier_cells', 8)
        self.declare_parameter('gain_weight', 0.05)
        self.declare_parameter('robot_radius_m', 0.16)
        self.declare_parameter('frontier_timeout_s', 60.0)
        self.declare_parameter('explore_timeout_s', 900.0)
        self.declare_parameter('map_loaded', False)
        self.declare_parameter('max_blacklist_ratio', 0.5)
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self._group = ReentrantCallbackGroup()
        self._grid: Optional[GridView] = None
        self._active = False
        self._generation = 0
        self._busy = False
        self._goal_lock = threading.Lock()
        self._session: Optional[ExplorationSession] = None
        self._nav_lock = threading.Lock()
        self._nav_seq = 0
        self._nav_handle = None
        self._nav_outcome: Optional[bool] = None
        self._wake = threading.Event()
        self._timer = None
        self._state = 'unconfigured'
        self._counters = {'goals': 0, 'succeeded': 0, 'failed': 0, 'cancelled': 0, 'rejected': 0,
                          'resumed': 0, 'nav_goals': 0}
        self._snapshot = {}
        self._explored = False

    # ---- lifecycle ----
    def _LifecycleNodeMixin__on_change_state(self, request, response):  # noqa: N802 - overrides the rclpy callback
        """A redundant or impossible transition must answer success=false (Jazzy's rclpy raises out of the
        service callback instead); same override as coverage_server."""
        try:
            return super()._LifecycleNodeMixin__on_change_state(request, response)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'transition {request.transition.id} refused in state {self._state}: {type(exc).__name__}')
            response.success = False
            return response

    def on_configure(self, state) -> TransitionCallbackReturn:
        config = ExplorationConfig(
            frontier_timeout_s=float(self.get_parameter('frontier_timeout_s').value),
            explore_timeout_s=float(self.get_parameter('explore_timeout_s').value),
            max_blacklist_ratio=float(self.get_parameter('max_blacklist_ratio').value),
            min_frontier_cells=int(self.get_parameter('min_frontier_cells').value),
            gain_weight=float(self.get_parameter('gain_weight').value),
            robot_radius_m=float(self.get_parameter('robot_radius_m').value))
        try:
            self._session = ExplorationSession(config, log=self.get_logger().info)
        except ValueError as exc:
            self.get_logger().error(f'configure failed: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._sub_map = self.create_subscription(OccupancyGrid, '/map', self._on_map, LATCHED,
                                                 callback_group=self._group)
        self._nav = ActionClient(self, NavigateToPose, self.get_parameter('nav_action').value,
                                 callback_group=self._group)
        # Plain publisher on purpose: the latched value must exist as soon as the node is configured (003-R13).
        self._pub_explored = self.create_publisher(Bool, '/myRobot/map/explored', LATCHED)
        self._pub_notify = self.create_lifecycle_publisher(Notification, '/myRobot/notifications', 10)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/exploration_server', 10)
        self._server = ActionServer(
            self, Explore, '/myRobot/explore', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel, callback_group=self._group)
        self._set_explored(bool(self.get_parameter('map_loaded').value), 'map_loaded parameter')
        self._set_state('inactive')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._generation += 1
        self._active = True
        self._timer = self.create_timer(TELEMETRY_PERIOD_S, self._telemetry, callback_group=self._group)
        self._set_state('idle')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False                # a running goal aborts on its next iteration
        self._generation += 1
        self._cancel_nav(wait=True)         # 003-R31: the Nav2 goal goes before the node stops
        self._wake.set()
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        super().on_deactivate(state)
        self._set_state('inactive')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self._server.destroy()
        self._nav.destroy()
        self.destroy_subscription(self._sub_map)
        for pub in (self._pub_explored, self._pub_notify, self._pub_telemetry):
            self.destroy_publisher(pub)
        self._tf_listener.unregister()
        self._grid = None
        self._set_state('unconfigured')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._active = False
        self._generation += 1
        self._cancel_nav(wait=True)         # 003-R31 also on shutdown
        self._wake.set()
        return TransitionCallbackReturn.SUCCESS

    def _set_state(self, new: str) -> None:
        if new != self._state:
            self.get_logger().info(f'state {self._state} -> {new}')
            self._state = new

    def _set_explored(self, value: bool, why: str) -> None:
        self.get_logger().info(f'explored={value} ({why})')
        self._explored = value
        self._pub_explored.publish(Bool(data=value))

    # ---- inputs -----------------------------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            self._grid = grid_view_from_msg(msg)
        except ValueError as exc:
            self.get_logger().error(f'ignoring invalid /map: {exc}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time(), Duration(seconds=0.0))
        except Exception:  # noqa: BLE001 - TF not there yet
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    # ---- action server ----------------------------------------------------------------------------------
    def _reject(self, why: str) -> GoalResponse:
        self._counters['rejected'] += 1
        self.get_logger().warning(f'goal rejected: {why}')
        return GoalResponse.REJECT

    def _on_goal(self, goal) -> GoalResponse:
        if not self._active:
            return self._reject('node not active')
        with self._goal_lock:
            if self._busy:
                return self._reject('another goal is active')
            if self._grid is None:
                return self._reject('no map received yet')
            if not self._nav.server_is_ready():
                return self._reject(f'{self.get_parameter("nav_action").value} is not available')
            if self._pose() is None:
                return self._reject(f'no TF {MAP_FRAME} -> {BASE_FRAME}')
            self._busy = True
        self.get_logger().info(f'goal accepted timeout_s={goal.timeout_s}')
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('cancel requested: cancelling the Nav2 goal (003-R12)')
        self._cancel_nav()
        self._wake.set()
        return CancelResponse.ACCEPT

    def _result(self, success: bool, done: int, blacklisted: int, start: float) -> Explore.Result:
        return Explore.Result(success=success, frontiers_done=int(done), frontiers_blacklisted=int(blacklisted),
                              duration_s=float(max(0.0, self._now() - start)))

    def _execute(self, goal_handle):
        session = self._session
        generation = self._generation
        self._counters['goals'] += 1
        self._set_state('exploring')
        resumed = session.begin(self._now(), float(goal_handle.request.timeout_s))
        if resumed:
            self._counters['resumed'] += 1
        start = session.start_time
        next_feedback = 0.0
        try:
            while rclpy.ok():
                t = self._now()
                if goal_handle.is_cancel_requested:
                    self._cancel_nav()
                    session.suspend(t)                  # 003-R32: blacklist, counters and clock origin are kept
                    self._counters['cancelled'] += 1
                    snap = session.snapshot()
                    goal_handle.canceled()
                    return self._result(False, snap['done'], snap['blacklisted'], start)
                if not self._active or generation != self._generation:
                    self._cancel_nav()
                    snap = session.snapshot()
                    session.end()
                    self._counters['failed'] += 1
                    goal_handle.abort()
                    self.get_logger().warning('goal aborted: node deactivated')
                    return self._result(False, snap['done'], snap['blacklisted'], start)
                with self._nav_lock:
                    outcome, self._nav_outcome = self._nav_outcome, None
                if outcome is not None:
                    session.goal_result(t, outcome)
                pose = self._pose()
                grid = self._grid
                step = session.step(t, grid, pose) if pose is not None and grid is not None else None
                if step is not None:
                    if step.kind is StepKind.SEND:
                        self._send_nav(step.frontier, pose)
                    elif step.kind is StepKind.CANCEL:
                        self._cancel_nav()
                    elif step.kind is StepKind.DONE:
                        if step.cancel_goal:
                            self._cancel_nav()
                        return self._finish(goal_handle, step, start)
                self._snapshot = session.snapshot()
                if t >= next_feedback:
                    goal_handle.publish_feedback(Explore.Feedback(
                        frontiers_remaining=int(self._snapshot.get('remaining', 0)),
                        elapsed_s=float(max(0.0, t - start))))
                    next_feedback = t + FEEDBACK_PERIOD_S
                self._wake.wait(LOOP_PERIOD_S)
                self._wake.clear()
            self._cancel_nav()
            goal_handle.abort()
            return self._result(False, 0, 0, start)
        finally:
            self._busy = False
            self._set_state('idle' if self._active else 'inactive')

    def _finish(self, goal_handle, step, start: float) -> Explore.Result:
        result = self._result(step.success, step.done, step.blacklisted, start)
        if step.success:
            self._counters['succeeded'] += 1
            self._set_explored(True, 'goal succeeded (003-R13)')
            goal_handle.succeed()
        else:
            self._counters['failed'] += 1
            self._notify(Notification.WARN, 'EXPLORATION_INCOMPLETE',
                         f'Exploration incomplete ({step.reason}); the partial map is kept')
            goal_handle.abort()
        self.get_logger().info(f'goal done success={result.success} reason={step.reason} '
                               f'done={step.done} blacklisted={step.blacklisted} duration={result.duration_s:.1f} s')
        return result

    def _notify(self, severity: int, code: str, text: str) -> None:
        msg = Notification(severity=int(severity), source=self.get_name(), code=code, text=text)
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_notify.publish(msg)
        self.get_logger().warning(f'notification {code}: {text}')

    # ---- Nav2 goal --------------------------------------------------------------------------------------
    def _send_nav(self, frontier, pose) -> None:
        x, y = frontier.centroid
        yaw = math.atan2(y - pose[1], x - pose[0])
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        with self._nav_lock:
            self._nav_seq += 1
            seq = self._nav_seq
            self._nav_outcome = None
        self._counters['nav_goals'] += 1
        self.get_logger().info(f'nav goal {seq}: ({x:.2f}, {y:.2f})')
        self._nav.send_goal_async(goal).add_done_callback(lambda f, s=seq: self._on_nav_accepted(f, s))

    def _on_nav_accepted(self, future, seq: int) -> None:
        handle = future.result()
        with self._nav_lock:
            current = seq == self._nav_seq
            if current and handle.accepted:
                self._nav_handle = handle
            elif current:
                self._nav_outcome = False          # Nav2 refused the goal: counts as a failed attempt
        if not handle.accepted:
            self._wake.set()
            return
        if not current:
            handle.cancel_goal_async()             # cancelled before Nav2 answered
            return
        handle.get_result_async().add_done_callback(lambda f, s=seq: self._on_nav_result(f, s))

    def _on_nav_result(self, future, seq: int) -> None:
        with self._nav_lock:
            if seq != self._nav_seq:
                return                              # late reply of a goal already cancelled
            self._nav_outcome = future.result().status == GoalStatus.STATUS_SUCCEEDED
            self._nav_handle = None
        self._wake.set()

    def _cancel_nav(self, wait: bool = False) -> None:
        """Invalidates the goal in flight (its replies are ignored from now on) and asks Nav2 to cancel it."""
        with self._nav_lock:
            self._nav_seq += 1
            self._nav_outcome = None
            handle, self._nav_handle = self._nav_handle, None
        if handle is None:
            return
        future = handle.cancel_goal_async()
        if wait:
            end = time.monotonic() + NAV_CANCEL_WAIT_S
            while not future.done() and time.monotonic() < end:
                time.sleep(0.005)

    # ---- outputs ----------------------------------------------------------------------------------------
    def _telemetry(self) -> None:
        """D-20: state and counters at 1 Hz."""
        values = dict(self._counters)
        values.update({k: round(v, 3) for k, v in self._snapshot.items()})
        values['explored'] = self._explored
        status = DiagnosticStatus(
            name=self.get_name(), hardware_id='vacuum', level=DiagnosticStatus.OK, message=self._state,
            values=[KeyValue(key='state', value=self._state)] + [
                KeyValue(key=k, value=str(v)) for k, v in values.items()])
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self._pub_telemetry.publish(array)


def spin_resilient(executor, node) -> None:
    """Spins until shutdown; an exception of one callback is logged and does not end the process."""
    while rclpy.ok():
        try:
            executor.spin()
            return
        except KeyboardInterrupt:
            return
        except Exception as exc:  # noqa: BLE001 - backstop
            if not rclpy.ok():
                return
            node.get_logger().error(f'callback raised {type(exc).__name__}: {exc}; spinning on')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationServerNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        spin_resilient(executor, node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
