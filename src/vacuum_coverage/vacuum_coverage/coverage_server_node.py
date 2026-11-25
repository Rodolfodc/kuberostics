"""`coverage_server` LifecycleNode (T511, 005-R16 to R22, R26, R27, R31, R32, R34, R36): CoverRoom action
server on /myRobot/cover_room that sends the lanes to Nav2 as NavigateToPose goals.

Thin glue over `CoverageRun` (pure Python, D-18). The node publishes NO velocity (D-24, 005-R19): Nav2's
controller_server is the only coverage actuator and drives on /myRobot/mux/nav. It owns the Nav2 goal only to
cancel it (goal cancel within 0.5 s, deactivate and shutdown, 005-R22 and R31).

Fixed by design, not parameters (the contract lists none): TF frames `map` and `base_link`.
Room topics (/myRobot/map/room_labels, /myRobot/map/rooms) wait for the Room messages of spec 003 (T303), so
a non-empty `room_id` is rejected as unknown for now (005-R16).
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
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from vacuum_interfaces.action import CoverRoom

from vacuum_coverage.core.coverage_run import ConfigError, CoverageConfig, CoverageRun
from vacuum_coverage.core.coverage_session import StepKind
from vacuum_coverage.core.region_builder import NoRegion, UnknownRoom
from vacuum_mapping.core.types import Raster, classify_occupancy

MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
LOOP_PERIOD_S = 0.05            # decision loop of a goal; bounds the reaction time of cancel and timeouts
FEEDBACK_PERIOD_S = 0.5         # 005-R34: 1 Hz or faster
COVERED_PERIOD_S = 1.0
NAV_CANCEL_WAIT_S = 0.5

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

CONFIG_PARAMS = {
    'lane_spacing_m': 0.25, 'robot_radius_m': 0.16, 'margin_m': 0.05, 'min_segment_m': 0.30,
    'tool_radius_m': 0.15, 'target_ratio': 0.95, 'min_acceptable_ratio': 0.70, 'max_skip_ratio': 0.3,
    'coverage_timeout_s': 1800.0, 'waypoint_timeout_s': 120.0, 'resume_window_s': 120.0,
    'max_region_area_m2': 150.0,
}


def grid_to_raster(msg: OccupancyGrid) -> Raster:
    cells = classify_occupancy(np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width))
    return Raster(cells, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)


def covered_to_grid(raster: Raster, data: np.ndarray, stamp) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id = MAP_FRAME
    msg.header.stamp = stamp
    msg.info.resolution = raster.resolution
    msg.info.width, msg.info.height = raster.width, raster.height
    msg.info.origin.position.x, msg.info.origin.position.y = raster.origin_x, raster.origin_y
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(v) for v in data.reshape(-1)]
    return msg


class _Rejected(Exception):
    """A goal is refused before any state of the run is touched."""


class CoverageServerNode(LifecycleNode):
    def __init__(self, node_name: str = 'coverage_server', **kwargs):
        super().__init__(node_name, **kwargs)
        for name, default in CONFIG_PARAMS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('telemetry_rate_hz', 1.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self._group = ReentrantCallbackGroup()
        self._raster: Optional[Raster] = None
        self._active = False
        self._generation = 0
        self._busy = False
        self._goal_lock = threading.Lock()
        self._run: Optional[CoverageRun] = None
        self._pending_run: Optional[CoverageRun] = None
        self._nav_lock = threading.Lock()
        self._nav_seq = 0
        self._nav_handle = None
        self._nav_outcome: Optional[bool] = None
        self._wake = threading.Event()
        self._timer = None
        self._counters = {'goals': 0, 'succeeded': 0, 'failed': 0, 'cancelled': 0, 'degraded': 0,
                          'resumed': 0, 'rejected': 0}
        self._state = 'unconfigured'
        self._ratio = 0.0
        self._snapshot = {}
        self._max_poll_gap_s = 0.0
        self._pose_stats = {'pose_samples': 0, 'max_pose_hop_m': 0.0, 'unbridged_hops': 0}

    # ---- lifecycle ----
    def _LifecycleNodeMixin__on_change_state(self, request, response):  # noqa: N802 - overrides the rclpy callback
        """A redundant or impossible transition (activate while active, configure while configured) must answer
        success=false. Lyrical's rclpy does that; Jazzy's raises out of the service callback, which used to end
        the process (found in the launch test). The service is registered with this attribute, so the override
        is picked up on both distros; if a future rclpy renames it, `spin_resilient` is the backstop."""
        try:
            return super()._LifecycleNodeMixin__on_change_state(request, response)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'transition {request.transition.id} refused in state {self._state}: {type(exc).__name__}')
            response.success = False
            return response
    def on_configure(self, state) -> TransitionCallbackReturn:
        values = {name: self.get_parameter(name).value for name in CONFIG_PARAMS}
        try:
            self._config = CoverageConfig(**values)
            self._config.validate()
        except (ConfigError, TypeError) as exc:
            self.get_logger().error(f'configure failed: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._run = CoverageRun(self._config, log=self.get_logger().info)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._sub_map = self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self._on_map, LATCHED,
            callback_group=self._group)
        self._nav = ActionClient(self, NavigateToPose, self.get_parameter('nav_action').value,
                                 callback_group=self._group)
        self._pub_path = self.create_lifecycle_publisher(Path, '/myRobot/coverage/path', 10)
        self._pub_covered = self.create_lifecycle_publisher(OccupancyGrid, '/myRobot/coverage/covered', LATCHED)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/coverage_server', 10)
        self._server = ActionServer(
            self, CoverRoom, '/myRobot/cover_room', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel, callback_group=self._group)
        self._set_state('inactive')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._generation += 1
        self._active = True
        self._timer = self.create_timer(1.0 / self.get_parameter('telemetry_rate_hz').value, self._telemetry,
                                        callback_group=self._group)
        self._set_state('idle')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False                # a running goal aborts on its next iteration
        self._generation += 1
        self._cancel_nav(wait=True)         # 005-R31: the Nav2 goal goes before the node stops
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
        self.destroy_publisher(self._pub_path)
        self.destroy_publisher(self._pub_covered)
        self.destroy_publisher(self._pub_telemetry)
        self._tf_listener.unregister()
        self._raster = None
        self._set_state('unconfigured')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self._active = False
        self._generation += 1
        self._cancel_nav(wait=True)         # 005-R31 also on shutdown
        self._wake.set()
        return TransitionCallbackReturn.SUCCESS

    def _set_state(self, new: str) -> None:
        if new != self._state:
            self.get_logger().info(f'state {self._state} -> {new}')
            self._state = new

    # ---- inputs -----------------------------------------------------------------------------------------
    @property
    def has_map(self) -> bool:
        return self._raster is not None

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._raster = grid_to_raster(msg)
        self.get_logger().info(f'map received {msg.info.width}x{msg.info.height} at {msg.info.resolution} m')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time(), Duration(seconds=0.0))
        except Exception:
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
        with self._goal_lock:               # check and claim are one step: two simultaneous goals cannot both pass
            if self._busy:
                return self._reject('another goal is active')
            self._busy = True
        try:
            if self._raster is None:
                raise _Rejected('no map received yet')
            if not self._nav.server_is_ready():
                raise _Rejected(f'{self.get_parameter("nav_action").value} is not available (005-R36)')
            pose = self._pose()
            if pose is None:
                raise _Rejected(f'no TF {MAP_FRAME} -> {BASE_FRAME}')
            # begin() consumes the saved session only when this goal is accepted (it validates first)
            resumed = self._run.begin(self._now(), self._raster, pose, goal.room_id, max(1, int(goal.passes)))
        except (_Rejected, UnknownRoom, NoRegion) as exc:
            self._busy = False
            return self._reject(str(exc))
        except Exception:
            self._busy = False
            raise
        if resumed:
            self._counters['resumed'] += 1
        self.get_logger().info(
            f'goal accepted room_id={goal.room_id!r} passes={goal.passes} region={self._run.region_source} '
            f'waypoints={len(self._run.session.waypoints)} resumed={resumed}')
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('cancel requested: cancelling the Nav2 goal (005-R22)')
        self._cancel_nav()
        self._wake.set()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        run = self._run
        generation = self._generation
        self._counters['goals'] += 1
        self._set_state('covering')
        self._publish_path(0)
        shown_pass = 0
        last_poll = 0.0
        self._max_poll_gap_s = 0.0
        next_feedback = next_covered = 0.0
        result = CoverRoom.Result(success=False, covered_ratio=0.0)
        try:
            while rclpy.ok():
                t = self._now()
                if goal_handle.is_cancel_requested:
                    self._cancel_nav()
                    run.suspend(t)
                    self._counters['cancelled'] += 1
                    result.covered_ratio = float(run.covered_ratio)
                    goal_handle.canceled()
                    self.get_logger().info(f'goal cancelled ratio={result.covered_ratio:.3f}, kept for resume')
                    return result
                if not self._active or generation != self._generation:
                    self._cancel_nav()
                    run.end()
                    self._counters['failed'] += 1
                    result.covered_ratio = float(run.covered_ratio)
                    goal_handle.abort()
                    self.get_logger().warning('goal aborted: node deactivated')
                    return result
                pose = self._pose()
                if pose is not None:
                    run.on_pose(*pose)
                    self._max_poll_gap_s = max(self._max_poll_gap_s, t - last_poll) if last_poll else 0.0
                    last_poll = t
                with self._nav_lock:
                    outcome, self._nav_outcome = self._nav_outcome, None
                if outcome is not None:
                    run.goal_result(t, outcome)
                step = run.step(t)
                self._ratio = float(run.covered_ratio)
                self._snapshot = run.session.snapshot()
                self._pose_stats = {'pose_samples': run.pose_samples, 'max_pose_hop_m': round(run.max_hop_m, 3),
                                    'unbridged_hops': run.unbridged_hops}
                if step.kind is StepKind.SEND:
                    self._send_nav(step.waypoint)
                    if step.waypoint.pass_index != shown_pass:
                        shown_pass = step.waypoint.pass_index
                        self._publish_path(shown_pass)
                elif step.kind is StepKind.CANCEL:
                    self._cancel_nav()
                elif step.kind is StepKind.DONE:
                    if step.cancel_goal:
                        self._cancel_nav()
                    return self._finish(goal_handle, run, step)
                if t >= next_feedback:
                    goal_handle.publish_feedback(CoverRoom.Feedback(covered_ratio=self._ratio))
                    next_feedback = t + FEEDBACK_PERIOD_S
                if t >= next_covered:
                    self._publish_covered(run)
                    next_covered = t + COVERED_PERIOD_S
                self._wake.wait(LOOP_PERIOD_S)
                self._wake.clear()
            self._cancel_nav()
            goal_handle.abort()
            return result
        finally:
            self._busy = False
            self._set_state('idle' if self._active else 'inactive')

    def _finish(self, goal_handle, run, step) -> CoverRoom.Result:
        run.end()
        self._publish_covered(run)
        result = CoverRoom.Result(success=bool(step.success), covered_ratio=float(run.covered_ratio))
        self._ratio = result.covered_ratio
        if step.success:
            self._counters['succeeded'] += 1
            if step.degraded:
                self._counters['degraded'] += 1
                self.get_logger().warning(
                    f'degraded success: covered_ratio={result.covered_ratio:.3f} below target '
                    f'{self._config.target_ratio} (reason={step.reason})')
            goal_handle.succeed()
        else:
            self._counters['failed'] += 1
            goal_handle.abort()
        self.get_logger().info(f'goal done success={result.success} reason={step.reason} '
                               f'covered_ratio={result.covered_ratio:.3f} skipped={run.session.skipped_waypoints}')
        return result

    # ---- Nav2 goal --------------------------------------------------------------------------------------
    def _send_nav(self, wp) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(wp.x), float(wp.y)
        goal.pose.pose.orientation.z = math.sin(wp.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(wp.yaw / 2.0)
        with self._nav_lock:
            self._nav_seq += 1
            seq = self._nav_seq
            self._nav_outcome = None
        self.get_logger().info(f'nav goal {seq}: ({wp.x:.2f}, {wp.y:.2f}, yaw {wp.yaw:.2f})')
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
    def _publish_path(self, pass_index: int) -> None:
        path = Path()
        path.header.frame_id = MAP_FRAME
        path.header.stamp = self.get_clock().now().to_msg()
        for wp in self._run.session.waypoints:
            if wp.pass_index != pass_index:
                continue
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(wp.x), float(wp.y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(wp.yaw / 2.0), math.cos(wp.yaw / 2.0)
            path.poses.append(pose)
        self._pub_path.publish(path)

    def _publish_covered(self, run) -> None:
        self._pub_covered.publish(covered_to_grid(run.session.tracker.raster, run.covered_grid(),
                                                  self.get_clock().now().to_msg()))

    def _telemetry(self) -> None:
        """D-20: state and counters at telemetry_rate_hz."""
        values = dict(self._counters)
        values['covered_ratio'] = round(self._ratio, 3)
        values['skipped_waypoints'] = self._snapshot.get('skipped_waypoints', 0)
        values['waypoints_total'] = self._snapshot.get('waypoints_total', 0)
        values['current_index'] = self._snapshot.get('current_index', 0)
        values.update(self._pose_stats)
        values['max_poll_gap_s'] = round(self._max_poll_gap_s, 3)
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
        except Exception as exc:  # noqa: BLE001 - backstop, the guarded callbacks should not get here
            if not rclpy.ok():
                return
            node.get_logger().error(f'callback raised {type(exc).__name__}: {exc}; spinning on')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageServerNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        spin_resilient(executor, node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
