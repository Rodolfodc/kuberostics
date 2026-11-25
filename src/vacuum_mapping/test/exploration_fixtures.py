"""Fakes for the exploration node and launch tests: a NavigateToPose server that moves a fake robot (TF map ->
base_link), a latched /map publisher whose map can change, and synthetic grids. No Nav2, no simulator.
(Same shape as the fakes of vacuum_coverage; they cannot be imported from there: that package depends on this one.)"""
import math
import threading
import time

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

REACH, ABORT, HANG = 'reach', 'abort', 'hang'
RES = 0.05


def wait_for(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def _room(left_unknown: bool, right_unknown: bool) -> np.ndarray:
    cells = np.zeros((30, 60), dtype=np.int8)
    cells[0, :] = 100
    cells[-1, :] = 100
    cells[:, 0] = 100 if not left_unknown else 0
    cells[:, -1] = 100 if not right_unknown else 0
    if left_unknown:
        cells[1:-1, :3] = -1
    if right_unknown:
        cells[1:-1, -3:] = -1
    return cells


def known_cells() -> np.ndarray:
    return _room(False, False)


def one_frontier_cells() -> np.ndarray:
    cells = _room(False, True)
    cells[:, 0] = 100
    return cells


def two_frontier_cells() -> np.ndarray:
    return _room(True, True)


def to_grid(cells: np.ndarray) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = RES
    msg.info.height, msg.info.width = cells.shape
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(v) for v in cells.reshape(-1)]
    return msg


class FakeNav(Node):
    """`script(index, x, y)` returns REACH, ABORT or HANG for the index-th goal received (0-based).
    REACH moves the fake robot (`step_m` per `step_period_s`) and succeeds, then calls `on_reach(index)`;
    HANG waits for a cancel."""

    def __init__(self, script, start=(0.5, 0.7), name='fake_nav2', context=None, step_m=0.1, step_period_s=0.01,
                 on_reach=None):
        super().__init__(name, context=context)
        self.script = script
        self.on_reach = on_reach
        self.step_m, self.step_period_s = step_m, step_period_s
        self.pose = list(start)
        self.goals = []              # (x, y) of every goal received
        self.cancel_times = []       # time.monotonic() of every cancel request
        self.active = False
        self.stopped = False
        self._lock = threading.Lock()
        self._tf = TransformBroadcaster(self)
        self.create_timer(0.01, self._publish_tf)
        self.server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose', execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT, cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())

    def destroy_node(self):
        self.stopped = True          # a HANG goal must not keep an executor thread alive after the test
        return super().destroy_node()

    def _publish_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = self.pose
        tf.transform.rotation.w = 1.0
        self._tf.sendTransform(tf)

    def _on_cancel(self, _):
        self.cancel_times.append(time.monotonic())
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        p = goal_handle.request.pose.pose.position
        with self._lock:
            index = len(self.goals)
            self.goals.append((p.x, p.y))
        outcome = self.script(index, p.x, p.y)
        self.active = True
        try:
            if outcome == HANG:
                while not goal_handle.is_cancel_requested and not self.stopped:
                    time.sleep(0.005)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return NavigateToPose.Result()
            if outcome == ABORT:
                goal_handle.abort()
                return NavigateToPose.Result()
            sx, sy = self.pose
            n = max(1, int(math.hypot(p.x - sx, p.y - sy) / self.step_m))
            for i in range(1, n + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                self.pose[:] = [sx + (p.x - sx) * i / n, sy + (p.y - sy) * i / n]
                time.sleep(self.step_period_s)
            if self.on_reach is not None:
                self.on_reach(index)
            goal_handle.succeed()
            return NavigateToPose.Result()
        finally:
            self.active = False


class MapPublisher(Node):
    def __init__(self, cells, name='map_publisher', context=None):
        super().__init__(name, context=context)
        self.pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.publish(cells)

    def publish(self, cells):
        self.pub.publish(to_grid(cells))
