"""Fakes shared by the node and launch tests: a NavigateToPose server that moves a fake robot (TF map ->
base_link) and a latched /map publisher. No Nav2, no simulator."""
import math
import threading
import time

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


def wait_for(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def raster_to_grid(raster, stamp=None) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = raster.resolution
    msg.info.width, msg.info.height = raster.width, raster.height
    msg.info.origin.position.x, msg.info.origin.position.y = raster.origin_x, raster.origin_y
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(v) for v in raster.cells.reshape(-1)]
    return msg


class FakeNav(Node):
    """`script(index, x, y)` returns REACH, ABORT or HANG for the index-th goal received (0-based).
    REACH moves the fake robot in 0.05 m steps (about 5 m/s) and succeeds; HANG waits for a cancel."""

    def __init__(self, script, start=(1.0, 1.0), name='fake_nav2'):
        super().__init__(name)
        self.script = script
        self.pose = list(start)
        self.goals = []              # (x, y) of every goal received
        self.cancel_times = []       # time.monotonic() of every cancel request
        self.active = False
        self._lock = threading.Lock()
        self._tf = TransformBroadcaster(self)
        self.create_timer(0.01, self._publish_tf)
        self.server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose', execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT, cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())

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
                while not goal_handle.is_cancel_requested:
                    time.sleep(0.005)
                goal_handle.canceled()
                return NavigateToPose.Result()
            if outcome == ABORT:
                goal_handle.abort()
                return NavigateToPose.Result()
            sx, sy = self.pose
            n = max(1, int(math.hypot(p.x - sx, p.y - sy) / 0.05))
            for i in range(1, n + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                self.pose[:] = [sx + (p.x - sx) * i / n, sy + (p.y - sy) * i / n]
                time.sleep(0.004)
            goal_handle.succeed()
            return NavigateToPose.Result()
        finally:
            self.active = False


class MapPublisher(Node):
    def __init__(self, raster, name='map_publisher'):
        super().__init__(name)
        self.pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.pub.publish(raster_to_grid(raster))
