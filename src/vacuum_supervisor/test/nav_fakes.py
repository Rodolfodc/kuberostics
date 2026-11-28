"""Shared fakes of the supervisor node tests: a fake Nav2 (`NavigateToPose` server driving `mux/nav`) and waits.

Not a test module (no `test_` prefix); imported by test_supervisor_watchdog.py and test_supervisor_explore.py.
"""
import os
import time

import pytest


def _need(module):
    """importorskip locally; under CI or the docker runner (DISTRO is set) a missing module is a failure."""
    if os.environ.get('CI') or os.environ.get('DISTRO'):
        return __import__(module)
    return pytest.importorskip(module)


rclpy = _need('rclpy')
nav2_msgs = _need('nav2_msgs')

from geometry_msgs.msg import Twist  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vacuum_interfaces.msg import Notification, Submode  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
BUDGET_S = 2.5 + 1.0 + 0.5          # feedback_stale_s + nav_cancel_s + margin (005-R34)


def wait_for(predicate, timeout=30.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _moving():
    msg = Twist()
    msg.linear.x = 0.2
    return msg


class FakeNav2(Node):
    """NavigateToPose server + the velocity of controller_server: non-zero on mux/nav while a goal is active."""

    def __init__(self):
        super().__init__('fake_nav2')
        self.active = False
        self.cancelled = False
        self.closing = False
        self.last_nonzero = None           # monotonic time of the last non-zero command received on mux/nav
        self.submode = None
        self.notifications = []
        self.pub_mode = self.create_publisher(String, '/myRobot/behaviour/mode', QOS)
        self.pub_vel = self.create_publisher(Twist, '/myRobot/mux/nav', 10)
        self.create_subscription(Twist, '/myRobot/mux/nav', self._on_vel, 10)
        self.create_subscription(Submode, '/myRobot/behaviour/submode', self._on_submode, QOS)
        self.create_subscription(Notification, '/myRobot/notifications', self.notifications.append, 10)
        cb = ReentrantCallbackGroup()
        ActionServer(self, NavigateToPose, '/navigate_to_pose', self._navigate, callback_group=cb,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=cb)

    def _on_vel(self, msg):
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.last_nonzero = time.monotonic()

    def _on_submode(self, msg):
        self.submode = msg

    def _navigate(self, goal_handle):
        self.active = True
        while not goal_handle.is_cancel_requested and not self.closing:
            self.pub_vel.publish(_moving())
            time.sleep(0.05)
        self.active = False
        if goal_handle.is_cancel_requested:
            self.cancelled = True
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return NavigateToPose.Result()

    def codes(self):
        return [(n.severity, n.code) for n in self.notifications]
