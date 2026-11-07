"""T009: tests of the `supervisor` LifecycleNode against stub action servers (001-R05 to 001-R09).

Runs the real node in-process with rclpy: no simulator, no launch file. The transition logic itself is
covered without ROS by test_submode_machine.py; here we check the wiring (topics, actions, service, lock,
notifications, timeout). Skipped when rclpy is not available.
"""
import os
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from vacuum_interfaces.action import CoverRoom, Escape  # noqa: E402
from vacuum_interfaces.msg import Notification, StuckEvent, Submode  # noqa: E402

from vacuum_supervisor.supervisor_node import SupervisorNode  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


class Harness(Node):
    """Plays the rest of the system: orchestrator, stuck detector and the two action servers."""

    def __init__(self):
        super().__init__('test_harness')
        cb = ReentrantCallbackGroup()
        self.escape_outcome = 'succeed'
        self.closing = False            # lets the stub servers return at teardown
        self.cover_goals = 0
        self.escape_goals = 0
        self.submode = None
        self.lock = None
        self.notifications = []
        self.telemetry = None
        self.pub_mode = self.create_publisher(String, '/myRobot/behaviour/mode', QOS)
        self.pub_stuck = self.create_publisher(StuckEvent, '/myRobot/stuck', 10)
        self.create_subscription(Submode, '/myRobot/behaviour/submode', lambda m: setattr(self, 'submode', m), QOS)
        self.create_subscription(Bool, '/myRobot/mux/lock_stop', lambda m: setattr(self, 'lock', m.data), QOS)
        self.create_subscription(Notification, '/myRobot/notifications', self.notifications.append, 10)
        self.create_subscription(DiagnosticArray, '/myRobot/telemetry/supervisor',
                                 lambda m: setattr(self, 'telemetry', m), 10)
        self.reset_client = self.create_client(Trigger, '/supervisor/reset')
        ActionServer(self, CoverRoom, '/myRobot/cover_room', self._cover, callback_group=cb,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        ActionServer(self, Escape, '/myRobot/escape', self._escape, callback_group=cb,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)

    def _cover(self, goal_handle):
        self.cover_goals += 1
        while not goal_handle.is_cancel_requested and not self.closing:     # coverage runs until cancelled
            time.sleep(0.05)
        goal_handle.canceled()
        return CoverRoom.Result()

    def _escape(self, goal_handle):
        self.escape_goals += 1
        if self.escape_outcome == 'hang':
            while not goal_handle.is_cancel_requested and not self.closing:
                time.sleep(0.05)
            goal_handle.canceled()
            return Escape.Result()
        result = Escape.Result(success=self.escape_outcome == 'succeed', strategy='test')
        goal_handle.succeed()
        return result

    def set_mode(self, mode):
        self.pub_mode.publish(String(data=mode))


def wait_for(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture()
def system():
    os.environ.setdefault('ROS_DOMAIN_ID', '87')
    rclpy.init()
    sup = SupervisorNode(parameter_overrides=[Parameter('escape_timeout_s', value=0.6)])
    harness = Harness()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(sup)
    executor.add_node(harness)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    sup.trigger_configure()
    sup.trigger_activate()
    assert wait_for(lambda: harness.submode is not None), 'supervisor never published a submode'
    yield sup, harness
    harness.closing = True
    sup.trigger_deactivate()
    sup.trigger_cleanup()
    executor.shutdown()
    sup.destroy_node()
    harness.destroy_node()
    rclpy.try_shutdown()
    thread.join(timeout=2)


def cover_until_covering(h):
    h.set_mode('CLEANING')
    assert wait_for(lambda: h.submode.submode == 'COVERING')
    assert wait_for(lambda: h.cover_goals == 1)


def test_001_R05_publishes_none_outside_cleaning(system):
    _, h = system
    h.set_mode('IDLE')
    assert wait_for(lambda: h.submode.mode == 'IDLE')
    assert h.submode.submode == 'NONE'


def test_001_R06_cleaning_publishes_covering_and_sends_cover_room(system):
    _, h = system
    start = time.monotonic()
    cover_until_covering(h)
    assert time.monotonic() - start < 2.0


def test_001_R07_R08_stuck_escapes_then_returns_to_covering(system):
    _, h = system
    cover_until_covering(h)
    h.pub_stuck.publish(StuckEvent())
    assert wait_for(lambda: h.escape_goals == 1)
    assert wait_for(lambda: h.cover_goals == 2 and h.submode.submode == 'COVERING')
    assert h.lock is False


def test_001_R09_failed_escape_locks_and_notifies(system):
    _, h = system
    h.escape_outcome = 'fail'
    cover_until_covering(h)
    h.pub_stuck.publish(StuckEvent())
    assert wait_for(lambda: h.submode.submode == 'ESCAPE_FAILED')
    assert wait_for(lambda: h.lock is True)
    assert wait_for(lambda: any(n.code == 'ESCAPE_FAILED' and n.severity == Notification.ERROR
                                for n in h.notifications))


def test_001_R09_escape_timeout_fails_the_escape(system):
    _, h = system
    h.escape_outcome = 'hang'
    cover_until_covering(h)
    h.pub_stuck.publish(StuckEvent())
    assert wait_for(lambda: h.submode.submode == 'ESCAPING')
    assert wait_for(lambda: h.submode.submode == 'ESCAPE_FAILED', timeout=3.0)
    assert wait_for(lambda: h.lock is True)


def test_001_R09_reset_service_releases_the_lock(system):
    _, h = system
    h.escape_outcome = 'fail'
    cover_until_covering(h)
    h.pub_stuck.publish(StuckEvent())
    assert wait_for(lambda: h.lock is True)
    assert h.reset_client.wait_for_service(timeout_sec=3.0)
    future = h.reset_client.call_async(Trigger.Request())
    assert wait_for(future.done)
    assert future.result().success
    assert wait_for(lambda: h.lock is False and h.submode.submode == 'NONE')


def test_D20_telemetry_reports_counters(system):
    _, h = system
    cover_until_covering(h)
    h.pub_stuck.publish(StuckEvent())
    assert wait_for(lambda: h.telemetry is not None and any(
        kv.key == 'stuck_events' and kv.value == '1' for kv in h.telemetry.status[0].values), timeout=5.0)
