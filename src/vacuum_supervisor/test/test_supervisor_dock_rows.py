"""Node tests of the supervisor rows of spec 005-R30: cover_room result -> notification, docking request, NONE.

The transition table is covered without ROS in test_submode_machine.py; here we check the wiring: the result
handler of `cover_room`, the injected docking request, the dock timer and the notifications. The docking
mechanism is injected (`dock_requester`), because `myrobot_interfaces` (third party) is not built in CI.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vacuum_interfaces.action import CoverRoom, Escape  # noqa: E402
from vacuum_interfaces.msg import Notification, Submode  # noqa: E402

from vacuum_supervisor.supervisor_node import SupervisorNode  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def wait_for(predicate, timeout=30.0):   # condition wait: returns as soon as true
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class Peer(Node):
    """Orchestrator and the action servers; `finish_cover(success)` ends the running cover_room goal."""

    def __init__(self):
        super().__init__('dock_rows_peer')
        self.cover_goals = 0
        self.closing = False
        self._outcome = None            # None = keep running; True/False = finish with that result
        self.submode = None
        self.submode_count = 0          # one per supervisor tick (publish_rate_hz); a clock for negative checks
        self.notifications = []
        self.dock_requests = 0
        self.pub_mode = self.create_publisher(String, '/myRobot/behaviour/mode', QOS)
        self.create_subscription(Submode, '/myRobot/behaviour/submode', self._on_submode, QOS)
        self.create_subscription(Notification, '/myRobot/notifications', self.notifications.append, 10)
        cb = ReentrantCallbackGroup()
        ActionServer(self, CoverRoom, '/myRobot/cover_room', self._cover, callback_group=cb,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        ActionServer(self, Escape, '/myRobot/escape', self._escape, callback_group=cb,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)

    def _on_submode(self, msg):
        self.submode = msg
        self.submode_count += 1

    def settle(self, ticks=3):
        """Waits for `ticks` more supervisor ticks. Goals are sent from the tick, so after this a goal that the
        supervisor wanted to send has been sent (negative checks wait on this event count, not on wall time)."""
        target = self.submode_count + ticks
        assert wait_for(lambda: self.submode_count >= target), 'the supervisor stopped ticking'
        time.sleep(0.1)                 # only the delivery of an already sent goal to the action server

    def _cover(self, goal_handle):
        self.cover_goals += 1
        self._outcome = None
        while self._outcome is None and not goal_handle.is_cancel_requested and not self.closing:
            time.sleep(0.02)
        if self._outcome is None:
            goal_handle.canceled()
            return CoverRoom.Result()
        if self._outcome:
            goal_handle.succeed()
            return CoverRoom.Result(success=True, covered_ratio=0.95)
        goal_handle.abort()
        return CoverRoom.Result(success=False, covered_ratio=0.3)

    def _escape(self, goal_handle):
        goal_handle.succeed()
        return Escape.Result(success=True, strategy='test')

    def finish_cover(self, success):
        self._outcome = success

    def request_dock(self):
        self.dock_requests += 1

    def set_mode(self, mode):
        assert wait_for(lambda: self.pub_mode.get_subscription_count() >= 1), 'supervisor never subscribed'
        self.pub_mode.publish(String(data=mode))

    def codes(self):
        return [(n.severity, n.code) for n in self.notifications]


@pytest.fixture()
def system():
    """Yields a factory `start(**params)` -> (supervisor, peer); the rows are enabled unless overridden."""
    rclpy.init()                            # the domain is isolated by conftest.py
    created = {}

    def start(**params):
        params.setdefault('dock_after_coverage', True)
        peer = Peer()
        sup = SupervisorNode(dock_requester=peer.request_dock,
                             parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(sup)
        executor.add_node(peer)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        created.update(sup=sup, peer=peer, executor=executor, thread=thread)
        sup.trigger_configure()
        sup.trigger_activate()
        assert wait_for(lambda: peer.submode is not None), 'supervisor never published a submode'
        assert wait_for(lambda: sup._cover_client.server_is_ready()), 'cover_room server not discovered'
        return sup, peer

    try:
        yield start
    finally:
        if created:
            created['peer'].closing = True
            try:
                created['sup'].trigger_deactivate()
                created['sup'].trigger_cleanup()
            except Exception:               # noqa: B902
                pass
            created['executor'].shutdown()
            created['sup'].destroy_node()
            created['peer'].destroy_node()
            created['thread'].join(timeout=2)
        rclpy.try_shutdown()


def start_covering(peer):
    peer.set_mode('CLEANING')
    assert wait_for(lambda: peer.submode.submode == 'COVERING' and peer.cover_goals == 1)


def test_005_R30_cover_success_notifies_done_requests_docking_and_publishes_none(system):
    _, peer = system(dock_request_timeout_s=60.0)
    start_covering(peer)
    peer.finish_cover(True)
    assert wait_for(lambda: (Notification.INFO, 'COVERAGE_DONE') in peer.codes())
    assert wait_for(lambda: peer.submode.submode == 'NONE')
    assert peer.dock_requests == 1
    # the orchestrator keeps publishing CLEANING (every 0.2 s): no new cover_room while coverage_blocked
    for _ in range(3):
        peer.set_mode('CLEANING')
        peer.settle(1)
    assert peer.cover_goals == 1
    assert peer.submode.submode == 'NONE'
    assert peer.dock_requests == 1


def test_005_R30_cover_failure_notifies_warn_and_requests_docking(system):
    _, peer = system(dock_request_timeout_s=60.0)
    start_covering(peer)
    peer.finish_cover(False)
    assert wait_for(lambda: (Notification.WARN, 'COVERAGE_FAILED') in peer.codes())
    assert wait_for(lambda: peer.submode.submode == 'NONE')
    assert wait_for(lambda: peer.dock_requests == 1)


def test_005_R30_dock_timer_expiry_notifies_error_and_sends_no_goal(system):
    _, peer = system(dock_request_timeout_s=0.5)
    start_covering(peer)
    peer.finish_cover(True)
    assert wait_for(lambda: (Notification.ERROR, 'DOCK_REQUEST_FAILED') in peer.codes())
    peer.set_mode('CLEANING')
    peer.settle()
    assert peer.cover_goals == 1
    assert peer.submode.submode == 'NONE'
    assert peer.codes().count((Notification.ERROR, 'DOCK_REQUEST_FAILED')) == 1


def test_005_R30_autodocking_before_the_timer_cancels_it(system):
    sup, peer = system(dock_request_timeout_s=0.6)
    start_covering(peer)
    peer.finish_cover(True)
    assert wait_for(lambda: peer.dock_requests == 1 and sup._dock_timer is not None)
    peer.set_mode('AUTODOCKING')
    assert wait_for(lambda: peer.submode.mode == 'AUTODOCKING')
    # the timer must be gone (a destroyed timer cannot fire), then a few ticks for any late notification
    assert wait_for(lambda: sup._dock_timer is None), 'AUTODOCKING did not cancel the dock timer'
    peer.settle()
    assert (Notification.ERROR, 'DOCK_REQUEST_FAILED') not in peer.codes()
    assert peer.submode.submode == 'NONE'


def test_005_R30_a_new_cleaning_session_covers_again_after_the_mode_left_cleaning(system):
    _, peer = system(dock_request_timeout_s=60.0)
    start_covering(peer)
    peer.finish_cover(True)
    assert wait_for(lambda: peer.submode.submode == 'NONE' and peer.dock_requests == 1)
    peer.set_mode('CHARGING')
    assert wait_for(lambda: peer.submode.mode == 'CHARGING')
    peer.set_mode('CLEANING')
    assert wait_for(lambda: peer.cover_goals == 2)
    assert wait_for(lambda: peer.submode.submode == 'COVERING')


def test_005_R30_rows_are_off_by_default_and_the_001_behaviour_holds(system):
    _, peer = system(dock_after_coverage=False)
    start_covering(peer)
    peer.finish_cover(True)
    peer.settle(4)
    assert peer.submode.submode == 'COVERING'
    assert peer.dock_requests == 0
    assert peer.codes() == []
