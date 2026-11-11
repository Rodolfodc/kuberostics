"""T021: the supervisor does not treat a not yet discovered escape server as an immediate failed escape.

Regression of a race found by the node tests: `_send_escape` checked `server_is_ready()` once and, on a miss,
reported a failed escape (lock + ESCAPE_FAILED, 001-R09) although the server was only seconds from being
discovered. Now the goal waits (without blocking the executor) at most `escape_server_wait_s`, and the
`escape_timeout_s` clock of 001-R09 is not extended. The transition table is unchanged (test_submode_machine.py).
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
from std_msgs.msg import Bool, String  # noqa: E402
from vacuum_interfaces.action import Escape  # noqa: E402
from vacuum_interfaces.msg import Notification, StuckEvent, Submode  # noqa: E402

from vacuum_supervisor.supervisor_node import SupervisorNode  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def wait_for(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class Peer(Node):
    """Orchestrator and stuck detector; the Escape server is created on demand (`start_escape_server`)."""

    def __init__(self, escape_action):
        super().__init__('race_peer')
        self.escape_action = escape_action
        self.submodes = []
        self.lock = None
        self.notifications = []
        self.t_stuck = None            # monotonic time just before the StuckEvent is published
        self.escape_goals = 0
        self._server = None
        self.pub_mode = self.create_publisher(String, '/myRobot/behaviour/mode', QOS)
        self.pub_stuck = self.create_publisher(StuckEvent, '/myRobot/stuck', 10)
        self.t_failed = None           # monotonic time at which ESCAPE_FAILED was first observed
        self.create_subscription(Submode, '/myRobot/behaviour/submode', self._on_submode, QOS)
        self.create_subscription(Bool, '/myRobot/mux/lock_stop', lambda m: setattr(self, 'lock', m.data), QOS)
        self.create_subscription(Notification, '/myRobot/notifications', self.notifications.append, 10)

    def _on_submode(self, msg):
        self.submodes.append(msg.submode)
        if msg.submode == 'ESCAPE_FAILED' and self.t_failed is None:
            self.t_failed = time.monotonic()

    def start_escape_server(self):
        self._server = ActionServer(self, Escape, self.escape_action, self._escape,
                                    callback_group=ReentrantCallbackGroup(),
                                    cancel_callback=lambda _: CancelResponse.ACCEPT)

    def _escape(self, goal_handle):
        self.escape_goals += 1
        goal_handle.succeed()
        return Escape.Result(success=True, strategy='test')

    @property
    def submode(self):
        return self.submodes[-1] if self.submodes else None


@pytest.fixture()
def system():
    """Yields a factory: `start(params)` returns (supervisor, peer) after a StuckEvent sent with no escape server."""
    created = []

    def start(params):
        rclpy.init()
        # a unique action name per test: the departure of the previous test's server may still be in the graph
        action = f'/myRobot/escape_race_{time.monotonic_ns()}'
        params = dict(params, escape_action=action)
        sup = SupervisorNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        peer = Peer(action)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(sup)
        executor.add_node(peer)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        created.append((sup, peer, executor, thread))
        sup.trigger_configure()
        sup.trigger_activate()
        assert wait_for(lambda: peer.submode is not None)
        assert wait_for(lambda: peer.pub_mode.get_subscription_count() >= 1
                        and peer.pub_stuck.get_subscription_count() >= 1)
        peer.pub_mode.publish(String(data='CLEANING'))
        assert wait_for(lambda: peer.submode == 'COVERING')
        assert not sup._escape_client.server_is_ready()
        peer.t_stuck = time.monotonic()
        peer.pub_stuck.publish(StuckEvent())
        return sup, peer

    yield start
    for sup, peer, executor, thread in created:
        sup.trigger_deactivate()
        sup.trigger_cleanup()
        executor.shutdown()
        sup.destroy_node()
        peer.destroy_node()
        rclpy.try_shutdown()
        thread.join(timeout=2)


def test_001_R07_late_escape_server_is_waited_for_instead_of_failing_the_escape(system):
    sup, peer = system({'escape_timeout_s': 15.0, 'escape_server_wait_s': 10.0})
    assert wait_for(lambda: peer.submode == 'ESCAPING')
    time.sleep(1.0)                                    # the server is not there: the supervisor keeps waiting
    assert peer.submode == 'ESCAPING' and peer.lock is not True
    peer.start_escape_server()
    assert wait_for(lambda: peer.escape_goals == 1), 'the escape goal was never sent'
    assert wait_for(lambda: peer.submode == 'COVERING')
    assert 'ESCAPE_FAILED' not in peer.submodes
    assert peer.lock is not True
    assert sup._pending_escape_deadline is None


def test_001_R09_escape_server_never_discovered_fails_after_escape_server_wait_s(system):
    sup, peer = system({'escape_timeout_s': 15.0, 'escape_server_wait_s': 0.8})
    assert wait_for(lambda: peer.t_failed is not None, timeout=10.0)   # far before the 15 s timeout
    # measured from the stuck event, so startup jitter cannot matter; an immediate failure (the original bug) would
    # give ~0 s here
    assert 0.8 - 0.1 <= peer.t_failed - peer.t_stuck <= 0.8 + 3.0
    assert wait_for(lambda: peer.lock is True)
    assert wait_for(lambda: any(n.code == 'ESCAPE_FAILED' and n.severity == Notification.ERROR
                                for n in peer.notifications))
    assert sup._pending_escape_deadline is None


def test_001_R09_escape_timeout_clock_is_not_extended_by_the_server_wait(system):
    """escape_timeout_s < escape_server_wait_s and the server never appears: exactly one failure, no stale goal."""
    sup, peer = system({'escape_timeout_s': 0.6, 'escape_server_wait_s': 30.0})
    assert wait_for(lambda: peer.t_failed is not None, timeout=10.0)   # far before the 30 s wait
    assert 0.6 - 0.1 <= peer.t_failed - peer.t_stuck <= 0.6 + 3.0
    assert sup._pending_escape_deadline is None, 'the wait outlived the escape timeout'
    peer.start_escape_server()
    assert wait_for(lambda: sup._escape_client.server_is_ready())
    time.sleep(1.0)                                    # several ticks pass
    assert peer.escape_goals == 0, 'a stale goal was sent after the escape had already timed out'
    assert sup._machine.snapshot()['escape_timeouts'] == 1
    assert sup._machine.snapshot()['escapes_failed'] == 1
    assert peer.submode == 'ESCAPE_FAILED'


def test_001_R05_a_mode_change_cancels_the_pending_escape_wait(system):
    """_cancel_actions clears the pending wait: leaving CLEANING must not send an escape goal afterwards."""
    sup, peer = system({'escape_timeout_s': 15.0, 'escape_server_wait_s': 30.0})
    assert wait_for(lambda: peer.submode == 'ESCAPING')
    assert sup._pending_escape_deadline is not None
    peer.pub_mode.publish(String(data='CHARGING'))
    assert wait_for(lambda: sup._pending_escape_deadline is None)
    peer.start_escape_server()
    assert wait_for(lambda: sup._escape_client.server_is_ready())
    time.sleep(1.0)
    assert peer.escape_goals == 0
