"""Node test of the supervisor watchdog of 005-R34, with a fake Nav2 and a fake velocity publisher.

The supervisor and a fake Nav2 (`NavigateToPose` server that publishes a non-zero `Twist` on `mux/nav` while
a goal is active, like `controller_server`) run in this process; the fake `cover_room` server is a child
process that the test kills mid-goal. After the kill there must be no non-zero command on `mux/nav` within
`feedback_stale_s + nav_cancel_s + 0.5 s` (4.0 s with the defaults). This proves only the supervisor's cancel
path; the real Nav2 cancel is shown in the simulator (T510, T517).
"""
import os
import signal
import subprocess
import sys
import threading
import time

import pytest

def _need(module):
    """importorskip locally; under CI or the docker runner (DISTRO is set) a missing module is a failure."""
    if os.environ.get('CI') or os.environ.get('DISTRO'):
        return __import__(module)
    return pytest.importorskip(module)


rclpy = _need('rclpy')
nav2_msgs = _need('nav2_msgs')

from action_msgs.srv import CancelGoal  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vacuum_interfaces.msg import Notification, Submode  # noqa: E402

from vacuum_supervisor.supervisor_node import SupervisorNode  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
FAKE_SERVER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fake_cover_server.py')
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


@pytest.fixture()
def system():
    rclpy.init()                            # the domain is isolated by conftest.py
    created = {'procs': []}

    def start(*server_args, **params):
        fake = FakeNav2()
        sup = SupervisorNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(sup)
        executor.add_node(fake)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        proc = subprocess.Popen([sys.executable, FAKE_SERVER, *server_args])
        created.update(sup=sup, fake=fake, executor=executor, thread=thread)
        created['procs'].append(proc)
        sup.trigger_configure()
        sup.trigger_activate()
        assert wait_for(lambda: sup._cover_client.server_is_ready()), 'cover_room server not discovered'
        assert wait_for(lambda: fake.client.server_is_ready()), 'fake Nav2 not discovered'
        return sup, fake, proc

    try:
        yield start
    finally:
        for proc in created['procs']:
            if proc.poll() is None:
                proc.kill()
            proc.wait(timeout=10)
        if 'sup' in created:
            created['fake'].closing = True
            try:
                created['sup'].trigger_deactivate()
                created['sup'].trigger_cleanup()
            except Exception:               # noqa: B902
                pass
            created['executor'].shutdown()
            created['sup'].destroy_node()
            created['fake'].destroy_node()
            created['thread'].join(timeout=2)
        rclpy.try_shutdown()


def begin_coverage_with_nav_goal(sup, fake):
    assert wait_for(lambda: fake.pub_mode.get_subscription_count() >= 1)
    fake.pub_mode.publish(String(data='CLEANING'))
    assert wait_for(lambda: sup._cover_handle is not None), 'cover_room goal was not accepted'
    fake.client.send_goal_async(NavigateToPose.Goal())          # what coverage_server does
    assert wait_for(lambda: fake.active and fake.last_nonzero is not None), 'fake Nav2 not driving'


@pytest.mark.parametrize('how', ['killed', 'silent'])
def test_005_R34_lost_cover_server_stops_the_nav_velocity_within_the_budget(system, how):
    sup, fake, proc = system(*(['--silent'] if how == 'silent' else []), dock_after_coverage=True,
                             dock_request_timeout_s=60.0)
    begin_coverage_with_nav_goal(sup, fake)
    t0 = time.monotonic()
    if how == 'killed':
        proc.send_signal(signal.SIGKILL)       # the process dies: the Nav2 goal is NOT cancelled by the server
        proc.wait(timeout=10)
        t0 = time.monotonic()
    # (silent: the goal is accepted at the supervisor but the server never sends feedback, so the watch starts
    # at acceptance and the budget is counted from the beginning of the coverage goal)
    assert wait_for(lambda: fake.cancelled, timeout=BUDGET_S + 2.0), 'navigate_to_pose goals were never cancelled'
    assert wait_for(lambda: not fake.active)
    # in-flight Twist messages published before the cancel still arrive: wait until the stream has stopped
    assert wait_for(lambda: time.monotonic() - fake.last_nonzero > 0.3)
    assert fake.last_nonzero - t0 <= BUDGET_S, (
        f'non-zero command {fake.last_nonzero - t0:.2f} s after the {how} (budget {BUDGET_S} s)')
    assert wait_for(lambda: (Notification.WARN, 'COVERAGE_FAILED') in fake.codes()), 'action not treated as failed'
    assert sup._machine.snapshot()['coverage_blocked']


def test_005_R34_a_live_server_with_feedback_is_never_cancelled(system):
    # default 2.5 s limit against a 2 Hz feeder, watched for 2x the limit: a 1 s scheduling stall is tolerated
    sup, fake, proc = system(dock_after_coverage=True)
    begin_coverage_with_nav_goal(sup, fake)
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        assert not fake.cancelled and fake.active
        time.sleep(0.05)
    assert sup._watchdog.armed
    assert fake.codes() == []


def test_005_R34_server_not_ready_cancels_nav_and_fails_the_action(system):
    sup, fake, proc = system(dock_after_coverage=True, feedback_stale_s=60.0)    # staleness out of the way
    begin_coverage_with_nav_goal(sup, fake)
    proc.send_signal(signal.SIGKILL)
    proc.wait(timeout=10)
    # the graph notices the dead process (liveliness) long before the 60 s: the supervisor must not wait for them
    assert wait_for(lambda: fake.cancelled, timeout=40.0), 'server not ready was not treated as lost'
    assert wait_for(lambda: (Notification.WARN, 'COVERAGE_FAILED') in fake.codes())


def test_005_R34_rows_off_the_loss_cancels_nav_and_the_001_behaviour_holds(system):
    sup, fake, proc = system(dock_after_coverage=False)
    begin_coverage_with_nav_goal(sup, fake)
    proc.send_signal(signal.SIGKILL)
    t0 = time.monotonic()
    proc.wait(timeout=10)
    assert wait_for(lambda: fake.cancelled, timeout=BUDGET_S + 2.0)
    assert wait_for(lambda: not fake.active)
    assert wait_for(lambda: time.monotonic() - fake.last_nonzero > 0.3)
    assert fake.last_nonzero - t0 <= BUDGET_S
    assert fake.codes() == []                       # rows off: no notification, no docking request
    assert fake.submode.submode == 'COVERING'


class SilentCancelService(Node):
    """Counts CancelGoal requests on the navigate_to_pose action and never answers."""

    def __init__(self):
        super().__init__('silent_cancel')
        self.requests = 0
        self.release = threading.Event()
        self.create_service(CancelGoal, '/navigate_to_pose/_action/cancel_goal', self._on_request,
                            callback_group=ReentrantCallbackGroup())

    def _on_request(self, request, response):
        self.requests += 1
        self.release.wait(timeout=30.0)             # the answer is held back until the test ends
        return response


def test_005_R34_unanswered_cancel_does_not_block_the_next_loss():
    rclpy.init()
    silent = None
    try:
        sup = SupervisorNode(parameter_overrides=[Parameter('nav_cancel_s', value=0.6)])
        silent = SilentCancelService()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(sup)
        executor.add_node(silent)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        sup.trigger_configure()
        sup.trigger_activate()
        assert wait_for(lambda: sup._nav_cancel_client.service_is_ready())
        sup._on_cover_lost('feedback_stale', time.monotonic())
        assert wait_for(lambda: silent.requests >= 1)
        assert wait_for(lambda: not sup._cancel_window.pending, timeout=5.0), 'window never expired'
        before = silent.requests
        sup._on_cover_lost('feedback_stale', time.monotonic())     # second loss, the first was never answered
        assert wait_for(lambda: silent.requests > before), 'second loss sent no cancel (stuck in-flight guard)'
    finally:
        if silent is not None:
            silent.release.set()
        sup.trigger_deactivate()
        sup.trigger_cleanup()
        executor.shutdown()
        sup.destroy_node()
        silent.destroy_node()
        thread.join(timeout=2)
        rclpy.try_shutdown()
