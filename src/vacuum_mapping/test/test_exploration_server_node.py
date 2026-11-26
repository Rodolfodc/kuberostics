"""exploration_server LifecycleNode wiring, in-process rclpy against a fake NavigateToPose server (no Nav2).
Requirements: 003-R10 to R13, R26, R31, R32. No fixed sleeps: every wait polls a condition.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.action.client import GoalStatus  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.lifecycle import TransitionCallbackReturn  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from vacuum_interfaces.action import Explore  # noqa: E402
from vacuum_interfaces.msg import Notification  # noqa: E402

from exploration_fixtures import (ABORT, HANG, LATCHED, REACH, FakeNav, MapPublisher, known_cells,  # noqa: E402
                                  one_frontier_cells, two_frontier_cells, wait_for)
from vacuum_mapping.exploration_server_node import ExplorationServerNode  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
MUX_INPUTS = ['teleop', 'stop', 'escape', 'nav', 'coverage', 'docking']


class System:
    def __init__(self, script=None, cells=None, params=(), with_nav=True, with_map=True, on_reach=None):
        rclpy.init()
        self.node = ExplorationServerNode(parameter_overrides=[Parameter(k, value=v) for k, v in params])
        self.probe = Node('exploration_probe')
        self.nodes = [self.node, self.probe]
        self.nav = FakeNav(script or (lambda i, x, y: REACH), on_reach=on_reach) if with_nav else None
        if self.nav:
            self.nodes.append(self.nav)
        self.maps = MapPublisher(one_frontier_cells() if cells is None else cells) if with_map else None
        if self.maps:
            self.nodes.append(self.maps)
        self.executor = MultiThreadedExecutor(num_threads=8)
        for n in self.nodes:
            self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.twists = []
        for name in MUX_INPUTS:
            self.probe.create_subscription(Twist, '/myRobot/mux/' + name, self.twists.append, QOS)
        self.telemetry, self.notes, self.explored = [], [], []
        self.probe.create_subscription(DiagnosticArray, '/myRobot/telemetry/exploration_server',
                                       self.telemetry.append, 10)
        self.probe.create_subscription(Notification, '/myRobot/notifications', self.notes.append, 10)
        self.probe.create_subscription(Bool, '/myRobot/map/explored', lambda m: self.explored.append(m.data), LATCHED)
        self.client = ActionClient(self.probe, Explore, '/myRobot/explore')
        self.feedback = []

    def configure(self):
        assert self.node.trigger_configure() == TransitionCallbackReturn.SUCCESS

    def up(self):
        self.configure()
        assert self.node.trigger_activate() == TransitionCallbackReturn.SUCCESS
        assert self.client.wait_for_server(timeout_sec=10.0)
        if self.maps and self.nav:      # inputs of a goal: the latched map, the Nav2 server and the first TF
            assert wait_for(lambda: self.node._grid is not None and self.node._nav.server_is_ready()
                            and self.node._pose() is not None, 10.0)

    def send(self, timeout_s=0.0):
        future = self.client.send_goal_async(
            Explore.Goal(timeout_s=timeout_s),
            feedback_callback=lambda m: self.feedback.append(time.monotonic()))
        assert wait_for(future.done, 10.0)
        return future.result()

    def result(self, handle, timeout=30.0):
        future = handle.get_result_async()
        assert wait_for(future.done, timeout), 'no result'
        return future.result()

    def values(self):
        msg = self.telemetry[-1] if self.telemetry else None
        return {kv.key: kv.value for kv in msg.status[0].values} if msg else {}

    def close(self):
        if self.nav:
            self.nav.stopped = True     # a HANG goal would keep an executor thread (and its join) alive
        self.executor.shutdown()
        self.thread.join(timeout=3)
        for n in self.nodes:
            n.destroy_node()
        rclpy.shutdown()


@pytest.fixture()
def make():
    systems = []

    def factory(*args, **kwargs):
        s = System(*args, **kwargs)
        systems.append(s)
        return s
    yield factory
    for s in systems:
        s.close()


def test_R10_R11_R13_goal_succeeds_when_the_map_gets_known_and_explored_turns_true(make):
    holder = {}
    s = make(on_reach=lambda i: holder['s'].maps.publish(known_cells()))
    holder['s'] = s
    s.up()
    assert wait_for(lambda: s.explored[:1] == [False], 30.0), 'latched false at startup (R13)'
    handle = s.send()
    assert handle.accepted
    res = s.result(handle)
    assert res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success
    assert res.result.frontiers_done >= 1 and res.result.frontiers_blacklisted == 0
    assert wait_for(lambda: s.explored[-1:] == [True], 5.0)
    assert s.nav.goals and s.nav.goals[0][0] > 2.0, 'the frontier is at the unknown right edge'
    assert not s.notes


def test_R10_R11_all_frontiers_aborted_blacklists_them_fails_and_warns(make):
    s = make(script=lambda i, x, y: ABORT, cells=two_frontier_cells())
    s.up()
    res = s.result(s.send())
    assert res.status == GoalStatus.STATUS_ABORTED and not res.result.success
    assert len(s.nav.goals) == 4, 'one retry per frontier, then blacklist (R10)'
    assert res.result.frontiers_blacklisted == 2
    assert wait_for(lambda: s.notes, 5.0)
    note = s.notes[0]
    assert note.code == 'EXPLORATION_INCOMPLETE' and note.severity == Notification.WARN
    assert note.source == 'exploration_server'
    assert s.explored[-1] is False


def test_R11_explore_timeout_returns_partial_failure(make):
    s = make(script=lambda i, x, y: HANG, params=[('explore_timeout_s', 1.0)])
    s.up()
    res = s.result(s.send())
    assert res.status == GoalStatus.STATUS_ABORTED and not res.result.success
    assert res.result.duration_s >= 0.9
    assert s.nav.cancel_times, 'the goal in flight is cancelled when the deadline passes'
    assert wait_for(lambda: [n.code for n in s.notes] == ['EXPLORATION_INCOMPLETE'], 5.0)


def test_R11_goal_timeout_overrides_the_parameter(make):
    s = make(script=lambda i, x, y: HANG, params=[('explore_timeout_s', 600.0)])
    s.up()
    res = s.result(s.send(timeout_s=1.0), timeout=20.0)
    assert not res.result.success and res.result.duration_s < 5.0


def test_R12_cancel_cancels_the_nav2_goal_within_half_a_second_and_publishes_no_velocity(make):
    s = make(script=lambda i, x, y: HANG)
    s.up()
    handle = s.send()
    assert wait_for(lambda: s.nav.active, 10.0)
    t0 = time.monotonic()
    future = handle.cancel_goal_async()
    assert wait_for(lambda: s.nav.cancel_times, 2.0)
    assert s.nav.cancel_times[0] - t0 < 0.5
    assert wait_for(future.done, 5.0)
    res = s.result(handle)
    assert res.status == GoalStatus.STATUS_CANCELED
    assert wait_for(lambda: not s.nav.active, 5.0)
    assert not s.twists, 'the node never publishes velocity (Art. III.4)'


def test_R13_explored_has_one_publisher_false_at_start_and_late_joiner_gets_it(make):
    s = make()
    s.configure()
    assert wait_for(lambda: s.probe.count_publishers('/myRobot/map/explored') == 1, 30.0)
    assert wait_for(lambda: s.explored[:1] == [False], 30.0)
    late = []
    s.probe.create_subscription(Bool, '/myRobot/map/explored', lambda m: late.append(m.data), LATCHED)
    assert wait_for(lambda: late == [False], 30.0)


def test_R13_map_loaded_parameter_makes_explored_true(make):
    s = make(params=[('map_loaded', True)])
    s.configure()
    assert wait_for(lambda: s.explored == [True], 30.0)
    assert s.probe.count_publishers('/myRobot/map/explored') == 1


def test_R31_deactivate_cancels_the_nav2_goal_before_stopping(make):
    s = make(script=lambda i, x, y: HANG)
    s.up()
    handle = s.send()
    assert wait_for(lambda: s.nav.active, 10.0)
    assert s.node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
    assert wait_for(lambda: s.nav.cancel_times, 1.0), 'the Nav2 goal is cancelled on deactivate'
    res = s.result(handle)
    assert res.status == GoalStatus.STATUS_ABORTED
    assert wait_for(lambda: not s.nav.active, 5.0)


def test_R31_shutdown_cancels_the_nav2_goal(make):
    s = make(script=lambda i, x, y: HANG)
    s.up()
    s.send()
    assert wait_for(lambda: s.nav.active, 10.0)
    assert s.node.trigger_shutdown() == TransitionCallbackReturn.SUCCESS
    assert wait_for(lambda: s.nav.cancel_times, 1.0)


def test_R31_frontier_timeout_bounds_and_cancels_every_frontier_goal(make):
    s = make(script=lambda i, x, y: HANG, params=[('frontier_timeout_s', 0.5), ('explore_timeout_s', 3.0)])
    s.up()
    s.send()
    assert wait_for(lambda: len(s.nav.cancel_times) >= 2, 10.0), 'first attempt and its retry time out'
    assert len(s.nav.goals) >= 2


def test_R31_feedback_is_published_at_least_every_second_during_a_goal(make):
    s = make(script=lambda i, x, y: HANG)
    s.up()
    handle = s.send()
    assert wait_for(lambda: len(s.feedback) >= 4, 10.0)
    gaps = [b - a for a, b in zip(s.feedback, s.feedback[1:])]
    assert max(gaps) <= 1.0, gaps
    handle.cancel_goal_async()


def test_R32_cancel_and_resend_keeps_the_blacklist_and_counters(make):
    # goals 0 and 1 abort (first frontier blacklisted), goal 2 hangs; the cancel and re-send must not revisit it
    def script(i, x, y):
        return ABORT if i < 2 else (HANG if i == 2 else REACH)
    s = make(script=script, cells=two_frontier_cells())
    s.up()
    first = s.send()
    assert wait_for(lambda: len(s.nav.goals) >= 3 and s.nav.active, 10.0)
    first.cancel_goal_async()
    assert s.result(first).status == GoalStatus.STATUS_CANCELED
    second = s.send()
    assert second.accepted
    assert wait_for(lambda: len(s.nav.goals) >= 4, 10.0)
    blacklisted = s.nav.goals[0]
    assert all(abs(g[0] - blacklisted[0]) > 0.5 for g in s.nav.goals[2:]), s.nav.goals
    assert wait_for(lambda: s.values().get('resumed') == '1', 5.0)
    assert s.values()['blacklisted'] == '1.0' or s.values()['blacklisted'] == '1'
    second.cancel_goal_async()


def test_R26_telemetry_reports_state_and_counters_at_1hz(make):
    s = make()
    s.up()
    assert wait_for(lambda: len(s.telemetry) >= 2, 5.0)
    values = s.values()
    assert values['state'] == 'idle' and values['goals'] == '0' and values['explored'] == 'False'


def test_goal_is_rejected_when_inactive_and_without_a_map(make):
    s = make(with_map=False)
    s.configure()
    assert s.client.wait_for_server(timeout_sec=10.0)
    assert not s.send().accepted, 'inactive'
    assert s.node.trigger_activate() == TransitionCallbackReturn.SUCCESS
    assert not s.send().accepted, 'no map yet'


def test_goal_is_rejected_without_the_nav2_server(make):
    s = make(with_nav=False)
    s.up()
    assert wait_for(lambda: s.node._grid is not None, 5.0)
    assert not s.send().accepted
