"""Real stuck_detector and escape_server LifecycleNodes, in-process rclpy against a fake world (no simulator).
Requirements: 005-R01 to R06 (detector), R08 to R14 (escape), R12 (stop), R26, R27. No fixed sleeps: every wait polls
a condition; "silence" is asserted as an unchanged message count over a settle window.

Domain IDs: range 1-5 of conftest.py (this package), one context and domain per test, rotating.
"""
import itertools
import math
import os
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

import yaml  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.action.client import GoalStatus  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.lifecycle import TransitionCallbackReturn  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from vacuum_contract_tests.contract import find_contract  # noqa: E402
from vacuum_interfaces.action import Escape  # noqa: E402
from vacuum_interfaces.msg import StuckEvent  # noqa: E402

from vacuum_coverage.escape_server_node import EscapeServerNode  # noqa: E402
from vacuum_coverage.stuck_detector_node import StuckDetectorNode  # noqa: E402
from world_fixtures import QOS, FakeWorld, forward_command_publisher, wait_for  # noqa: E402

_domains = itertools.cycle(range(1, 6))
IGNORED = {'use_sim_time', 'start_type_description_service'}     # rclpy's own parameters
FAST_STUCK = [('window_s', 1.0), ('cooldown_s', 1.5)]
# a quick fallback: speeds raised with their limits so that validate() holds (sweep 2.1 s + 1.05 s + 1.0 s < 5 s)
FAST_FALLBACK = [('primary_budget_s', 1.0), ('fallback_budget_s', 5.0), ('default_timeout_s', 8.0),
                 ('turn_speed_rad_s', 3.0), ('max_angular_speed_rad_s', 3.0),
                 ('reverse_speed_m_s', 0.3), ('max_linear_speed_m_s', 0.3)]


class Rig:
    def __init__(self, detector=None, escape=None, world=None, up=True):
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=next(_domains))
        self.nodes = []
        self.probe = Node('probe', context=self.context)
        self.nodes.append(self.probe)
        self.world = FakeWorld(self.context, **(world or {})) if world is not None else None
        if self.world:
            self.nodes.append(self.world)
        self.detector = self.escape = None
        if detector is not None:
            self.detector = StuckDetectorNode(context=self.context, parameter_overrides=[
                Parameter(k, value=v) for k, v in detector])
            self.nodes.append(self.detector)
        if escape is not None:
            self.escape = EscapeServerNode(context=self.context, parameter_overrides=[
                Parameter(k, value=v) for k, v in escape])
            self.nodes.append(self.escape)
        self.events, self.telemetry = [], {}
        self.probe.create_subscription(StuckEvent, '/myRobot/stuck', self.events.append, 10)
        for name in ('stuck_detector', 'escape_server'):
            self.probe.create_subscription(
                DiagnosticArray, f'/myRobot/telemetry/{name}',
                lambda m, n=name: self.telemetry.__setitem__(n, {kv.key: kv.value for kv in m.status[0].values}), 10)
        self.client = ActionClient(self.probe, Escape, '/myRobot/escape')
        self.executor = MultiThreadedExecutor(num_threads=8, context=self.context)
        for n in self.nodes:
            self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        if up:
            self.up()

    def up(self):
        for n in (self.detector, self.escape):
            if n is not None:
                assert n.trigger_configure() == TransitionCallbackReturn.SUCCESS
                assert n.trigger_activate() == TransitionCallbackReturn.SUCCESS
        if self.escape is not None:
            assert self.client.wait_for_server(timeout_sec=30.0)
            if self.world is not None and self.world.with_odom:       # the first odometry and laser have arrived
                assert wait_for(lambda: self.escape._odom is not None, 30.0)
            if self.world is not None and self.world.with_scan:
                assert wait_for(lambda: self.escape._scan.geometry() is not None, 30.0)

    def send(self, timeout_s=0.0, ranges=None, feedback=None):
        goal = Escape.Goal(timeout_s=float(timeout_s))
        goal.event.cause = 'NO_PROGRESS'
        goal.event.scan_ranges = ranges if ranges is not None else [
            2.0 if -math.pi + i * math.radians(1.0) > 0 else 1.0 for i in range(360)]
        future = self.client.send_goal_async(goal, feedback_callback=feedback)
        assert wait_for(future.done, 30.0)
        return future.result()

    def result(self, handle, timeout=40.0):
        future = handle.get_result_async()
        assert wait_for(future.done, timeout), 'no result'
        return future.result()

    def silent_for(self, name='escape', settle_s=0.6):
        """True when no message arrives on mux/<name> over `settle_s` (count unchanged)."""
        n = self.world.count(name)
        time.sleep(settle_s)
        return self.world.count(name) == n

    def close(self):
        try:
            for n in (self.detector, self.escape):
                if n is not None:
                    for transition in (n.trigger_deactivate, n.trigger_cleanup):
                        try:
                            transition()
                        except Exception:       # already deactivated by the test (Jazzy raises, Lyrical does not)
                            pass
        finally:
            self.executor.shutdown()
            for n in self.nodes:
                n.destroy_node()
            self.context.try_shutdown()
            self.thread.join(timeout=3)


@pytest.fixture
def rig():
    made = []

    def make(**kw):
        r = Rig(**kw)
        made.append(r)
        return r
    yield make
    for r in made:
        r.close()


# --------------------------------------------------------------------------- stuck_detector

def test_005_R01_R04_wedged_robot_yields_a_no_progress_event_with_pose_and_scan(rig):
    r = rig(detector=FAST_STUCK, world={})
    forward_command_publisher(r.probe, 0.2)
    assert wait_for(lambda: r.events, 30.0), 'no StuckEvent'
    e = r.events[0]
    assert e.cause == 'NO_PROGRESS'
    assert len(e.scan_ranges) == 360                     # copied from the fresh /scan (005-R04)
    assert e.pose.orientation.w == pytest.approx(1.0)    # no TF in this graph: the odometry pose (005-R04)
    assert e.stamp.sec > 0 or e.stamp.nanosec > 0


def test_005_R02_zero_command_never_produces_an_event(rig):
    r = rig(detector=FAST_STUCK, world={})
    state, _ = forward_command_publisher(r.probe, 0.0)   # the zero Twist of the stop lock (D-22)
    assert wait_for(lambda: r.detector._core.snapshot()['state'] == 'idle', 10.0)
    time.sleep(3.0)                                      # > window_s + cooldown: a negative check must wait it out
    assert r.events == []


def test_005_R01_a_command_that_moves_the_robot_is_not_stuck(rig):
    r = rig(detector=FAST_STUCK, world={})
    # the escape input drives the fake robot; the detector sees the same motion on /myRobot/cmd_vel
    cmd_pub = r.probe.create_publisher(Twist, '/myRobot/cmd_vel', QOS)
    esc_pub = r.probe.create_publisher(Twist, '/myRobot/mux/escape', QOS)
    msg = Twist()
    msg.angular.z = 1.0

    def tick():
        cmd_pub.publish(msg)
        esc_pub.publish(msg)
    r.probe.create_timer(0.05, tick)
    assert wait_for(lambda: r.telemetry.get('stuck_detector', {}).get('state') == 'monitoring', 20.0)
    time.sleep(3.0)
    assert r.events == []


def test_005_R05_bumper_rising_edge_publishes_a_bumper_event(rig):
    r = rig(detector=[('bumper_topic', '/bumper'), *FAST_STUCK], world={})
    pub = r.probe.create_publisher(Bool, '/bumper', 10)
    assert wait_for(lambda: pub.get_subscription_count() > 0, 20.0)
    pub.publish(Bool(data=True))
    assert wait_for(lambda: r.events, 20.0)
    assert r.events[0].cause == 'BUMPER'


def test_005_R06_trigger_service_publishes_one_event_per_call(rig):
    r = rig(detector=[], world={})
    client = r.probe.create_client(Trigger, '/myRobot/stuck_detector/trigger')
    assert client.wait_for_service(timeout_sec=30.0)
    for expected in (1, 2):
        future = client.call_async(Trigger.Request())
        assert wait_for(future.done, 10.0)
        assert future.result().success and future.result().message == 'NO_PROGRESS'
        assert wait_for(lambda: len(r.events) == expected, 10.0)


def test_005_R27_detector_publishes_telemetry_and_rejects_bad_parameters(rig):
    r = rig(detector=[], world={})
    assert wait_for(lambda: 'stuck_detector' in r.telemetry, 20.0)
    assert 'events' in r.telemetry['stuck_detector']
    bad = StuckDetectorNode(context=r.context, parameter_overrides=[Parameter('window_s', value=-1.0)])
    assert bad.trigger_configure() == TransitionCallbackReturn.FAILURE
    bad.destroy_node()


def test_005_R05_no_bumper_subscription_by_default(rig):
    r = rig(detector=[], world={})
    assert all(s.topic_name != '/bumper' for s in r.detector._subs)
    assert len(r.detector._subs) == 3


# --------------------------------------------------------------------------- escape_server

def test_005_R08_R10_R13_primary_escape_succeeds_and_drives_only_mux_escape(rig):
    r = rig(escape=[], world={})
    feedback = []
    h = r.send(feedback=lambda m: feedback.append(m.feedback.elapsed_s))
    assert h.accepted
    res = r.result(h)
    assert h.status == GoalStatus.STATUS_SUCCEEDED
    assert res.result.success and res.result.strategy == 'scripted'
    assert res.result.duration_s > 1.0
    assert len(feedback) >= 4                                  # >= 2 Hz over a few seconds (005-R13)
    assert r.world.count('escape') > 10
    assert r.world.total_moving_other_than_escape() == 0       # nothing on any other mux input (Art. VI)
    assert r.world.cmd_vel_msgs == []                          # and nobody touches /myRobot/cmd_vel
    assert any(m.linear.x < 0 for _, m in r.world.mux_msgs['escape'])    # it backed up
    assert any(m.angular.z != 0 for _, m in r.world.mux_msgs['escape'])  # and turned
    assert r.world.yaw > 1.0                                   # turned left, toward the larger side range
    # the stop: one zero Twist, then silence (005-R12, D-22)
    assert r.world.last('escape').linear.x == 0.0 and r.world.last('escape').angular.z == 0.0
    assert r.silent_for('escape')


def test_005_R10_R11_blocked_primary_runs_the_fallback_and_succeeds(rig):
    r = rig(escape=FAST_FALLBACK, world={'door_yaw': math.pi - 0.1})
    h = r.send()
    res = r.result(h)
    assert res.result.success, 'fallback did not find the opening'
    assert res.result.duration_s > 1.0                         # past the primary budget
    assert any(m.angular.z != 0 for _, m in r.world.mux_msgs['escape'])
    assert r.world.total_moving_other_than_escape() == 0


def test_005_R12_timeout_stops_the_robot_and_returns_failure(rig):
    r = rig(escape=[], world={'always_blocked': True})
    h = r.send(timeout_s=2.0)
    res = r.result(h)
    assert h.status == GoalStatus.STATUS_SUCCEEDED and not res.result.success
    assert 1.8 <= res.result.duration_s <= 3.5
    assert r.world.last('escape').linear.x == 0.0 and r.world.last('escape').angular.z == 0.0
    assert r.silent_for('escape')


def test_005_R12_no_odometry_fails_without_moving(rig):
    r = rig(escape=[], world={'with_odom': False})
    h = r.send()
    res = r.result(h)
    assert not res.result.success
    assert res.result.duration_s >= 0.9                # it waited for the first odometry (ODOM_GRACE_S)
    assert r.world.count('escape') == 0


def test_005_R12_cancel_mid_escape_stops_the_motion(rig):
    r = rig(escape=[], world={'always_blocked': True})
    h = r.send(timeout_s=15.0)
    assert wait_for(lambda: r.world.count('escape') > 5, 20.0)
    cancel = h.cancel_goal_async()
    assert wait_for(cancel.done, 10.0)
    res = r.result(h)
    assert h.status == GoalStatus.STATUS_CANCELED
    assert not res.result.success
    assert r.world.last('escape').linear.x == 0.0 and r.world.last('escape').angular.z == 0.0
    assert r.silent_for('escape')


def test_005_R12_deactivation_mid_escape_aborts_and_stops(rig):
    r = rig(escape=[], world={'always_blocked': True})
    h = r.send(timeout_s=15.0)
    assert wait_for(lambda: r.world.count('escape') > 5, 20.0)
    r.escape.trigger_deactivate()
    res = r.result(h)
    assert h.status == GoalStatus.STATUS_ABORTED and not res.result.success
    # the brake went out before the lifecycle publisher was disabled (it was dropped by the first version)
    assert r.world.last('escape').linear.x == 0.0 and r.world.last('escape').angular.z == 0.0
    assert r.silent_for('escape')


def test_005_R14_hang_outcome_holds_still_until_cancelled(rig):
    r = rig(escape=[('outcome', 'hang'), ('reverse_s', 0.2), ('turn_s', 0.2)], world={})
    h = r.send(timeout_s=15.0)
    assert wait_for(lambda: r.world.count('escape') > 15, 20.0)
    assert r.world.last('escape').linear.x == 0.0 and r.world.last('escape').angular.z == 0.0
    h.cancel_goal_async()
    res = r.result(h)
    assert h.status == GoalStatus.STATUS_CANCELED
    assert not res.result.success


@pytest.mark.parametrize('outcome,expected', [('succeed', True), ('fail', False)])
def test_005_R14_succeed_and_fail_outcomes_keep_the_stub_behaviour(rig, outcome, expected):
    r = rig(escape=[('outcome', outcome), ('reverse_s', 0.2), ('turn_s', 0.2)], world={})
    h = r.send()
    res = r.result(h)
    assert h.status == GoalStatus.STATUS_SUCCEEDED and res.result.success is expected
    assert res.result.strategy == 'scripted'


def test_005_R12_a_second_goal_is_rejected_while_one_runs_and_when_inactive(rig):
    r = rig(escape=[], world={'always_blocked': True})
    first = r.send(timeout_s=10.0)
    assert first.accepted
    second = r.send()
    assert not second.accepted
    first.cancel_goal_async()
    r.result(first)
    r.escape.trigger_deactivate()
    assert not r.send().accepted


def test_005_R11_configure_fails_when_the_speeds_exceed_the_limits(rig):
    r = rig(escape=[('reverse_speed_m_s', 2.0)], world={}, up=False)
    assert r.escape.trigger_configure() == TransitionCallbackReturn.FAILURE


def test_005_R14_configure_fails_on_an_unknown_outcome(rig):
    r = rig(escape=[('outcome', 'explode')], world={}, up=False)
    assert r.escape.trigger_configure() == TransitionCallbackReturn.FAILURE


def test_005_R27_escape_publishes_telemetry_and_counts_results(rig):
    r = rig(escape=[('outcome', 'succeed'), ('reverse_s', 0.1), ('turn_s', 0.1)], world={})
    r.result(r.send())
    assert wait_for(lambda: r.telemetry.get('escape_server', {}).get('succeeded') == '1', 20.0)


def test_005_R26_declared_parameters_cover_the_contract_block(rig):
    """Art. I: every public parameter of the contract is declared by the real node (and so overridable)."""
    with open(find_contract(), encoding='utf-8') as handle:
        params = yaml.safe_load(handle)['parameters']
    r = rig(detector=[], escape=[], world={}, up=False)
    for node, block in (('detector', 'stuck_detector'), ('escape', 'escape_server')):
        declared = set(getattr(r, node)._parameters) - IGNORED
        missing = sorted(set(params[block]) - declared)
        assert not missing, f'{block} does not declare {missing}'
        undeclared = sorted(n for n in declared - set(params[block]) if n not in IGNORED)
        assert not undeclared, f'{block} declares parameters missing from the contract: {undeclared}'
