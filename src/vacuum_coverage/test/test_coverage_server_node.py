"""coverage_server LifecycleNode wiring, in-process rclpy against a fake NavigateToPose server (no Nav2).
Requirements: 005-R16, R19 to R22, R26, R27, R31, R32, R34, R36. No fixed sleeps: every wait polls a condition.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.action.client import GoalStatus  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.lifecycle import TransitionCallbackReturn  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from vacuum_interfaces.action import CoverRoom  # noqa: E402

from coverage_fixtures import rect_room  # noqa: E402
from node_fixtures import ABORT, HANG, LATCHED, REACH, FakeNav, MapPublisher, wait_for  # noqa: E402
from vacuum_coverage.core.coverage_planner import CoveragePlanner  # noqa: E402
from vacuum_coverage.coverage_server_node import CoverageServerNode  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
RASTER = rect_room(2.0, 2.0)
MUX_INPUTS = ['teleop', 'stop', 'escape', 'nav', 'coverage', 'docking']


class System:
    def __init__(self, script=None, params=(), with_nav=True, with_map=True):
        rclpy.init()
        self.node = CoverageServerNode(parameter_overrides=[Parameter(k, value=v) for k, v in params])
        self.probe = Node('coverage_probe')
        self.nodes = [self.node, self.probe]
        self.nav = FakeNav(script or (lambda i, x, y: REACH)) if with_nav else None
        if self.nav:
            self.nodes.append(self.nav)
        if with_map:
            self.nodes.append(MapPublisher(RASTER))
        self.executor = MultiThreadedExecutor(num_threads=8)
        for n in self.nodes:
            self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.twists = []
        for name in MUX_INPUTS:
            self.probe.create_subscription(Twist, '/myRobot/mux/' + name, self.twists.append, QOS)
        self.telemetry = []
        self.probe.create_subscription(DiagnosticArray, '/myRobot/telemetry/coverage_server',
                                       self.telemetry.append, 10)
        self.paths, self.covered = [], []
        self.probe.create_subscription(Path, '/myRobot/coverage/path', self.paths.append, 10)
        self.probe.create_subscription(OccupancyGrid, '/myRobot/coverage/covered', self.covered.append, LATCHED)
        self.client = ActionClient(self.probe, CoverRoom, '/myRobot/cover_room')
        self.feedback = []

    def up(self):
        assert self.node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert self.node.trigger_activate() == TransitionCallbackReturn.SUCCESS
        assert self.client.wait_for_server(timeout_sec=10.0)

    def send(self, room_id='', passes=1):
        goal = CoverRoom.Goal(room_id=room_id, passes=passes)
        future = self.client.send_goal_async(goal, feedback_callback=lambda m: self.feedback.append(
            (time.monotonic(), m.feedback.covered_ratio)))
        assert wait_for(future.done, 10.0)
        handle = future.result()
        return handle

    def result(self, handle, timeout=60.0):
        future = handle.get_result_async()
        assert wait_for(future.done, timeout), 'no result'
        return future.result()

    def status_values(self):
        return {kv.key: kv.value for kv in self.telemetry[-1].status[0].values}

    def wait_ready(self):
        """Map, TF and the fake Nav2 server discovered: the node accepts goals from here on."""
        assert wait_for(lambda: self.node.has_map and self.node._nav.server_is_ready() and self.node._pose() is not None, 20.0)

    def close(self):
        try:
            for transition in (self.node.trigger_deactivate, self.node.trigger_cleanup):
                try:
                    transition()
                except Exception:           # already deactivated by the test (Jazzy raises, Lyrical does not)
                    pass
        finally:
            self.executor.shutdown()
            for n in self.nodes:
                n.destroy_node()
            rclpy.try_shutdown()
            self.thread.join(timeout=2)


@pytest.fixture
def system():
    made = []

    def make(**kwargs):
        s = System(**kwargs)
        made.append(s)
        return s

    yield make
    for s in made:
        s.close()


def waypoint_count():
    from vacuum_coverage.core.coverage_run import CoverageConfig
    c = CoverageConfig()
    planner = CoveragePlanner(c.lane_spacing_m, c.robot_radius_m, c.margin_m, c.min_segment_m)
    from vacuum_mapping.core.types import FREE
    return len(planner.plan(RASTER, RASTER.cells == FREE, 1))


def test_R21_configure_fails_when_lanes_leave_gaps():
    rclpy.init()
    try:
        node = CoverageServerNode(parameter_overrides=[Parameter('lane_spacing_m', value=0.5)])
        assert node.trigger_configure() == TransitionCallbackReturn.FAILURE
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_R26_documented_defaults_match_the_contract():
    rclpy.init()
    try:
        node = CoverageServerNode()
        expected = {'lane_spacing_m': 0.25, 'robot_radius_m': 0.16, 'margin_m': 0.05, 'min_segment_m': 0.30,
                    'tool_radius_m': 0.15, 'target_ratio': 0.95, 'min_acceptable_ratio': 0.70,
                    'max_skip_ratio': 0.3, 'coverage_timeout_s': 1800.0, 'waypoint_timeout_s': 120.0,
                    'resume_window_s': 120.0, 'max_region_area_m2': 150.0, 'telemetry_rate_hz': 1.0,
                    'map_topic': '/map', 'nav_action': '/navigate_to_pose'}
        for name, value in expected.items():
            assert node.get_parameter(name).value == value, name
        assert not node.has_parameter('linear_x')          # stub only
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_R36_goal_rejected_while_navigation_is_not_available(system):
    # map and TF exist; the node is pointed at a NavigateToPose action nobody serves
    s = system(params=[('nav_action', '/no_such_navigate_to_pose')])
    s.up()
    assert wait_for(lambda: s.node.has_map and s.node._pose() is not None, 20.0)
    assert not s.node._nav.server_is_ready()
    assert not s.send().accepted


def test_R16_goal_rejected_without_a_map(system):
    s = system(with_map=False)
    s.up()
    assert not s.send().accepted


def test_R16_unknown_room_id_rejected_without_room_topics(system):
    s = system()
    s.up()
    s.wait_ready()
    assert not s.send(room_id='room_99').accepted


def test_R19_R21_success_publishes_no_velocity_and_reports_ratio(system):
    s = system()
    s.up()
    s.wait_ready()
    handle = s.send()
    assert handle.accepted
    res = s.result(handle)
    assert res.status == GoalStatus.STATUS_SUCCEEDED
    assert res.result.success and res.result.covered_ratio >= 0.95
    assert s.nav.goals, 'no NavigateToPose goal was sent'
    assert s.feedback
    assert s.twists == [], 'coverage_server must publish no velocity (005-R19, D-24)'


def test_R21_degraded_success_between_min_and_target(system):
    keep = int(0.8 * waypoint_count())
    s = system(script=lambda i, x, y: REACH if i < keep else ABORT, params=[('max_skip_ratio', 1.0)])
    s.up()
    s.wait_ready()
    res = s.result(s.send())
    assert res.result.success is True
    assert 0.70 <= res.result.covered_ratio < 0.95


def test_R20_R21_failure_below_min_ratio(system):
    s = system(script=lambda i, x, y: ABORT, params=[('max_skip_ratio', 1.0)])
    s.up()
    s.wait_ready()
    res = s.result(s.send())
    assert res.status == GoalStatus.STATUS_ABORTED
    assert res.result.success is False


def test_R20_hanging_waypoint_is_cancelled_retried_once_then_skipped(system):
    first = {}

    def script(i, x, y):
        first.setdefault('pose', (x, y))
        return HANG if (x, y) == first['pose'] and i < 2 else REACH

    s = system(script=script, params=[('waypoint_timeout_s', 0.5)])
    s.up()
    s.wait_ready()
    res = s.result(s.send())
    assert s.nav.goals[0] == s.nav.goals[1] != s.nav.goals[2], 'retry once, then skip'
    assert len(s.nav.cancel_times) >= 2
    assert res.result.success
    assert wait_for(lambda: s.telemetry and s.status_values().get('skipped_waypoints') == '1', 5.0)


def test_R22_cancel_cancels_the_nav_goal_within_half_a_second(system):
    s = system(script=lambda i, x, y: HANG)
    s.up()
    s.wait_ready()
    handle = s.send()
    assert wait_for(lambda: s.nav.goals, 10.0)
    t0 = time.monotonic()
    cancel = handle.cancel_goal_async()
    assert wait_for(lambda: s.nav.cancel_times, 2.0)
    assert s.nav.cancel_times[0] - t0 < 0.5
    assert wait_for(cancel.done, 5.0)
    assert s.result(handle).status == GoalStatus.STATUS_CANCELED


def test_R32_same_room_after_cancel_resumes_at_the_interrupted_waypoint(system):
    stage = {'phase': 1}

    def script(i, x, y):
        return REACH if (stage['phase'] == 1 and i < 3) or stage['phase'] == 2 else HANG

    s = system(script=script)
    s.up()
    s.wait_ready()
    handle = s.send()
    assert wait_for(lambda: len(s.nav.goals) == 4, 10.0)       # three reached, the fourth hangs
    interrupted = s.nav.goals[3]
    cancel = handle.cancel_goal_async()
    assert wait_for(cancel.done, 5.0)
    assert s.result(handle).status == GoalStatus.STATUS_CANCELED
    assert wait_for(lambda: not s.nav.active, 5.0)
    stage['phase'] = 2
    count = len(s.nav.goals)
    res = s.result(s.send())
    assert s.nav.goals[count] == interrupted, 'resume from the first waypoint not yet reached'
    assert res.result.success


def test_R31_deactivate_cancels_the_nav_goal_in_flight(system):
    s = system(script=lambda i, x, y: HANG)
    s.up()
    s.wait_ready()
    handle = s.send()
    assert wait_for(lambda: s.nav.goals, 10.0)
    s.node.trigger_deactivate()
    assert wait_for(lambda: s.nav.cancel_times, 2.0)
    assert s.result(handle).status == GoalStatus.STATUS_ABORTED


def test_R34_feedback_at_one_hertz_or_faster_while_a_goal_is_active(system):
    s = system(script=lambda i, x, y: HANG)
    s.up()
    s.wait_ready()
    s.send()
    assert wait_for(lambda: len(s.feedback) >= 4, 10.0)
    gaps = [b[0] - a[0] for a, b in zip(s.feedback, s.feedback[1:])]
    assert max(gaps) <= 1.0, gaps


def test_R27_telemetry_path_and_covered_grid_are_published(system):
    s = system()
    s.up()
    s.wait_ready()
    assert wait_for(lambda: s.telemetry, 5.0)
    assert s.status_values()['state'] == 'idle'
    s.result(s.send())
    assert wait_for(lambda: s.paths, 5.0)
    assert s.paths[0].header.frame_id == 'map' and s.paths[0].poses
    assert wait_for(lambda: s.covered and max(s.covered[-1].data) == 100, 5.0)
    assert min(s.covered[-1].data) == -1
    values = s.status_values()
    assert {'goals', 'succeeded', 'failed', 'cancelled', 'skipped_waypoints', 'covered_ratio'} <= set(values)


def test_R16_two_simultaneous_goals_accept_exactly_one(system):
    s = system(script=lambda i, x, y: HANG)
    s.up()
    s.wait_ready()
    futures = [s.client.send_goal_async(CoverRoom.Goal(room_id='', passes=1)) for _ in range(4)]
    assert wait_for(lambda: all(f.done() for f in futures), 10.0)
    assert sum(1 for f in futures if f.result().accepted) == 1
