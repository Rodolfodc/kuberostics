"""Stub LifecycleNodes wiring, in-process rclpy (no launch). Requirements: 001-R06, 001-R07, 001-R16."""
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from vacuum_interfaces.action import CoverRoom, Escape  # noqa: E402
from vacuum_interfaces.msg import StuckEvent  # noqa: E402

from vacuum_stubs.coverage_server_node import CoverageServerNode  # noqa: E402
from vacuum_stubs.escape_server_node import EscapeServerNode  # noqa: E402
from vacuum_stubs.stuck_detector_node import StuckDetectorNode  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def wait_for(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def wait_quiet(count, quiet=1.0, timeout=10.0):
    """Waits until `count()` stops changing for `quiet` seconds (messages already in flight are delivered)."""
    deadline = time.monotonic() + timeout
    last, since = count(), time.monotonic()
    while time.monotonic() < deadline:
        now = count()
        if now != last:
            last, since = now, time.monotonic()
        elif time.monotonic() - since >= quiet:
            return True
        time.sleep(0.02)
    return False


class Harness:
    def __init__(self, node):
        rclpy.init()                        # the domain is isolated by conftest.py
        self.node = node()
        self.probe = Node('stub_probe')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor.add_node(self.probe)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def up(self):
        self.node.trigger_configure()
        self.node.trigger_activate()

    def close(self):
        self.node.trigger_deactivate()
        self.node.trigger_cleanup()
        self.executor.shutdown()
        self.node.destroy_node()
        self.probe.destroy_node()
        rclpy.try_shutdown()
        self.thread.join(timeout=2)


def make_system(node_cls, params=()):
    def factory():
        return node_cls(parameter_overrides=[Parameter(k, value=v) for k, v in params])
    h = Harness(factory)
    h.up()
    return h


def collect_twists(h, topic):
    got = []
    h.probe.create_subscription(Twist, topic, lambda m: got.append((time.monotonic(), m)), QOS)
    return got


def send(h, client, goal):
    assert client.wait_for_server(timeout_sec=10.0)
    fut = client.send_goal_async(goal)
    assert wait_for(fut.done)
    handle = fut.result()
    assert handle.accepted
    return handle


def result_of(handle, timeout=15.0):
    fut = handle.get_result_async()
    assert wait_for(fut.done, timeout), 'no result'
    return fut.result().result


@pytest.fixture()
def coverage():
    h = make_system(CoverageServerNode, [('linear_x', 0.2), ('angular_z', 0.1)])
    yield h
    h.close()


def test_001_R06_coverage_publishes_constant_twist_while_goal_active_and_stops_on_cancel(coverage):
    got = collect_twists(coverage, '/myRobot/mux/coverage')
    time.sleep(0.5)
    assert got == [], 'must publish nothing when idle'
    client = ActionClient(coverage.probe, CoverRoom, '/myRobot/cover_room')
    handle = send(coverage, client, CoverRoom.Goal(room_id='', passes=1))
    # 21 samples (20 intervals, ~2 s): a delivery burst after a CPU stall cannot dominate the mean rate
    assert wait_for(lambda: len(got) >= 21, 15.0)
    assert all(abs(m.linear.x - 0.2) < 1e-6 and abs(m.angular.z - 0.1) < 1e-6 for _, m in got)
    window = got[:21]
    assert 8.0 <= 20 / (window[-1][0] - window[0][0]) <= 12.0
    cancel = handle.cancel_goal_async()
    assert wait_for(cancel.done)
    res = handle.get_result_async()
    assert wait_for(res.done)
    assert res.result().result.success is False and res.result().result.covered_ratio == 0.0
    assert wait_quiet(lambda: len(got)), 'kept publishing after cancel'
    n = len(got)
    time.sleep(0.5)
    assert len(got) == n, 'must publish nothing after cancel'


def test_001_R06_coverage_feedback_ratio_increases(coverage):
    fb = []
    client = ActionClient(coverage.probe, CoverRoom, '/myRobot/cover_room')
    assert client.wait_for_server(timeout_sec=10.0)
    fut = client.send_goal_async(CoverRoom.Goal(), feedback_callback=lambda m: fb.append(m.feedback.covered_ratio))
    assert wait_for(fut.done)
    assert wait_for(lambda: len(fb) >= 2, 10.0)
    assert fb[-1] >= fb[0] >= 0.0


def test_001_R06_D20_coverage_telemetry_published(coverage):
    got = []
    coverage.probe.create_subscription(DiagnosticArray, '/myRobot/telemetry/coverage_server', got.append, 10)
    assert wait_for(lambda: got, 10.0)


def run_escape(outcome, **extra):
    params = [('outcome', outcome), ('reverse_s', 0.3), ('turn_s', 0.3)]
    params += list(extra.items())
    return make_system(EscapeServerNode, params)


@pytest.mark.parametrize('outcome,expected', [('succeed', True), ('fail', False)])
def test_001_R16_escape_outcome_parameter_decides_result(outcome, expected):
    h = run_escape(outcome)
    try:
        got = collect_twists(h, '/myRobot/mux/escape')
        client = ActionClient(h.probe, Escape, '/myRobot/escape')
        handle = send(h, client, Escape.Goal(event=StuckEvent(), timeout_s=20.0))
        res = result_of(handle)
        assert res.success is expected
        assert res.strategy == 'scripted'
        assert 0.5 <= res.duration_s <= 2.0
        signs = [m for _, m in got]
        assert any(m.linear.x < 0 for m in signs), 'reverse phase'
        assert any(m.angular.z > 0 for m in signs), 'turn phase'
    finally:
        h.close()


def test_001_R07_escape_hang_keeps_publishing_until_cancelled():
    h = run_escape('hang')
    try:
        got = collect_twists(h, '/myRobot/mux/escape')
        client = ActionClient(h.probe, Escape, '/myRobot/escape')
        handle = send(h, client, Escape.Goal(event=StuckEvent(), timeout_s=20.0))
        time.sleep(1.5)                      # well beyond the 0.6 s script
        res = handle.get_result_async()
        assert not res.done()
        n = len(got)
        assert wait_for(lambda: len(got) > n), 'hang keeps publishing'
        cancel = handle.cancel_goal_async()
        assert wait_for(cancel.done)
        assert wait_for(res.done)
        assert res.result().result.success is False
        assert wait_quiet(lambda: len(got))
        n = len(got)
        time.sleep(0.5)
        assert len(got) == n
    finally:
        h.close()


def test_001_R07_stuck_trigger_publishes_one_event_per_call():
    h = make_system(StuckDetectorNode)
    try:
        events = []
        h.probe.create_subscription(StuckEvent, '/myRobot/stuck', events.append, 10)
        cli = h.probe.create_client(Trigger, '/myRobot/stuck_detector/trigger')
        assert cli.wait_for_service(timeout_sec=10.0)
        for i in (1, 2):
            fut = cli.call_async(Trigger.Request())
            assert wait_for(fut.done)
            assert fut.result().success
            assert wait_for(lambda: len(events) == i)
        time.sleep(0.3)
        assert len(events) == 2
        e = events[0]
        assert e.cause == StuckEvent.NO_PROGRESS
        assert len(e.scan_ranges) == 0
        assert e.pose.position.x == 0.0 and e.pose.orientation.w == 1.0
        assert e.stamp.sec > 0 or e.stamp.nanosec > 0
    finally:
        h.close()


def test_001_R06_coverage_rejects_second_goal_while_active(coverage):
    client = ActionClient(coverage.probe, CoverRoom, '/myRobot/cover_room')
    first = send(coverage, client, CoverRoom.Goal())
    fut = client.send_goal_async(CoverRoom.Goal())
    assert wait_for(fut.done)
    assert not fut.result().accepted
    wait_for(first.cancel_goal_async().done)
    assert wait_for(first.get_result_async().done)
    again = send(coverage, client, CoverRoom.Goal())      # accepted again once the first has finished
    again.cancel_goal_async()


def test_001_R06_coverage_goal_does_not_resume_after_deactivate_activate(coverage):
    got = collect_twists(coverage, '/myRobot/mux/coverage')
    client = ActionClient(coverage.probe, CoverRoom, '/myRobot/cover_room')
    send(coverage, client, CoverRoom.Goal())
    assert wait_for(lambda: len(got) >= 3)
    coverage.node.trigger_deactivate()
    coverage.node.trigger_activate()
    assert wait_quiet(lambda: len(got))
    n = len(got)
    time.sleep(0.5)
    assert len(got) == n, 'old goal must not resume publishing'


def test_001_R07_escape_rejects_second_goal_while_active():
    h = run_escape('hang')
    try:
        client = ActionClient(h.probe, Escape, '/myRobot/escape')
        first = send(h, client, Escape.Goal(event=StuckEvent(), timeout_s=20.0))
        fut = client.send_goal_async(Escape.Goal(event=StuckEvent(), timeout_s=20.0))
        assert wait_for(fut.done)
        assert not fut.result().accepted
        wait_for(first.cancel_goal_async().done)
    finally:
        h.close()


def test_001_R07_escape_goal_does_not_resume_after_deactivate_activate():
    h = run_escape('hang')
    try:
        got = collect_twists(h, '/myRobot/mux/escape')
        client = ActionClient(h.probe, Escape, '/myRobot/escape')
        send(h, client, Escape.Goal(event=StuckEvent(), timeout_s=20.0))
        assert wait_for(lambda: len(got) >= 3)
        h.node.trigger_deactivate()
        h.node.trigger_activate()
        assert wait_quiet(lambda: len(got))
        n = len(got)
        time.sleep(0.5)
        assert len(got) == n
    finally:
        h.close()


def test_001_R07_D20_escape_telemetry_published():
    h = run_escape('succeed')
    try:
        got = []
        h.probe.create_subscription(DiagnosticArray, '/myRobot/telemetry/escape_server', got.append, 10)
        assert wait_for(lambda: got, 10.0)
        assert any(kv.key == 'outcome' and kv.value == 'succeed' for kv in got[0].status[0].values)
    finally:
        h.close()


def test_001_R07_D20_stuck_detector_telemetry_counts_events():
    h = make_system(StuckDetectorNode)
    try:
        got = []
        h.probe.create_subscription(DiagnosticArray, '/myRobot/telemetry/stuck_detector', got.append, 10)
        cli = h.probe.create_client(Trigger, '/myRobot/stuck_detector/trigger')
        assert cli.wait_for_service(timeout_sec=10.0)
        fut = cli.call_async(Trigger.Request())
        assert wait_for(fut.done)
        assert wait_for(lambda: got and any(kv.key == 'events' and kv.value == '1'
                                            for kv in got[-1].status[0].values), 10.0)
    finally:
        h.close()
