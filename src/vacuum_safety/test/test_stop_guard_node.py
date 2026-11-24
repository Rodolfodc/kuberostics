"""T018: stop_guard LifecycleNode wiring (001-R19). In-process rclpy, no launch file. Skipped without rclpy."""
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from vacuum_safety.stop_guard_node import StopGuardNode  # noqa: E402

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


class Probe(Node):
    def __init__(self):
        super().__init__('stop_guard_probe')
        self.stamps = []
        self.telemetry = None
        self.pub_lock = self.create_publisher(Bool, '/myRobot/mux/lock_stop', QOS)
        self.create_subscription(Twist, '/myRobot/mux/stop', self._on_stop, QOS)
        self.create_subscription(DiagnosticArray, '/myRobot/telemetry/stop_guard',
                                 lambda m: setattr(self, 'telemetry', m), 10)

    def _on_stop(self, msg):
        assert msg.linear.x == 0.0 and msg.angular.z == 0.0
        self.stamps.append(time.monotonic())

    def lock(self, value):
        # the topic is volatile: a sample published before discovery completes is lost, so wait for the guard
        assert wait_for(lambda: self.pub_lock.get_subscription_count() >= 1), 'guard never subscribed to the lock'
        self.pub_lock.publish(Bool(data=value))


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


@pytest.fixture()
def system():
    rclpy.init()                            # the domain is isolated by conftest.py
    guard = StopGuardNode()
    probe = Probe()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(guard)
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    guard.trigger_configure()
    guard.trigger_activate()
    yield guard, probe
    guard.trigger_deactivate()
    guard.trigger_cleanup()
    executor.shutdown()
    guard.destroy_node()
    probe.destroy_node()
    rclpy.try_shutdown()
    thread.join(timeout=2)


def test_001_R19_publishes_nothing_while_unlocked(system):
    _, p = system
    p.lock(False)
    time.sleep(0.6)
    assert p.stamps == []


def test_001_R19_publishes_zero_velocity_at_10_hz_while_locked(system):
    _, p = system
    p.lock(True)
    # 21 samples (20 intervals, ~2 s): a delivery burst after a CPU stall cannot dominate the mean rate
    assert wait_for(lambda: len(p.stamps) >= 21, timeout=15.0)
    window = p.stamps[:21]
    rate = 20 / (window[-1] - window[0])
    assert 8.0 <= rate <= 12.0, rate        # well above the 2 Hz minimum implied by the 0.5 s input timeout


def test_001_R19_stops_publishing_after_the_lock_is_released(system):
    _, p = system
    p.lock(True)
    assert wait_for(lambda: len(p.stamps) >= 3)
    p.lock(False)
    assert wait_quiet(lambda: len(p.stamps)), 'zeros kept coming after the lock was released'
    count = len(p.stamps)
    time.sleep(0.5)
    assert len(p.stamps) == count


def test_D20_telemetry_reports_the_lock(system):
    _, p = system
    p.lock(True)
    assert wait_for(lambda: p.telemetry is not None and any(
        kv.key == 'locked' and kv.value == 'True' for kv in p.telemetry.status[0].values), timeout=4.0)


def test_001_R19_zeros_resume_after_a_deactivate_activate_cycle_while_locked(system):
    g, p = system
    p.lock(True)
    assert wait_for(lambda: len(p.stamps) >= 3)
    g.trigger_deactivate()
    assert wait_quiet(lambda: len(p.stamps))
    count = len(p.stamps)
    g.trigger_activate()
    assert wait_for(lambda: len(p.stamps) > count + 2, timeout=10.0), 'zeros did not resume after reactivation'
