"""T018: stop_guard LifecycleNode wiring (001-R19). In-process rclpy, no launch file. Skipped without rclpy."""
import os
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
        self.pub_lock.publish(Bool(data=value))


def wait_for(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture()
def system():
    os.environ.setdefault('ROS_DOMAIN_ID', '88')
    rclpy.init()
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
    assert wait_for(lambda: len(p.stamps) >= 11, timeout=3.0)
    window = p.stamps[-11:]
    rate = 10 / (window[-1] - window[0])
    assert 8.0 <= rate <= 12.0, rate        # well above the 2 Hz minimum implied by the 0.5 s input timeout


def test_001_R19_stops_publishing_after_the_lock_is_released(system):
    _, p = system
    p.lock(True)
    assert wait_for(lambda: len(p.stamps) >= 3)
    p.lock(False)
    time.sleep(0.3)
    count = len(p.stamps)
    time.sleep(0.5)
    assert len(p.stamps) == count


def test_D20_telemetry_reports_the_lock(system):
    _, p = system
    p.lock(True)
    assert wait_for(lambda: p.telemetry is not None and any(
        kv.key == 'locked' and kv.value == 'True' for kv in p.telemetry.status[0].values), timeout=4.0)
