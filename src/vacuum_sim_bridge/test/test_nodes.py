"""Bridge nodes against a fake simulator client (no ZMQ, no CoppeliaSim).
Basis: spec 000 section 4 and D-06/D-07 (topic names and types stay as in topics.yaml), Art. II (same-name
replacement nodes), robot_report.txt (frames, drive path)."""
import math
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage

from fakes import FakeSim
from vacuum_sim_bridge.cmd_vel_node import CmdVelNode
from vacuum_sim_bridge.geometry import quaternion_from_yaw
from vacuum_sim_bridge.odometry_node import OdometryNode
from vacuum_sim_bridge.scan_node import ScanNode
from vacuum_sim_bridge.vision_node import VisionNode

TIMEOUT = 10.0


@pytest.fixture(autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_until(executor, predicate, timeout=TIMEOUT):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def make(node_cls, sim, overrides=None, **kwargs):
    from rclpy.parameter import Parameter
    params = [Parameter(k, value=v) for k, v in (overrides or {}).items()]
    node = node_cls(client=sim, parameter_overrides=params, **kwargs)
    probe = Node('probe_' + node.get_name())
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(probe)
    return node, probe, ex


def test_scan_node_publishes_inf_for_misses_in_laser_link():
    sim = FakeSim()
    sim.arrays['signal.distSignal'] = [0.0, 1.5] * 226
    node, probe, ex = make(ScanNode, sim, {'rate_hz': 50.0})
    got = []
    probe.create_subscription(LaserScan, '/scan', got.append, 10)
    assert spin_until(ex, lambda: got)
    m = got[0]
    assert m.header.frame_id == 'laser_link' and len(m.ranges) == 452
    assert math.isinf(m.ranges[0]) and m.ranges[1] == pytest.approx(1.5)
    assert m.angle_max == pytest.approx(m.angle_min + 451 * m.angle_increment)
    assert m.range_min == pytest.approx(0.055) and m.range_max == pytest.approx(10.0)
    node.destroy_node()


def test_scan_node_survives_a_simulator_outage():
    sim = FakeSim()
    sim.fail = True
    sim.arrays['signal.distSignal'] = [1.0] * 452
    node, probe, ex = make(ScanNode, sim, {'rate_hz': 50.0})
    got = []
    probe.create_subscription(LaserScan, '/scan', got.append, 10)
    assert not spin_until(ex, lambda: got, timeout=0.5)
    sim.fail = False
    assert spin_until(ex, lambda: got)
    node.destroy_node()


def test_vision_node_publishes_rgb8_and_metric_depth_with_one_stamp():
    sim = FakeSim()
    w, h = 4, 2
    sim.rgb = (bytes(range(w * h * 3)), w, h)
    sim.depth = (np.full(w * h, 0.5, dtype=np.float32).tobytes(), w, h)
    node, probe, ex = make(VisionNode, sim, {'rate_hz': 50.0})
    rgb, depth = [], []
    probe.create_subscription(Image, '/camera', rgb.append, 10)
    probe.create_subscription(Image, '/camera_depth', depth.append, 10)
    assert spin_until(ex, lambda: rgb and depth)
    assert rgb[0].encoding == 'rgb8' and rgb[0].step == w * 3 and rgb[0].header.frame_id == 'camera_link'
    assert depth[0].encoding == '32FC1' and depth[0].step == w * 4
    d = np.frombuffer(bytes(depth[0].data), dtype=np.float32)
    assert d[0] == pytest.approx(0.01 + 0.5 * 3.49, abs=1e-4)
    # both images of one tick carry the same stamp
    assert (rgb[0].header.stamp.sec, rgb[0].header.stamp.nanosec) == (depth[0].header.stamp.sec,
                                                                       depth[0].header.stamp.nanosec)
    node.destroy_node()


def test_cmd_vel_node_writes_wheel_speeds_every_cycle_then_zeroes_on_timeout():
    sim = FakeSim()
    now = [100.0]
    node, probe, ex = make(CmdVelNode, sim, {'write_rate_hz': 1000.0}, clock=lambda: now[0])
    pub = probe.create_publisher(Twist, '/cmd_vel', 10)
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.5
    pub.publish(msg)
    assert spin_until(ex, lambda: len(sim.wheel_writes) >= 3)
    robot, left, right = sim.wheel_writes[-1]
    assert robot == sim.objects['/myRobot']
    assert left == pytest.approx(0.2 - 0.05) and right == pytest.approx(0.2 + 0.05)
    now[0] += 10.0                                       # command stream stopped
    n = len(sim.wheel_writes)
    assert spin_until(ex, lambda: sim.wheel_writes[-1][1:] == (0.0, 0.0) and len(sim.wheel_writes) > n)
    stopped = len(sim.wheel_writes)
    spin_until(ex, lambda: False, timeout=0.2)           # idle: no more writes
    assert len(sim.wheel_writes) == stopped
    node.destroy_node()


def test_odometry_node_publishes_odom_and_tf_in_the_contract_frames():
    sim = FakeSim()
    sim.pose = (3.0, 1.0, 0.08) + quaternion_from_yaw(-math.pi / 2)
    node, probe, ex = make(OdometryNode, sim, {'rate_hz': 50.0})
    odom, tfs = [], []
    probe.create_subscription(Odometry, '/myrobot/odom', odom.append, 10)
    probe.create_subscription(TFMessage, '/tf', tfs.append, 10)
    assert spin_until(ex, lambda: odom and tfs)
    sim.pose = (4.0, 1.0, 0.08) + quaternion_from_yaw(-math.pi / 2)
    sim.sim_time = 5.0
    assert spin_until(ex, lambda: odom[-1].pose.pose.position.x > 0.9)
    m = odom[-1]
    assert m.header.frame_id == 'my_robot_odom' and m.child_frame_id == 'base_footprint'
    assert m.pose.pose.position.x == pytest.approx(1.0)
    assert m.twist.twist.linear.x == pytest.approx(0.2)
    t = tfs[-1].transforms[0]
    assert (t.header.frame_id, t.child_frame_id) == ('my_robot_odom', 'base_footprint')
    node.destroy_node()


def test_odometry_node_wheel_source():
    sim = FakeSim()
    node, probe, ex = make(OdometryNode, sim, {'rate_hz': 50.0, 'odom_source': 'wheels'})
    odom = []
    probe.create_subscription(Odometry, '/myrobot/odom', odom.append, 10)
    assert spin_until(ex, lambda: odom)
    left = sim.objects['/myRobot/leftMotor']
    right = sim.objects['/myRobot/rightMotor']
    for i in range(1, 11):                                     # 0.1 m forward per poll
        sim.joints[left] = sim.joints[right] = -0.1 * i / 0.05
        sim.sim_time = 0.5 * i
        n = len(odom)
        assert spin_until(ex, lambda: len(odom) > n + 1)
    assert spin_until(ex, lambda: odom[-1].pose.pose.position.x > 0.99)
    assert odom[-1].pose.pose.position.x == pytest.approx(1.0)
    node.destroy_node()


def test_odometry_node_rejects_an_unknown_source():
    with pytest.raises(ValueError):
        from rclpy.parameter import Parameter
        OdometryNode(client=FakeSim(), parameter_overrides=[Parameter('odom_source', value='gps')])


def test_vision_node_handles_sensors_only_when_asked():
    sim = FakeSim()
    sim.rgb = (bytes(4 * 2 * 3), 4, 2)
    sim.depth = (np.zeros(8, dtype=np.float32).tobytes(), 4, 2)
    node, probe, ex = make(VisionNode, sim, {'rate_hz': 50.0})
    got = []
    probe.create_subscription(Image, '/camera', got.append, 10)
    assert spin_until(ex, lambda: got)
    assert not getattr(sim, 'handled', [])
    node.destroy_node()
    sim2 = FakeSim()
    sim2.rgb, sim2.depth = sim.rgb, sim.depth
    node, probe, ex = make(VisionNode, sim2, {'rate_hz': 50.0, 'explicit_sensor_handling': True})
    got = []
    probe.create_subscription(Image, '/camera', got.append, 10)
    assert spin_until(ex, lambda: got)
    assert set(sim2.handled) == {sim2.objects['/myRobot/kinect/rgb'], sim2.objects['/myRobot/kinect/depth']}
    node.destroy_node()


def test_odometry_node_warns_on_a_large_wheel_jump():
    sim = FakeSim()
    node, probe, ex = make(OdometryNode, sim, {'rate_hz': 50.0, 'odom_source': 'wheels'})
    warnings = []
    node.get_logger().warning = lambda m, **k: warnings.append(m)
    odom = []
    probe.create_subscription(Odometry, '/myrobot/odom', odom.append, 10)
    assert spin_until(ex, lambda: odom)
    assert not warnings
    sim.joints[sim.objects['/myRobot/leftMotor']] = 2.0           # > quarter turn in one poll
    assert spin_until(ex, lambda: warnings)
    node.destroy_node()
