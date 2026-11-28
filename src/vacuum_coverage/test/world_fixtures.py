"""Fake world for the stuck_detector and escape_server tests: odometry and laser of a robot that obeys only the
Twist received on /myRobot/mux/escape (the wedged robot ignores /myRobot/cmd_vel, which is what "stuck" means).
No simulator. `door_yaw=None` puts a wall ahead at yaw 0 only (a quarter turn frees the front); a number makes
the front free only around that heading."""
import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
MUX = ['teleop', 'stop', 'escape', 'nav', 'coverage', 'docking']


def wait_for(predicate, timeout=20.0, period=0.01):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(period)
    return False


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FakeWorld(Node):
    def __init__(self, context, with_odom=True, with_scan=True, door_yaw=None, always_blocked=False,
                 rate_hz=50.0):
        super().__init__('fake_world', context=context)
        self.x = self.y = self.yaw = 0.0
        self.door_yaw, self.always_blocked = door_yaw, always_blocked
        self.with_odom, self.with_scan = with_odom, with_scan
        self.escape_cmd = Twist()
        self.escape_cmd_t = 0.0
        self.mux_msgs = {name: [] for name in MUX}       # (monotonic time, Twist)
        self.cmd_vel_msgs = []
        self._lock = threading.Lock()
        for name in MUX:
            self.create_subscription(Twist, f'/myRobot/mux/{name}', self._record(name), QOS)
        self.create_subscription(Twist, '/myRobot/mux/escape', self._on_escape, QOS)
        self.create_subscription(Twist, '/myRobot/cmd_vel', self.cmd_vel_msgs.append, QOS)
        self.odom_pub = self.create_publisher(Odometry, '/myrobot/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.dt = 1.0 / rate_hz
        self.create_timer(self.dt, self._tick)

    def _record(self, name):
        def cb(msg):
            with self._lock:
                self.mux_msgs[name].append((time.monotonic(), msg))
        return cb

    def count(self, name):
        with self._lock:
            return len(self.mux_msgs[name])

    def last(self, name):
        with self._lock:
            return self.mux_msgs[name][-1][1] if self.mux_msgs[name] else None

    def total_moving_other_than_escape(self):
        with self._lock:
            return sum(len(v) for k, v in self.mux_msgs.items() if k != 'escape')

    def _on_escape(self, msg):
        self.escape_cmd, self.escape_cmd_t = msg, time.monotonic()

    def front_range(self):
        if self.always_blocked:
            return 0.1
        if self.door_yaw is None:
            return 0.1 if abs(wrap(self.yaw)) < 0.9 else 3.0
        return 3.0 if abs(wrap(self.yaw - self.door_yaw)) < 0.9 else 0.1

    def _tick(self):
        # the robot follows the last escape command for 0.3 s (the mux timeout is 0.5 s)
        if time.monotonic() - self.escape_cmd_t < 0.3:
            v, w = self.escape_cmd.linear.x, self.escape_cmd.angular.z
            self.x += v * math.cos(self.yaw) * self.dt
            self.y += v * math.sin(self.yaw) * self.dt
            self.yaw = wrap(self.yaw + w * self.dt)
        now = self.get_clock().now().to_msg()
        if self.with_odom:
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = now, 'odom'
            odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            self.odom_pub.publish(odom)
        if self.with_scan:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = now, 'laser'
            scan.angle_min, scan.angle_increment, scan.angle_max = -math.pi, math.radians(1.0), math.pi
            scan.range_min, scan.range_max = 0.05, 8.0
            ranges = []
            for i in range(360):
                a = wrap(scan.angle_min + i * scan.angle_increment)
                ranges.append(self.front_range() if abs(a) <= 0.4 else (2.0 if a > 0 else 1.0))
            scan.ranges = ranges
            self.scan_pub.publish(scan)


def forward_command_publisher(node, linear=0.2, rate_hz=20.0, topic='/myRobot/cmd_vel'):
    """Plays twist_mux: publishes the command on the (final) cmd_vel topic with the ia368ii profile."""
    pub = node.create_publisher(Twist, topic, QOS)
    state = {'linear': linear}

    def tick():
        msg = Twist()
        msg.linear.x = state['linear']
        pub.publish(msg)
    timer = node.create_timer(1.0 / rate_hz, tick)
    return state, timer
