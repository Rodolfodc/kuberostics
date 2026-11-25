"""Fake robot for the Nav2 graph test (not a test module, run as a script by test_nav_graph.py).

Plays the sim bridges and SLAM without CoppeliaSim: integrates /myRobot/mux/nav (unstamped Twist, forward = +x of
base_link) as a unicycle and publishes /myrobot/odom (nav_msgs/Odometry, child base_footprint, with twist), TF
map -> my_robot_odom (identity) -> base_footprint -> base_link -> laser_link, and an empty /scan (every beam reads
beyond the obstacle range, so the costmaps see only the static map).
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def transform(stamp, parent, child, x=0.0, y=0.0, z=0.0, yaw=0.0):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    return t


class FakeRobot(Node):
    def __init__(self, x, y, yaw):
        super().__init__('fake_robot')
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.last_cmd = None
        self.dt = 0.05
        self.create_subscription(Twist, '/myRobot/mux/nav', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, '/myrobot/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf = TransformBroadcaster(self)
        static = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        static.sendTransform([transform(now, 'map', 'my_robot_odom'),
                              transform(now, 'base_footprint', 'base_link', z=0.08),
                              transform(now, 'base_link', 'laser_link', z=0.16)])
        self.create_timer(self.dt, self.step)
        self.create_timer(0.2, self.publish_scan)

    def on_cmd(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z
        self.last_cmd = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        # the base stops by itself when commands stop (the wheel bridge has a 0.5 s command timeout)
        if self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > 0.5:
            self.v = self.w = 0.0
        self.yaw += self.w * self.dt
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        stamp = now.to_msg()
        self.tf.sendTransform(transform(stamp, 'my_robot_odom', 'base_footprint', self.x, self.y, 0.0, self.yaw))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'my_robot_odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(odom)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min, scan.angle_increment = -math.pi / 2, math.pi / 450.0
        scan.angle_max = scan.angle_min + 451 * scan.angle_increment
        scan.range_min, scan.range_max = 0.055, 10.0
        scan.ranges = [9.0] * 452
        self.scan_pub.publish(scan)


def main():
    x, y, yaw = (float(a) for a in sys.argv[1:4])
    rclpy.init()
    rclpy.spin(FakeRobot(x, y, yaw))


if __name__ == '__main__':
    main()
