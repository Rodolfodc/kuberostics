"""my_robot_scan_node (plain Node on purpose, see README): /scan from the model's distSignal (replaces the stock node of the same name, D-07)."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from vacuum_sim_bridge.node_support import (declare_connection, declare_profile, make_client,
                                            static_transform)
from vacuum_sim_bridge.scan_core import convert_scan


class ScanNode(Node):
    def __init__(self, client=None, **kwargs):
        super().__init__('my_robot_scan_node', **kwargs)
        declare_connection(self)
        self.declare_parameter('rate_hz', 10.0)
        self._p = declare_profile(self)
        self._client = client if client is not None else make_client(self)
        self._handle = None
        self._warned = False
        self._pub = self.create_publisher(LaserScan, '/scan', 10)
        self._tf = static_transform(self, self._p.base_frame, self._p.laser_frame, self._p.laser_xyz,
                                    (0.0, 0.0, 0.0, 1.0))
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def tick(self):
        try:
            if self._handle is None:
                self._handle = self._client.get_object(self._p.robot_path)
            raw = self._client.get_float_array(self._handle, self._p.scan_signal)
        except Exception as exc:
            self._handle = None
            self.get_logger().warning(f'simulator not ready: {exc}', throttle_duration_sec=5.0)
            return
        scan = convert_scan(raw, self._p)
        if scan is None:
            return
        if scan.warning and not self._warned:
            self._warned = True
            self.get_logger().warning(scan.warning)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._p.laser_frame
        msg.angle_min, msg.angle_max = scan.angle_min, scan.angle_max
        msg.angle_increment = scan.angle_increment
        msg.range_min, msg.range_max = scan.range_min, scan.range_max
        msg.ranges = scan.ranges
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
