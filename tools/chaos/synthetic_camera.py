#!/usr/bin/env python3
"""Synthetic robot sensors for the local offload run (009a chaos test): no simulator needed.

Publishes a fixed JPEG as `rgb8` /camera (stamped with the robot clock), a constant-depth /camera_depth (32FC1) at the same
stamp, an all-free /topologic_map, and a static TF map -> camera_link, so that detector -> detection_selector -> projector
runs for real. Usage: synthetic_camera.py <image.jpg> [--rate 10] [--depth 3.0]
Runs inside the detector image (rclpy, OpenCV, numpy); ROS_DOMAIN_ID and RMW come from the environment.
"""
import argparse

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import StaticTransformBroadcaster

# Same QoS as the real publishers (vacuum_sim_bridge vision_node and the 2025S2 nodes): reliable, depth 10. image_throttle and
# the projector subscribe reliable, so a best-effort camera would not match them.
SENSOR = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SyntheticCamera(Node):
    def __init__(self, path: str, rate: float, depth_m: float):
        super().__init__('synthetic_camera')
        bgr = cv2.imread(path)
        if bgr is None:
            raise SystemExit(f'cannot read image {path}')
        self._rgb = np.ascontiguousarray(cv2.resize(bgr, (640, 480))[:, :, ::-1])
        self._depth = np.full((480, 640), depth_m, dtype=np.float32)
        self._cam = self.create_publisher(Image, '/camera', SENSOR)
        self._dep = self.create_publisher(Image, '/camera_depth', SENSOR)
        self._map = self.create_publisher(OccupancyGrid, '/topologic_map', LATCHED)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'camera_link'
        tf.transform.translation.x = 1.0
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = -0.5, 0.5, -0.5, 0.5
        tf.header.stamp = self.get_clock().now().to_msg()
        self._tf = StaticTransformBroadcaster(self)
        self._tf.sendTransform(tf)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution, grid.info.width, grid.info.height = 0.5, 200, 200
        grid.info.origin.position.x = grid.info.origin.position.y = -50.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (200 * 200)
        self._grid = grid
        self._frames = 0
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(2.0, self._publish_map)

    def _publish_map(self):
        self._grid.header.stamp = self.get_clock().now().to_msg()
        self._map.publish(self._grid)

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        rgb = Image(height=480, width=640, encoding='rgb8', step=640 * 3, data=self._rgb.tobytes())
        rgb.header.stamp, rgb.header.frame_id = stamp, 'camera_link'
        dep = Image(height=480, width=640, encoding='32FC1', step=640 * 4, data=self._depth.tobytes())
        dep.header.stamp, dep.header.frame_id = stamp, 'camera_link'
        self._cam.publish(rgb)
        self._dep.publish(dep)
        self._frames += 1
        if self._frames % 100 == 0:
            self.get_logger().info(f'{self._frames} frames published')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('image')
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--depth', type=float, default=3.0)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = SyntheticCamera(args.image, args.rate, args.depth)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
