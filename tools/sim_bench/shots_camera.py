#!/usr/bin/env python3
"""Saves one RGB frame (/camera) and one depth frame (/camera_depth) as PNG, via the bridge topics (local-only).
Usage (in the ROS container): shots_camera.py OUT_PREFIX   -> OUT_PREFIX_rgb.png, OUT_PREFIX_depth.png"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

rclpy.init(); n = Node('shots_camera'); got = {}
n.create_subscription(Image, '/camera', lambda m: got.setdefault('rgb', m), 1)
n.create_subscription(Image, '/camera_depth', lambda m: got.setdefault('depth', m), 1)
while len(got) < 2:
    rclpy.spin_once(n, timeout_sec=1.0)
r, d = got['rgb'], got['depth']
rgb = np.frombuffer(r.data, np.uint8).reshape(r.height, r.step // 3, 3)[:, :r.width]
dep = np.frombuffer(d.data, np.float32).reshape(d.height, d.step // 4)[:, :d.width]
plt.imsave(sys.argv[1] + '_rgb.png', rgb)
plt.figure(figsize=(6.4, 4.8)); plt.imshow(dep, cmap='viridis'); plt.colorbar(label='depth (m)'); plt.axis('off')
plt.savefig(sys.argv[1] + '_depth.png', dpi=100, bbox_inches='tight')
print('saved', sys.argv[1], rgb.shape, float(np.nanmin(dep)), float(np.nanmax(dep)))
