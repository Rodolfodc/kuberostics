#!/usr/bin/env python3
"""Renders the Nav2 global costmap, the planned path (/plan), the robot odometry track and the laser scan to a PNG when the
first /plan arrives (local-only, run in the ROS container while a goal is active). Usage: shots_nav.py OUT.png [timeout_s]"""
import math, sys, time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

rclpy.init(); n = Node('shots_nav'); S = {'track': []}
sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: S.__setitem__('cm', m), tl)
n.create_subscription(Path, '/plan', lambda m: S.__setitem__('plan', m) if m.poses else None, 5)
n.create_subscription(Odometry, '/myrobot/odom', lambda m: S['track'].append((m.pose.pose.position.x, m.pose.pose.position.y)), sensor)
n.create_subscription(LaserScan, '/scan', lambda m: S.__setitem__('scan', m), sensor)
t_end = time.time() + float(sys.argv[2] if len(sys.argv) > 2 else 300)
while 'plan' not in S and time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.5)
for _ in range(10):
    rclpy.spin_once(n, timeout_sec=0.5)
cm = S.get('cm')
fig, ax = plt.subplots(figsize=(7, 7))
if cm is not None:
    a = np.array(cm.data, np.int8).reshape(cm.info.height, cm.info.width)
    o, r = cm.info.origin.position, cm.info.resolution
    ax.imshow(a, origin='lower', cmap='magma_r', vmin=0, vmax=100, extent=(o.x, o.x + cm.info.width * r, o.y, o.y + cm.info.height * r))
if 'plan' in S:
    ax.plot([p.pose.position.x for p in S['plan'].poses], [p.pose.position.y for p in S['plan'].poses], 'c-', lw=2, label='/plan')
if S['track']:
    ax.plot(*zip(*S['track']), 'g.-', ms=3, label='odom track'); ax.plot(*S['track'][-1], 'go')
if 'scan' in S and S['track']:
    s = S['scan']  # approximate, drawn from the last odometry point (yaw unknown here: shown only as a range ring hint)
ax.legend(); ax.set_xlabel('x (m, map)'); ax.set_ylabel('y (m, map)'); ax.set_title('Nav2 global costmap and plan')
ax.set_xlim(-2.5, 2.5); ax.set_ylim(-2.5, 2.5)
fig.savefig(sys.argv[1], dpi=100, bbox_inches='tight'); print('saved', sys.argv[1], 'costmap' if cm else 'NO costmap', 'plan' if 'plan' in S else 'NO plan')
