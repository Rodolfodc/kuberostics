#!/usr/bin/env python3
"""Builds a saved occupancy map of the area around a centre by moving the robot (simulator pose, no driving) and
accumulating the real /scan of the bridge at known ground-truth poses (local-only, Art. V; needs the bridges up).
Map frame = simulator world frame (x, y); heading = model yaw + 90 deg (ROS base_link, see the bridge README).
Usage: live_map_scan.py OUT_DIR CX CY [half_size_m=3.0] [grid_step_m=0.5] [grid_half_m=1.0]  -> OUT_DIR/room.pgm, room.yaml"""
import math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

out, cx, cy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
half = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
step = float(sys.argv[5]) if len(sys.argv) > 5 else 0.5
ghalf = float(sys.argv[6]) if len(sys.argv) > 6 else 1.0
RES = 0.05
sim = RemoteAPIClient().require('sim'); robot = sim.getObject('/myRobot')
rclpy.init(); node = Node('live_map_scan'); last = {}
node.create_subscription(LaserScan, '/scan', lambda m: last.__setitem__('m', m), QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
n = int(2 * half / RES); free = np.zeros((n, n), np.int32); occ = np.zeros((n, n), np.int32)
ox, oy = cx - half, cy - half


def cell(x, y):
    return int((x - ox) / RES), int((y - oy) / RES)


def fresh(count=3, timeout=30.0):
    got, t_end, seen = None, time.time() + timeout, 0
    stamp0 = last['m'].header.stamp.sec + last['m'].header.stamp.nanosec * 1e-9 if 'm' in last else 0
    while seen < count and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
        m = last.get('m')
        if m is not None and m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 > stamp0:
            stamp0 = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; seen += 1; got = m
    return got


offs = np.arange(-ghalf, ghalf + 1e-6, step); poses = 0
for dx in offs:
    for dy in offs:
        for hd in (0, 90, 180, 270):
            x, y = cx + dx, cy + dy; psi = math.radians(hd - 90)  # model yaw = ROS heading - 90 deg
            sim.setObjectPose(robot, [x, y, 0.08, 0, 0, math.sin(psi / 2), math.cos(psi / 2)], -1)
            t_set = sim.getSimulationTime()
            while sim.getSimulationTime() < t_set + 0.3:   # the scan must come from a step after the teleport
                time.sleep(0.1)
            m = fresh(2)
            if m is None:
                continue
            poses += 1; th = math.radians(hd); c0 = cell(x, y)
            for i, r in enumerate(m.ranges):
                if not math.isfinite(r) or r < m.range_min:
                    continue
                a = th + m.angle_min + i * m.angle_increment
                hit = r < m.range_max - 0.05; rr = min(r, m.range_max)
                for s in np.arange(0.0, rr - 2 * RES, RES / 2):
                    ci, cj = cell(x + s * math.cos(a), y + s * math.sin(a))
                    if 0 <= ci < n and 0 <= cj < n:
                        free[cj, ci] += 1
                if hit:
                    ci, cj = cell(x + r * math.cos(a), y + r * math.sin(a))
                    if 0 <= ci < n and 0 <= cj < n:
                        occ[cj, ci] += 1
    print('column dx', dx, 'poses so far', poses, flush=True)
img = np.full((n, n), 205, np.uint8); img[free > 0] = 254; img[occ >= 1] = 0
os.makedirs(out, exist_ok=True)
with open(os.path.join(out, 'room.pgm'), 'wb') as f:
    f.write(b'P5\n%d %d\n255\n' % (n, n) + np.flipud(img).tobytes())   # PGM row 0 is the top (max y)
open(os.path.join(out, 'room.yaml'), 'w').write(
    f'image: room.pgm\nmode: trinary\nresolution: {RES}\norigin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print('map saved', out, 'poses', poses, 'free', int((img == 254).sum()), 'occupied', int((img == 0).sum()), 'unknown', int((img == 205).sum()))
