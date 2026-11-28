#!/usr/bin/env python3
"""Stuck scenario of the full-robot live run (local-only, run in the ROS container with the live graph up and the supervisor in
COVERING). Steers the robot toward a point TARGET_X TARGET_Y (map frame, a wall behind it) through /myRobot/mux/teleop (priority 100,
above coverage), keeps pushing until a StuckEvent arrives (or the push timeout), then stops publishing and follows the escape for
FOLLOW_S seconds, logging pose and front clearance (min of the scan within +-20 deg). Output lines go to stdout, wall seconds since the start.
Usage: full_live_stuck.py (TARGET_X TARGET_Y | straight) [push_timeout_s=240] [follow_s=90]"""
import math, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from vacuum_interfaces.msg import StuckEvent
from action_msgs.msg import GoalStatusArray

STRAIGHT = sys.argv[1] == 'straight'   # constant forward command (no steering), toward a box or wall placed ahead
tx, ty = (0.0, 0.0) if STRAIGHT else (float(sys.argv[1]), float(sys.argv[2]))
push_timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 240.0; follow_s = float(sys.argv[4]) if len(sys.argv) > 4 else 90.0
rclpy.init(); n = Node('full_live_stuck'); t0 = time.time(); S = {'stuck': None, 'scan': None, 'esc': []}
pub = n.create_publisher(Twist, '/myRobot/mux/teleop', 10)
n.create_subscription(StuckEvent, '/myRobot/stuck', lambda m: S.__setitem__('stuck', (time.time() - t0, m)), 10)
n.create_subscription(LaserScan, '/scan', lambda m: S.__setitem__('scan', m), QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
n.create_subscription(GoalStatusArray, '/myRobot/escape/_action/status', lambda m: S['esc'].append((time.time() - t0, [s.status for s in m.status_list])), 10)
buf = Buffer(); TransformListener(buf, n)


def pose():
    t = buf.lookup_transform('map', 'base_link', rclpy.time.Time()); q = t.transform.rotation
    return t.transform.translation.x, t.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def front():
    m = S['scan']
    if m is None:
        return float('nan')
    r = np.array(m.ranges); a = m.angle_min + np.arange(len(r)) * m.angle_increment
    sel = (np.abs(a) < math.radians(20)) & np.isfinite(r) & (r > m.range_min)
    return float(r[sel].min()) if sel.any() else float('nan')


def p(msg):
    print(f'{time.time() - t0:7.1f} {msg}', flush=True)


last = 0.0
while rclpy.ok() and S['stuck'] is None and time.time() - t0 < push_timeout:
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        x, y, yaw = pose()
    except Exception:
        continue
    err = math.atan2(ty - y, tx - x) - yaw; err = math.atan2(math.sin(err), math.cos(err))
    cmd = Twist(); cmd.angular.z = max(-0.6, min(0.6, 1.2 * err)); cmd.linear.x = 0.15 if abs(err) < 0.5 else 0.0
    if STRAIGHT:
        cmd.angular.z = 0.0; cmd.linear.x = 0.15
    pub.publish(cmd)
    if time.time() - last > 5:
        last = time.time(); p(f'PUSH pose=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f} front={front():.2f} cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})')
push_end = time.time() - t0
if S['stuck'] is None:
    p('NO StuckEvent within the push timeout'); sys.exit(1)
x, y, yaw = pose(); fr = front()
p(f'STUCK_EVENT detected at t={S["stuck"][0]:.1f}s cause={S["stuck"][1].cause} pose=({x:.2f},{y:.2f}) front={fr:.2f}; pushing stopped')
x0, y0 = x, y; t_stuck = time.time()
while rclpy.ok() and time.time() - t_stuck < follow_s:
    rclpy.spin_once(n, timeout_sec=0.2)
    if int(time.time() - t_stuck) % 5 == 0 and time.time() - last > 4.5:
        last = time.time(); x, y, yaw = pose(); p(f'FOLLOW pose=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f} front={front():.2f} moved={math.hypot(x - x0, y - y0):.2f}')
x, y, yaw = pose()
p(f'END pose=({x:.2f},{y:.2f}) moved_from_stuck={math.hypot(x - x0, y - y0):.2f} front_before={fr:.2f} front_after={front():.2f} escape_status_changes={S["esc"][-4:]}')
