#!/usr/bin/env python3
"""Sends one CoverRoom goal to the real coverage_server and records the run (local-only, run in the ROS container with the
live graph up: bridges, twist_mux, map_server + map -> my_robot_odom, nav.launch.py, coverage_server active).
Writes OUT.csv (t, wall s, sim-time-free: odom x y, ratio feedback), prints feedback, and OUT.png (covered grid + lanes +
trajectory). Usage: live_cover_run.py OUT_PREFIX [timeout_s=3600]"""
import sys, time, math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from tf2_ros import Buffer, TransformListener
from vacuum_interfaces.action import CoverRoom
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

out, timeout = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 3600.0
rclpy.init(); n = Node('live_cover_run'); S = {'track': [], 'fb': []}
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/myRobot/coverage/covered', lambda m: S.__setitem__('cov', m), tl)
n.create_subscription(OccupancyGrid, '/map', lambda m: S.__setitem__('map', m), tl)
n.create_subscription(Path, '/myRobot/coverage/path', lambda m: S.__setitem__('path', m), 5)   # contract qos: default (volatile), published once per pass
buf = Buffer(); TransformListener(buf, n)
client = ActionClient(n, CoverRoom, '/myRobot/cover_room')
assert client.wait_for_server(timeout_sec=60), 'cover_room action server not there'
t0 = time.time()


def sample():
    try:
        t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        S['track'].append((time.time() - t0, t.transform.translation.x, t.transform.translation.y))
    except Exception:
        pass


for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.2); sample()
goal = CoverRoom.Goal(); goal.room_id = ''; goal.passes = 1
fut = client.send_goal_async(goal, feedback_callback=lambda f: S['fb'].append((time.time() - t0, f.feedback.covered_ratio)))
while not fut.done():
    rclpy.spin_once(n, timeout_sec=0.2)
h = fut.result()
print('goal accepted:', h.accepted, flush=True)
if not h.accepted:
    sys.exit(2)
res = h.get_result_async(); last_print = 0
while not res.done() and time.time() - t0 < timeout:
    rclpy.spin_once(n, timeout_sec=0.1); sample()
    if time.time() - last_print > 30 and S['fb']:
        last_print = time.time(); print(f't={time.time() - t0:6.0f}s ratio={S["fb"][-1][1]:.3f} pose={S["track"][-1][1:] if S["track"] else None}', flush=True)
if not res.done():
    print('TIMEOUT: cancelling', flush=True); h.cancel_goal_async()
    for _ in range(50):
        rclpy.spin_once(n, timeout_sec=0.2)
    r = None
else:
    r = res.result().result
wall = time.time() - t0
print('RESULT', None if r is None else (r.success, round(r.covered_ratio, 3)), 'status', None if r is None else res.result().status,
      f'wall {wall:.0f} s, feedback msgs {len(S["fb"])}', flush=True)
for _ in range(10):
    rclpy.spin_once(n, timeout_sec=0.3)
with open(out + '.csv', 'w') as f:
    f.write('wall_s,x,y\n' + ''.join(f'{a:.2f},{b:.3f},{c:.3f}\n' for a, b, c in S['track']))
with open(out + '_ratio.csv', 'w') as f:
    f.write('wall_s,ratio\n' + ''.join(f'{a:.1f},{b:.4f}\n' for a, b in S['fb']))
fig, ax = plt.subplots(figsize=(7, 7))
cm = S.get('cov')
if cm is not None:
    a = np.array(cm.data, np.int8).reshape(cm.info.height, cm.info.width).astype(float); o, rs = cm.info.origin.position, cm.info.resolution
    img = np.where(a < 0, 0.0, np.where(a >= 100, 2.0, 1.0))
    ax.imshow(img, origin='lower', cmap=matplotlib.colors.ListedColormap(['#444444', '#f0c0c0', '#5cc86c']), vmin=0, vmax=2,
              extent=(o.x, o.x + cm.info.width * rs, o.y, o.y + cm.info.height * rs))
if S.get('path') is not None and S['path'].poses:
    ax.plot([p.pose.position.x for p in S['path'].poses], [p.pose.position.y for p in S['path'].poses], 'b.--', ms=6, lw=1, label='lane waypoints')
if S['track']:
    ax.plot([t[1] for t in S['track']], [t[2] for t in S['track']], 'k-', lw=1.2, label='robot trajectory')
ax.legend(); ax.set_title('coverage: covered (green), uncovered (pink), outside (grey); ratio %.3f' % (r.covered_ratio if r else float('nan')))
ax.set_aspect('equal'); fig.savefig(out + '.png', dpi=100, bbox_inches='tight'); print('saved', out)
