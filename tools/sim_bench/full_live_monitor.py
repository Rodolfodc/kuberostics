#!/usr/bin/env python3
"""Event logger for the full-robot live run (local-only, run in the ROS container with the live graph up).
Writes one line per event to OUT.log (wall s since start): StuckEvent, supervisor submode changes, cover_room and escape
action status/feedback, lifecycle sequencer state changes, coverage ratio (feedback), robot pose (TF map -> base_link, 1 Hz).
Usage: full_live_monitor.py OUT.log [duration_s=3600]; stops at the duration or on SIGINT."""
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import Buffer, TransformListener
from vacuum_interfaces.msg import StuckEvent, Submode
from vacuum_interfaces.action import CoverRoom, Escape

out = sys.argv[1]; dur = float(sys.argv[2]) if len(sys.argv) > 2 else 3600.0
rclpy.init(); n = Node('full_live_monitor'); t0 = time.time(); f = open(out, 'w', buffering=1)
last = {}


def log(msg):
    f.write(f'{time.time() - t0:8.1f} {msg}\n')


def change(key, value, text):
    if last.get(key) != value:
        last[key] = value; log(text)


n.create_subscription(StuckEvent, '/myRobot/stuck', lambda m: log(f'STUCK cause={m.cause} pose=({m.pose.position.x:.2f},{m.pose.position.y:.2f})'), 10)
n.create_subscription(Submode, '/myRobot/behaviour/submode', lambda m: change('sub', (m.mode, m.submode), f'SUBMODE {m.mode}/{m.submode} reason={m.reason}'), 10)
names = {1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
for tag, topic in (('cover', '/myRobot/cover_room/_action/status'), ('escape', '/myRobot/escape/_action/status')):
    n.create_subscription(GoalStatusArray, topic, lambda m, tag=tag: change(tag, tuple((bytes(s.goal_info.goal_id.uuid).hex()[:6], s.status) for s in m.status_list),
                                                                           f'{tag.upper()} goals ' + ' '.join(f'{bytes(s.goal_info.goal_id.uuid).hex()[:6]}:{names.get(s.status, s.status)}' for s in m.status_list)), 10)
n.create_subscription(CoverRoom.Impl.FeedbackMessage, '/myRobot/cover_room/_action/feedback', lambda m: log(f'COVER_FB ratio={m.feedback.covered_ratio:.3f}'), 10)
n.create_subscription(Escape.Impl.FeedbackMessage, '/myRobot/escape/_action/feedback', lambda m: log(f'ESCAPE_FB elapsed={m.feedback.elapsed_s:.1f}'), 10)
n.create_subscription(DiagnosticArray, '/myRobot/telemetry/lifecycle_sequencer', lambda m: change('seq', m.status[0].message, f'SEQUENCER {m.status[0].message}'), 10)
n.create_subscription(DiagnosticArray, '/myRobot/telemetry/coverage_server',
                      lambda m: change('covtel', None, 'COVTEL ' + ' '.join(f'{kv.key}={kv.value}' for kv in m.status[0].values)) if time.time() - last.get('covt', 0) > 30 and not last.__setitem__('covt', time.time()) else None, 10)
buf = Buffer(); TransformListener(buf, n); lastpose = 0
while rclpy.ok() and time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.2)
    if time.time() - lastpose >= 1.0:
        lastpose = time.time()
        try:
            t = buf.lookup_transform('map', 'base_link', rclpy.time.Time()); q = t.transform.rotation
            log(f'POSE {t.transform.translation.x:.3f} {t.transform.translation.y:.3f} yaw={math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))):.1f}')
        except Exception:
            pass
