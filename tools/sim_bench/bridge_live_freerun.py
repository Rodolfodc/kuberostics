#!/usr/bin/env python3
"""Free-running (realtime, not stepping) wheel check (local-only): tight loop of ZmqSimClient.set_wheel_speeds writes from
one client, ground truth sampled by a second client in another thread. Usage: bridge_live_freerun.py WORKTREE_SRC v w seconds [period_s]
period_s > 0 spaces the writes (e.g. 0.05 = 20 Hz like the bridge's timer, but without any other ZMQ clients)."""
import math, sys, threading, time
sys.path.insert(0, sys.argv[1] + '/vacuum_sim_bridge')
from vacuum_sim_bridge.sim_client import ZmqSimClient
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
v, w, secs = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]); period = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
c = ZmqSimClient(); c.get_sim_time(); sim = c._sim; robot = sim.getObject('/myRobot')
gt = RemoteAPIClient().require('sim'); lj = gt.getObject('/myRobot/leftMotor'); rj = gt.getObject('/myRobot/rightMotor')
rows = []; stop = threading.Event()
def pose():
    p = gt.getObjectPose(robot, -1); q = p[3:]
    return p[0], p[1], math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
def sampler():
    while not stop.is_set():
        t = gt.getSimulationTime(); x, y, yaw = pose(); rows.append((t, x, y, yaw, -gt.getJointVelocity(lj) * 0.05, -gt.getJointVelocity(rj) * 0.05))
x0, y0, yaw0 = pose(); t0 = gt.getSimulationTime(); half = 0.1; l, r = v - w * half, v + w * half
th = threading.Thread(target=sampler, daemon=True); th.start()
writes = 0
while c.get_sim_time() - t0 < secs:
    c.set_wheel_speeds(robot, l, r); writes += 1
    if period: time.sleep(period)
stop.set(); th.join(); c.set_wheel_speeds(robot, 0.0, 0.0)
fx, fy = -math.sin(yaw0), math.cos(yaw0)
t, x, y, yaw, wl, wr = zip(*rows); n = len(rows)
sel = [i for i, tt in enumerate(t) if tt - t0 > 2.0]
d = lambda i: (x[i] - x0) * fx + (y[i] - y0) * fy
print('writes=%d (%.1f per sim-s) samples=%d' % (writes, writes / secs, n))
print('forward %.3f m, yaw change %+.2f deg, speed after 2 s %.3f m/s (command %.3f), wheels L mean %.3f R mean %.3f, wheel-speed zero samples %d of %d' % (
    d(-1), math.degrees((yaw[-1] - yaw0 + math.pi) % (2 * math.pi) - math.pi), (d(-1) - d(sel[0])) / (t[-1] - t[sel[0]]), v,
    sum(wl[i] for i in sel) / len(sel), sum(wr[i] for i in sel) / len(sel), sum(1 for i in sel if abs(wl[i]) < 0.02), len(sel)))
