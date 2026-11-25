#!/usr/bin/env python3
"""Deterministic wheel check (local-only): stepping mode, one set_wheel_speeds write per step (the bridge's ZmqSimClient),
ground truth pose and joint velocities read after each step. Usage: bridge_live_stepping.py WORKTREE_SRC v w seconds [two_calls]
`two_calls` replaces the atomic write by two plain setFloatSignal calls (right then left) for comparison."""
import math, sys
sys.path.insert(0, sys.argv[1] + '/vacuum_sim_bridge')
from vacuum_sim_bridge.sim_client import ZmqSimClient
v, w, secs = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]); two = len(sys.argv) > 5
c = ZmqSimClient(); sim = c._sim
robot = sim.getObject('/myRobot'); lj = sim.getObject('/myRobot/leftMotor'); rj = sim.getObject('/myRobot/rightMotor')
sim.setStepping(True)
def pose():
    p = sim.getObjectPose(robot, -1); q = p[3:]
    return p[0], p[1], math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
x0, y0, yaw0 = pose(); t0 = sim.getSimulationTime(); half = 0.1
l, r = v - w * half, v + w * half
rows = []
while sim.getSimulationTime() - t0 < secs:
    if two:
        sim.setFloatSignal(f'{robot}rightVel', r); sim.setFloatSignal(f'{robot}leftVel', l)
    else:
        c.set_wheel_speeds(robot, l, r)
    sim.step()
    x, y, yaw = pose(); rows.append((sim.getSimulationTime() - t0, x, y, yaw, -sim.getJointVelocity(lj) * 0.05, -sim.getJointVelocity(rj) * 0.05))
sim.setStepping(False)
c.set_wheel_speeds(robot, 0.0, 0.0)
fx, fy = -math.sin(yaw0), math.cos(yaw0)
t, x, y, yaw, wl, wr = zip(*rows)
sel = [i for i, tt in enumerate(t) if tt > 2.0]
dist = lambda i: (x[i] - x0) * fx + (y[i] - y0) * fy
sp = (dist(-1) - dist(sel[0])) / (t[-1] - t[sel[0]])
print('steps=%d simtime=%.2f s atomic=%s' % (len(rows), t[-1], not two))
print('forward %.3f m, lateral %+.3f m, yaw change %+.2f deg' % (dist(-1), -(x[-1]-x0)*fy + (y[-1]-y0)*fx, math.degrees((yaw[-1] - yaw0 + math.pi) % (2*math.pi) - math.pi)))
print('speed after 2 s (ground truth) %.3f m/s (command %.3f); wheel joint speeds L mean %.3f R mean %.3f m/s; max |L-R| %.4f' % (
    sp, v, sum(wl[i] for i in sel)/len(sel), sum(wr[i] for i in sel)/len(sel), max(abs(wl[i]-wr[i]) for i in sel)))
