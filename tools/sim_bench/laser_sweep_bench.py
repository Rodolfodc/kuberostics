#!/usr/bin/env python3
"""steps/s of the combined robot model versus laser sweep period N (local-only; GUI simulator from `SIM_HEADLESS=0 tools/sim.sh start`).
Run ONE value of N per simulator start (see laser_sweep_bench.sh): stopping and restarting a simulation in one session hangs the simulator.
Realtime off, display off, stepping mode; loads the model, starts, warms up, places the robot in the livingroom, drives 0.2 m/s.
Prints steps/s, the median step time, the in-script sweep wall time (signal laserSweepSeconds) and the signal contract.
Known: some runs are stuck at ~1.05 s per step regardless of N (GUI-side stall, cause not found); they show as median step ~1.05 s.
Usage: laser_sweep_bench.py MODEL.ttm [--n 1] [--freerun SECONDS] [--steps 40] [--kinect-explicit]"""
import argparse, math, statistics, time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
ap = argparse.ArgumentParser(); ap.add_argument('model'); ap.add_argument('--n', type=int, default=1)
ap.add_argument('--steps', type=int, default=40); ap.add_argument('--freerun', type=float, metavar='SECONDS', help='no stepping: measure simulated time per wall second over SECONDS (the GUI stepping stall does not apply)'); ap.add_argument('--kinect-explicit', action='store_true')
a = ap.parse_args()
sim = RemoteAPIClient().require('sim')
sim.setBoolParam(sim.boolparam_display_enabled, False)
sim.setBoolParam(sim.boolparam_realtime_simulation, False)  # else the stepping rate is throttled
room = sim.getObject('/house/livingroom'); rx, ry, _ = sim.getObjectPosition(room, sim.handle_world)
R = sim.loadModel(a.model)
L = sim.getObject('/myRobot/LaserScanner_2D')
if a.kinect_explicit:
    for n in ('rgb', 'depth'): sim.setExplicitHandling(sim.getObject('/myRobot/kinect/' + n), 1)
if a.freerun:
    sim.setObjectPosition(R, sim.handle_world, [rx, ry, 0.08]); sim.setObjectOrientation(R, sim.handle_world, [0, 0, -math.pi / 2])
    sim.setStringProperty(L, 'customData.scanEveryN', str(a.n)); sim.startSimulation(); time.sleep(8)
    sim.setFloatSignal(str(R) + 'rightVel', 0.2); sim.setFloatSignal(str(R) + 'leftVel', 0.2)
    s0, w0 = sim.getSimulationTime(), time.perf_counter(); time.sleep(a.freerun); s1, w1 = sim.getSimulationTime(), time.perf_counter()
    dt = sim.getSimulationTimeStep(); sw = sim.getFloatSignal('laserSweepSeconds'); d = sim.getFloatArrayProperty(R, 'signal.distSignal', {'noError': True}) or []
    print('N=%d kinect_%s free-run: %.2f steps/s over %.0f s wall; sweep wall time %s s; distSignal len %d zeros %d' % (a.n, 'explicit' if a.kinect_explicit else 'auto', (s1 - s0) / dt / (w1 - w0), w1 - w0, '%.3f' % sw if sw is not None else 'n/a', len(d), sum(1 for x in d if x == 0)), flush=True)
    raise SystemExit
sim.setStepping(True); sim.startSimulation()
for _ in range(8): sim.step()  # warm-up: the first steps take ~10 s (first sweep and script start)
sim.setObjectPosition(R, sim.handle_world, [rx, ry, 0.08]); sim.setObjectOrientation(R, sim.handle_world, [0, 0, -math.pi / 2])
sim.setStringProperty(L, 'customData.scanEveryN', str(a.n))
for _ in range(4): sim.step()
p0 = sim.getObjectPosition(R, -1); times = []
for _ in range(a.steps):
    t = time.perf_counter()
    sim.setFloatSignal(str(R) + 'rightVel', 0.2); sim.setFloatSignal(str(R) + 'leftVel', 0.2); sim.step()
    times.append(time.perf_counter() - t)
p1 = sim.getObjectPosition(R, -1)
d = sim.getFloatArrayProperty(R, 'signal.distSignal', {'noError': True}) or []
sw = sim.getFloatSignal('laserSweepSeconds')
print('N=%d kinect_%s: %.2f steps/s (median step %.3f s); sweep wall time %s s; moved %.3f m in %.2f s sim; distSignal len %d min %.2f max %.2f zeros %d' % (
    a.n, 'explicit' if a.kinect_explicit else 'auto', len(times) / sum(times), statistics.median(times),
    '%.3f' % sw if sw is not None else 'n/a', math.dist(p0[:2], p1[:2]), a.steps * 0.05, len(d), min(d, default=0), max(d, default=0),
    sum(1 for x in d if x == 0)), flush=True)
if statistics.median(times) > 0.9:
    print('STALLED start (median step > 0.9 s regardless of N): discard and restart the simulator'); raise SystemExit(3)
