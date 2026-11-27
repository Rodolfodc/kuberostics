#!/usr/bin/env python3
"""RSS-growth bisect for CoppeliaSim (local-only; needs a running simulator started by tools/sim.sh, GUI mode).
Builds one configuration in the running scene, disables rendering (display_enabled=False), runs the simulation for
--minutes while sampling the simulator RSS every --interval s, and prints the least-squares slope (MB/min) after a
warm-up. Usage: rss_bisect.py CONFIG [--pid PID] [--model PATH] [--minutes 5] [--interval 30] [--out CSV]
CONFIG: house | robot_bare | robot_kinect | robot_laser | robot_full | poll_shim | poll_vision | poll_both
  robot_bare   : robot, laser script removed, kinect removed
  robot_kinect : robot, laser script removed
  kinect_lowres / kinect_explicit / kinect_ondemand : robot_kinect with 160x120 / explicit handling never triggered / explicit handling triggered every 3.3 s
  robot_laser  : robot, kinect removed
  robot_full   : whole robot, no client attached
  poll_shim    : robot_full + ZMQ loop at 10 Hz like diag_shim (2 setFloatSignal, getFloatArrayProperty, setBufferProperty)
  poll_vision  : robot_full + getVisionSensorImg/Depth of kinect at 0.3 Hz each (what my_robot_vision_node does)
  poll_both    : both pollers"""
import argparse, os, subprocess, sys, threading, time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

HERE = os.path.dirname(os.path.abspath(__file__))
SIMDIR = os.path.join(HERE, '..', '..', '.sim')


def rss_mb(pid):
    kids = subprocess.run(['pgrep', '-P', str(pid)], capture_output=True, text=True).stdout.split()
    tot = 0
    for p in [str(pid)] + kids:
        o = subprocess.run(['ps', '-o', 'rss=', '-p', p], capture_output=True, text=True).stdout.split()
        tot += int(o[0]) if o else 0
    return tot / 1024.0


def strip_scripts(sim, h):
    for t in (sim.scripttype_simulation, sim.scripttype_customization):
        try:
            s = sim.getScript(t, h)
        except Exception:
            continue
        if s != -1:
            sim.removeScript(s)


def build(sim, cfg, model):
    if cfg == 'house':
        return None
    r = sim.loadModel(os.path.abspath(model))
    sim.setObjectPosition(r, -1, [3.35, -1.75, 0.08]); sim.setObjectOrientation(r, -1, [0, 0, -1.5708])
    if cfg in ('robot_bare', 'robot_kinect') or cfg.startswith('kinect_'):
        strip_scripts(sim, sim.getObject('/myRobot/LaserScanner_2D'))
    for n in ('rgb', 'depth'):
        h = sim.getObject('/myRobot/kinect/' + n) if cfg.startswith('kinect_') else None
        if cfg == 'kinect_lowres':
            sim.setObjectInt32Param(h, sim.visionintparam_resolution_x, 160); sim.setObjectInt32Param(h, sim.visionintparam_resolution_y, 120)
        if cfg in ('kinect_explicit', 'kinect_ondemand'):
            sim.setExplicitHandling(h, 1)
    if cfg in ('robot_bare', 'robot_laser'):
        sim.removeModel(sim.getObject('/myRobot/kinect'))
    return r


def poll_shim(sim, robot, stop):
    while not stop.is_set():
        sim.setFloatSignal(str(robot) + 'rightVel', 0.0); sim.setFloatSignal(str(robot) + 'leftVel', 0.0)
        a = sim.getFloatArrayProperty(robot, 'signal.distSignal', {'noError': True})
        if a:
            sim.setBufferProperty(sim.handle_scene, 'signal.myRobot_scan', sim.packFloatTable(a))
        time.sleep(0.1)


def poll_vision(sim, stop):
    rgb = sim.getObject('/myRobot/kinect/rgb'); dep = sim.getObject('/myRobot/kinect/depth')
    while not stop.is_set():
        sim.getVisionSensorImg(rgb); sim.getVisionSensorDepth(dep)
        time.sleep(3.3)


def handle_vision(sim, stop):
    rgb = sim.getObject('/myRobot/kinect/rgb'); dep = sim.getObject('/myRobot/kinect/depth')
    while not stop.is_set():
        sim.handleVisionSensor(rgb); sim.handleVisionSensor(dep); sim.getVisionSensorImg(rgb); sim.getVisionSensorDepth(dep)
        time.sleep(3.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('config'); ap.add_argument('--pid', type=int)
    ap.add_argument('--model', default=os.path.join(SIMDIR, 'myRobotLaser.ttm'))
    ap.add_argument('--minutes', type=float, default=5); ap.add_argument('--interval', type=float, default=30)
    ap.add_argument('--warmup', type=float, default=60, help='seconds ignored by the slope fit')
    ap.add_argument('--freerun', action='store_true'); ap.add_argument('--out')
    a = ap.parse_args()
    pid = a.pid or int(open(os.path.join(SIMDIR, 'sim-23000.pid')).read())
    sim = RemoteAPIClient().require('sim')
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation(); time.sleep(2)
    robot = build(sim, a.config, a.model)
    sim.setBoolParam(sim.boolparam_display_enabled, False)
    sim.setBoolParam(sim.boolparam_realtime_simulation, not a.freerun)   # default realtime: ~20 steps/s like the soak
    sim.startSimulation()
    stop = threading.Event(); th = []
    if a.config in ('poll_shim', 'poll_both'):
        th.append(threading.Thread(target=poll_shim, args=(RemoteAPIClient().require('sim'), robot, stop)))
    if a.config == 'kinect_ondemand':
        th.append(threading.Thread(target=handle_vision, args=(RemoteAPIClient().require('sim'), stop)))
    if a.config in ('poll_vision', 'poll_both'):
        th.append(threading.Thread(target=poll_vision, args=(RemoteAPIClient().require('sim'), stop)))
    for t in th:
        t.daemon = True; t.start()
    t0 = time.time(); xs = []; ys = []
    while time.time() - t0 <= a.minutes * 60:
        el = time.time() - t0; m = rss_mb(pid)
        st = sim.getSimulationTime()
        print('%s t=%.0fs rss=%.0fMB simtime=%.1f' % (a.config, el, m, st), flush=True)
        if a.out:
            open(a.out, 'a').write('%s,%.0f,%.1f,%.1f\n' % (a.config, el, m, st))
        if el >= a.warmup:
            xs.append(el / 60); ys.append(m)
        time.sleep(a.interval)
    stop.set()
    n = len(xs); mx = sum(xs) / n; my = sum(ys) / n
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    print('RESULT %s slope=%.1f MB/min (n=%d, rss %.0f -> %.0f MB, simtime %.0f s)' % (a.config, slope, n, ys[0], ys[-1], sim.getSimulationTime()))
    sim.stopSimulation()


main()
