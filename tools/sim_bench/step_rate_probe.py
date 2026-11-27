#!/usr/bin/env python3
"""Steps/s and RSS growth of the robot model under one configuration (local-only; GUI simulator from `SIM_HEADLESS=0 tools/sim.sh start`).
One configuration per simulator start (stopping and restarting a simulation inside one session can hang it).
Loads the model in the livingroom, display off unless --display, then measures simulated time per wall second over --seconds.
Modes: free (non-realtime free run), realtime (realtime on, 20 steps/s ideal), stepping (client calls sim.step()).
--poll emulates the three bridges with a ZMQ thread each: odometry 20 Hz (getSimulationTime + getObjectPose),
scan 10 Hz (getFloatArrayProperty distSignal), wheels 20 Hz (2 x setFloatSignal); --poll-rate-scale scales all three rates.
Usage: step_rate_probe.py MODEL.ttm [--mode free] [--seconds 30] [--poll] [--kinect-explicit] [--laser-n N | --no-laser] [--display] [--no-python-script] [--vision-hz HZ] [--no-kinect] [--remove NAME ...] [--pid PID]"""
import argparse, os, subprocess, threading, time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

SIMDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '.sim')


def rss_mb(pid):
    tot = 0
    for p in [str(pid)] + subprocess.run(['pgrep', '-P', str(pid)], capture_output=True, text=True).stdout.split():
        o = subprocess.run(['ps', '-o', 'rss=', '-p', p], capture_output=True, text=True).stdout.split()
        tot += int(o[0]) if o else 0
    return tot / 1024.0


def poller(robot, hz, fn, stop, counter):
    sim = RemoteAPIClient().require('sim')
    period = 1.0 / hz
    while not stop.is_set():
        t = time.time()
        fn(sim, robot); counter[0] += 1
        time.sleep(max(0.0, period - (time.time() - t)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('model'); ap.add_argument('--mode', choices=('free', 'realtime', 'stepping'), default='free')
    ap.add_argument('--seconds', type=float, default=30); ap.add_argument('--poll', action='store_true')
    ap.add_argument('--poll-rate-scale', type=float, default=1.0)
    ap.add_argument('--poll-only', choices=('odom', 'scan', 'wheels'), help='run only this poller')
    ap.add_argument('--kinect-explicit', action='store_true'); ap.add_argument('--display', action='store_true')
    ap.add_argument('--no-laser', action='store_true'); ap.add_argument('--laser-n', type=int)
    ap.add_argument('--no-python-script', action='store_true', help='remove the Python child script of /myRobot (the robot then does not drive)')
    ap.add_argument('--no-kinect', action='store_true', help='remove /myRobot/kinect from the scene')
    ap.add_argument('--remove', action='append', default=[], metavar='NAME', help='remove /myRobot/NAME and its subtree before the start (repeatable)')
    ap.add_argument('--vision-hz', type=float, default=0.0, help='also read kinect rgb + depth images at this rate (what the vision bridge does)')
    ap.add_argument('--pid', type=int)
    a = ap.parse_args()
    pid = a.pid or int(open(os.path.join(SIMDIR, 'sim-23000.pid')).read())
    sim = RemoteAPIClient().require('sim')
    sim.setBoolParam(sim.boolparam_display_enabled, a.display)
    sim.setBoolParam(sim.boolparam_realtime_simulation, a.mode == 'realtime')
    room = sim.getObject('/house/livingroom'); rx, ry, _ = sim.getObjectPosition(room, sim.handle_world)
    r = sim.loadModel(os.path.abspath(a.model))
    laser = sim.getObject('/myRobot/LaserScanner_2D')
    if a.no_laser:
        for t in (sim.scripttype_simulation, sim.scripttype_customization):
            s = sim.getScript(t, laser)
            if s != -1:
                sim.removeScript(s)
    if a.no_python_script:
        for t in (sim.scripttype_simulation, sim.scripttype_customization):
            try:
                s = sim.getScript(t, r)
                if s != -1:
                    sim.removeScript(s)
            except Exception:
                pass
    for name in a.remove:
        sim.removeObjects(sim.getObjectsInTree(sim.getObject('/myRobot/' + name)))
    if a.no_kinect:
        sim.removeModel(sim.getObject('/myRobot/kinect'))
    if a.laser_n:
        sim.setStringProperty(laser, 'customData.scanEveryN', str(a.laser_n))
    if a.kinect_explicit and not a.no_kinect:
        for n in ('rgb', 'depth'):
            sim.setExplicitHandling(sim.getObject('/myRobot/kinect/' + n), 1)
    sim.setObjectPosition(r, sim.handle_world, [rx, ry, 0.08]); sim.setObjectOrientation(r, sim.handle_world, [0, 0, -1.5708])
    if a.mode == 'stepping':
        sim.setStepping(True)
    sim.startSimulation()
    stop = threading.Event(); counts = {}
    fns = {
        'odom': (20, lambda s, h: (s.getSimulationTime(), s.getObjectPose(h, s.handle_world))),
        'scan': (10, lambda s, h: s.getFloatArrayProperty(h, 'signal.distSignal', {'noError': True})),
        'wheels': (20, lambda s, h: (s.setFloatSignal(str(h) + 'rightVel', 0.2), s.setFloatSignal(str(h) + 'leftVel', 0.2))),
    }
    if a.vision_hz:
        rgb, dep = sim.getObject('/myRobot/kinect/rgb'), sim.getObject('/myRobot/kinect/depth')
        counts['vision'] = [0]
        threading.Thread(target=poller, args=(0, a.vision_hz, lambda s_, h: (s_.getVisionSensorImg(rgb), s_.getVisionSensorDepth(dep)), stop, counts['vision']), daemon=True).start()
    if a.poll or a.poll_only:
        for k, (hz, fn) in fns.items():
            if a.poll_only and k != a.poll_only:
                continue
            counts[k] = [0]
            threading.Thread(target=poller, args=(r, hz * a.poll_rate_scale, fn, stop, counts[k]), daemon=True).start()
    else:
        sim.setFloatSignal(str(r) + 'rightVel', 0.2); sim.setFloatSignal(str(r) + 'leftVel', 0.2)
    time.sleep(10)   # warm-up: first sweep, script start
    dt = sim.getSimulationTimeStep()
    s0, w0, m0 = sim.getSimulationTime(), time.perf_counter(), rss_mb(pid)
    n0 = 0
    if a.mode == 'stepping':
        t_end = time.perf_counter() + a.seconds
        while time.perf_counter() < t_end:
            sim.step(); n0 += 1
        sim_steps = n0
    else:
        time.sleep(a.seconds)
        sim_steps = (sim.getSimulationTime() - s0) / dt
    w1, m1 = time.perf_counter(), rss_mb(pid)
    sps = sim_steps / (w1 - w0)
    print('RESULT %s mode=%s poll=%s%s kinect=%s laser=%s display=%s%s: %.2f steps/s (x%.3f real time), RSS %.0f -> %.0f MB = %.1f MB/min = %.2f MB/step, polls %s' % (
        os.path.basename(a.model), a.mode, 'all' if a.poll else (a.poll_only or 'no'),
        '' if a.poll_rate_scale == 1 else '@x%g' % a.poll_rate_scale,
        'none' if a.no_kinect else 'explicit' if a.kinect_explicit else 'auto', 'none' if a.no_laser else ('N=%d' % a.laser_n if a.laser_n else 'N=1'),
        'on' if a.display else 'off', (' nopython' if a.no_python_script else '') + (' nokinect' if a.no_kinect else '') + ''.join(' -' + n for n in a.remove), sps, sps * dt, m0, m1, (m1 - m0) / (w1 - w0) * 60,
        (m1 - m0) / max(sim_steps, 1e-9), {k: v[0] for k, v in counts.items()}), flush=True)
    stop.set()


main()
