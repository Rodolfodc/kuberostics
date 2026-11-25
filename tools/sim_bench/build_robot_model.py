#!/usr/bin/env python3
"""Builds the combined robot model myRobot + P3 2D laser with the fixes (local-only; needs a running simulator from
`tools/sim.sh start`, one fresh start per build). Writes .sim/myRobotLaserFixed.ttm by default.

Steps (see models/README.md):
  1. --laser-from-scene P3.ttt : export /myRobot/LaserScanner_2D of the slam_toolbox P3 scene to --laser (skipped if the
     --laser file exists and the option is absent).
  2. load --robot (models/myRobot.ttm), load --laser, mount the laser so that Laserscanner_ref sits at (0,0,0.16) in the
     robot frame, yawed +90 deg (beam 0 deg = robot forward +Y).
  3. fixes: (a) kinect/rgb and kinect/depth render mode 0 (stock mode 7, OpenGL3, leaks ~1.3 MB per render);
     (b) python_controler no longer clears leftVel/rightVel, so the wheels hold the last command;
     (c) the laser script is replaced by tools/sim_bench/robot_laser_sweep.lua (cheaper, sweep every Nth step).
Usage: build_robot_model.py [--robot models/myRobot.ttm] [--laser .sim/p3/laser2d.ttm] [--laser-from-scene P3.ttt] [--out OUT.ttm]"""
import argparse, math, os
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.join(HERE, '..', '..')
ap = argparse.ArgumentParser()
ap.add_argument('--robot', default=os.path.join(REPO, 'models', 'myRobot.ttm'))
ap.add_argument('--laser', default=os.path.join(REPO, '.sim', 'p3', 'laser2d.ttm'))
ap.add_argument('--laser-from-scene')
ap.add_argument('--out', default=os.path.join(REPO, '.sim', 'myRobotLaserFixed.ttm'))
ap.add_argument('--base', help='start from an existing combined model (skips steps 1-2)')
a = ap.parse_args()
ab = lambda p: os.path.abspath(p)
sim = RemoteAPIClient().require('sim')
if sim.getSimulationState() != sim.simulation_stopped:
    sim.stopSimulation()

if a.base:
    R = sim.loadModel(ab(a.base))
else:
    if a.laser_from_scene:
        sim.loadScene(ab(a.laser_from_scene))
        sim.saveModel(sim.getObject('/myRobot/LaserScanner_2D'), ab(a.laser))
        sim.closeScene()
    R = sim.loadModel(ab(a.robot)); Lh = sim.loadModel(ab(a.laser))
    ref = [h for h in sim.getObjectsInTree(Lh, sim.handle_all, 0) if sim.getObjectAlias(h) == 'Laserscanner_ref'][0]
    inv = sim.getPoseInverse(sim.getObjectPose(ref, Lh))
    yaw = math.pi / 2
    sim.setObjectParent(Lh, R, True)
    sim.setObjectPose(Lh, R, sim.multiplyPoses([0, 0, 0.16, 0, 0, math.sin(yaw / 2), math.cos(yaw / 2)], inv))

for n in ('rgb', 'depth'):
    sim.setObjectInt32Param(sim.getObject('/myRobot/kinect/' + n), sim.visionintparam_render_mode, 0)

sc = sim.getScript(sim.scripttype_simulation, sim.getObject('/myRobot/python_controler'))
code = sim.getStringProperty(sc, 'code')
new = ''.join(l for l in code.splitlines(True) if not ('clearFloatSignal' in l and ('leftVel' in l or 'rightVel' in l)))
if code != new:
    assert code.count('sim.clearFloatSignal') - new.count('sim.clearFloatSignal') == 2
    sim.setStringProperty(sc, 'code', new)

lsc = sim.getScript(sim.scripttype_simulation, sim.getObject('/myRobot/LaserScanner_2D'))
sim.setStringProperty(lsc, 'code', open(os.path.join(HERE, 'robot_laser_sweep.lua')).read())
sim.setStringProperty(sim.getObject('/myRobot/LaserScanner_2D'), 'customData.scanEveryN', '1')
sim.setStringProperty(sim.getObject('/myRobot/LaserScanner_2D'), 'customData.scanDensity', '2.5')

sim.saveModel(R, ab(a.out)); print('saved', ab(a.out), os.path.getsize(ab(a.out)), 'bytes')
sim.removeModel(R)
