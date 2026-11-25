#!/usr/bin/env python3
"""Prepares the running simulator (tools/sim.sh start, GUI) for the vacuum_sim_bridge live checks (local-only).
Usage: bridge_live_setup.py [--model PATH] [--make-explicit OUT.ttm] [--freerun] [--no-start]
  --make-explicit OUT : load the model, set explicit handling on kinect/rgb and kinect/depth, save a copy to OUT, exit.
  default             : (re)load the model at the livingroom pose, display off, realtime, start the simulation."""
import argparse, os, time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

SIMDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '.sim')
ap = argparse.ArgumentParser()
ap.add_argument('--model', default=os.path.join(SIMDIR, 'myRobotLaserFixed.ttm'))
ap.add_argument('--strip-laser-script', action='store_true', help='remove the laser sweep script (timing experiment: the laser costs ~0.2 s per step)'); ap.add_argument('--hold-signals', action='store_true', help='python_controler keeps the last leftVel/rightVel instead of clearing them after one step'); ap.add_argument('--make-explicit'); ap.add_argument('--no-explicit', action='store_true', help='with --make-explicit: keep auto handling'); ap.add_argument('--rgb-render-mode', type=int, help='set kinect/rgb render mode (7 = stock OpenGL3, leaks; 0 = OpenGL)'); ap.add_argument('--freerun', action='store_true'); ap.add_argument('--no-start', action='store_true')
a = ap.parse_args()
sim = RemoteAPIClient().require('sim')
if sim.getSimulationState() != sim.simulation_stopped:
    sim.stopSimulation(); time.sleep(2)
try:
    sim.removeModel(sim.getObject('/myRobot'))
except Exception:
    pass
r = sim.loadModel(os.path.abspath(a.model))
if a.strip_laser_script:
    for t in (sim.scripttype_simulation, sim.scripttype_customization):
        sc = sim.getScript(t, sim.getObject('/myRobot/LaserScanner_2D'))
        if sc != -1:
            sim.removeScript(sc)
if a.hold_signals:
    sc = sim.getScript(sim.scripttype_simulation, sim.getObject('/myRobot/python_controler'))
    code = sim.getStringProperty(sc, 'code')
    new = ''.join(l for l in code.splitlines(True) if not ('clearFloatSignal' in l and ('leftVel' in l or 'rightVel' in l)))
    assert code.count('sim.clearFloatSignal') - new.count('sim.clearFloatSignal') == 2, 'patch did not apply'
    sim.setStringProperty(sc, 'code', new)
if a.rgb_render_mode is not None:
    sim.setObjectInt32Param(sim.getObject('/myRobot/kinect/rgb'), sim.visionintparam_render_mode, a.rgb_render_mode)
if a.make_explicit:
    for n in ('rgb', 'depth'):
        sim.setExplicitHandling(sim.getObject('/myRobot/kinect/' + n), 0 if a.no_explicit else 1)
    sim.saveModel(r, os.path.abspath(a.make_explicit)); sim.removeModel(r); print('saved', a.make_explicit); raise SystemExit
sim.setObjectPosition(r, -1, [3.35, -1.75, 0.08]); sim.setObjectOrientation(r, -1, [0, 0, -1.5708])
sim.setBoolParam(sim.boolparam_display_enabled, False)
sim.setBoolParam(sim.boolparam_realtime_simulation, not a.freerun)
if not a.no_start:
    sim.startSimulation()
print('robot handle', r, 'explicit rgb/depth:', [sim.getExplicitHandling(sim.getObject('/myRobot/kinect/' + n)) for n in ('rgb', 'depth')])
