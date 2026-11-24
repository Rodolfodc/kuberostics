"""T004 / D-11: steps per second of the House scene with the myRobot model, stepping mode.

The House scene (scene_house.ttt) has no robot. The robot comes from the 2025S2 scene
`pega_banana.ttt` (has /myRobot/{leftMotor,rightMotor,kinect,bumper,odometry,...}); this script
exports it once as a model and loads it into the House scene copy started by tools/sim.sh.

Usage: tools/sim.sh start 23000 && python3 tools/sim_bench/bench.py [--port 23000] [--steps 2000]
Rule (spec 001 plan): >= 200 steps/s -> train RL in the simulator; otherwise in the 2D env.
"""
import argparse, os, shutil, sys, time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=23000)
ap.add_argument("--steps", type=int, default=2000)
ap.add_argument("--warmup", type=int, default=200)
ap.add_argument("--model-src", default=os.environ.get("SIM_MODEL_SRC"),
                help="2025S2 pega_banana.ttt, only needed to export .sim/myRobot.ttm the first time (or env SIM_MODEL_SRC)")
ap.add_argument("--no-kinect", action="store_true", help="remove /myRobot/kinect/{rgb,depth} vision sensors before stepping (physics-only ceiling)")
ap.add_argument("--no-robot", action="store_true", help="House scene alone (no robot model), measures the scene baseline")
ap.add_argument("--wheel-speed", type=float, default=3.0, help="rad/s on both wheels")
a = ap.parse_args()

work = os.path.abspath(".sim"); os.makedirs(work, exist_ok=True)
house = os.path.join(work, f"scene-{a.port}.ttt")
model = os.path.join(work, "myRobot.ttm")
sim = RemoteAPIClient(port=a.port).require("sim")
if sim.getSimulationState() != sim.simulation_stopped:   # leftover run from a previous attempt
    sim.setStepping(False); sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped: time.sleep(0.1)

if not os.path.exists(model):                     # export /myRobot once, from a copy
    if not a.model_src or not os.path.isfile(a.model_src):
        sys.exit(f"{model} does not exist: pass --model-src /path/to/pega_banana.ttt or set SIM_MODEL_SRC (got {a.model_src!r})")
    tmp = os.path.join(work, "pega_banana_copy.ttt"); shutil.copy(a.model_src, tmp)
    sim.loadScene(tmp)
    sim.saveModel(sim.getObject("/myRobot"), model)
    print("exported", model)

sim.loadScene(house)                              # the copy, never the original
robot = None if a.no_robot else sim.loadModel(model)
if robot is not None:
    room = sim.getObject("/house/livingroom")
    p = sim.getObjectPosition(robot, sim.handle_world)
    rx, ry, _ = sim.getObjectPosition(room, sim.handle_world)
    sim.setObjectPosition(robot, sim.handle_world, [rx, ry, p[2]])
    if a.no_kinect:
        # remove the Kinect vision sensors (the render cost); removing only the kinect shape tree left them in place
        sim.removeObjects([sim.getObject("/myRobot/kinect/depth"), sim.getObject("/myRobot/kinect/rgb")], True)
    left, right = sim.getObject("/myRobot/leftMotor"), sim.getObject("/myRobot/rightMotor")
print("dt (s):", sim.getSimulationTimeStep(), "| objects:", len(sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)))

sim.setStepping(True)
sim.startSimulation()
pos = lambda: sim.getObjectPosition(robot, sim.handle_world)
path = 0.0
try:
    if robot is not None:
        sim.setJointTargetVelocity(left, a.wheel_speed); sim.setJointTargetVelocity(right, a.wheel_speed)
        start = last = pos()
    for _ in range(a.warmup):
        sim.step()
        if robot is not None:                          # path length sampled during warmup only (not timed)
            cur = pos(); path += sum((c - l) ** 2 for c, l in zip(cur, last)) ** 0.5; last = cur
    t0 = time.perf_counter()
    for _ in range(a.steps): sim.step()
    dt = time.perf_counter() - t0
    end = pos() if robot is not None else None
finally:
    sim.stopSimulation()
if robot is None:
    moved = "n/a"
else:
    moved = f"{sum((e - s) ** 2 for e, s in zip(end, start)) ** 0.5:.2f} m (path length during warmup: {path:.2f} m)"
print(f"steps/s: {a.steps / dt:.1f}  ({a.steps} steps in {dt:.2f}s)  | simulated time: {a.steps * sim.getSimulationTimeStep():.1f}s | net displacement over warmup+steps: {moved}")
