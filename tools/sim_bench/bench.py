"""T004 / D-11: steps per second of the House scene with the myRobot model, stepping mode.

The House scene (scene_house.ttt) has no robot. The robot comes from the 2025S2 scene
`pega_banana.ttt` (has /myRobot/{leftMotor,rightMotor,kinect,bumper,odometry,...}); this script
exports it once as a model and loads it into the House scene copy started by tools/sim.sh.

Usage: tools/sim.sh start 23000 && python3 tools/sim_bench/bench.py [--port 23000] [--steps 2000]
Rule (spec 001 plan): >= 200 steps/s -> train RL in the simulator; otherwise in the 2D env.
"""
import argparse, os, shutil, time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

SRC = os.path.expanduser("~/Documents/IA368-II/IA368ii/IA368_ws/src/ia368_pkg/yolo_detector/pega_banana.ttt")
ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=23000)
ap.add_argument("--steps", type=int, default=2000)
ap.add_argument("--warmup", type=int, default=200)
ap.add_argument("--model-src", default=SRC)
ap.add_argument("--no-kinect", action="store_true", help="remove /myRobot/kinect before stepping (physics-only ceiling)")
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
    tmp = os.path.join(work, "pega_banana_copy.ttt"); shutil.copy(a.model_src, tmp)
    sim.loadScene(tmp)
    sim.saveModel(sim.getObject("/myRobot"), model)
    print("exported", model)

sim.loadScene(house)                              # the copy, never the original
robot = sim.loadModel(model)
room = sim.getObject("/house/livingroom")
p = sim.getObjectPosition(robot, sim.handle_world)
rx, ry, _ = sim.getObjectPosition(room, sim.handle_world)
sim.setObjectPosition(robot, sim.handle_world, [rx, ry, p[2]])
if a.no_kinect:
    kin = sim.getObject("/myRobot/kinect"); sim.removeObjects(sim.getObjectsInTree(kin, sim.handle_all, 0) + [kin], True)
left, right = sim.getObject("/myRobot/leftMotor"), sim.getObject("/myRobot/rightMotor")
print("dt (s):", sim.getSimulationTimeStep(), "| objects:", len(sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)))

sim.setStepping(True)
sim.startSimulation()
sim.setJointTargetVelocity(left, a.wheel_speed); sim.setJointTargetVelocity(right, a.wheel_speed)
start = sim.getObjectPosition(robot, sim.handle_world)
for _ in range(a.warmup): sim.step()
t0 = time.perf_counter()
for _ in range(a.steps): sim.step()
dt = time.perf_counter() - t0
end = sim.getObjectPosition(robot, sim.handle_world)
sim.stopSimulation()
moved = sum((e - s) ** 2 for e, s in zip(end, start)) ** 0.5
print(f"steps/s: {a.steps / dt:.1f}  ({a.steps} steps in {dt:.2f}s)  | simulated time: {a.steps * sim.getSimulationTimeStep():.1f}s | robot moved {moved:.2f} m")
