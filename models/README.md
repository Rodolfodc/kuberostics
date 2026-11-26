# CoppeliaSim models

This folder does not version `.ttm` files (see `.gitignore`): the robot model comes from the 2025S2 project (the `IA368II` repository) and the laser from the `slam_toolbox` P3 demo scene. Both are third-party and are not redistributed here. The models are built locally from the sources below.

## `myRobot.ttm` (the 2025S2 robot)

1. `scripts/setup.sh` fetches the `IA368II` repository into `src/deps/` (read-only, Art. II).
2. `python3 tools/sim_bench/bench.py --model-src <path>/pega_banana.ttt` exports `/myRobot` to `.sim/myRobot.ttm`. Instead of `--model-src` you can set the environment variable `SIM_MODEL_SRC`; the file is in the `IA368II` repository (`IA368_ws/src/ia368_pkg/yolo_detector/`).
3. Optional: copy the file into this folder. It stays ignored by git.

The model contains `/myRobot/{leftMotor,rightMotor,kinect,bumper,odometry,battery,dockingSensor,proximitySensor,...}`.

## `myRobotLaserFixed.ttm` (robot plus 2D laser, used by `vacuum_sim_bridge`)

`myRobot.ttm` plus the P3 laser scanner (`/myRobot/LaserScanner_2D`, same sub-object names, same signal `distSignal` on `/myRobot`), with three fixes. About 0.39 MB; not committed, for the reason above.

```bash
SIM_HEADLESS=0 tools/sim.sh start        # a fresh simulator per build (reloading hangs it)
python3 tools/sim_bench/build_robot_model.py \
    --robot models/myRobot.ttm --laser-from-scene <slam_toolbox>/.../p3_slam_toolbox.ttt   # also exports the laser to .sim/p3/laser2d.ttm
tools/sim.sh stop                        # output: .sim/myRobotLaserFixed.ttm
```

What the script does:

1. Mounts the laser so that `Laserscanner_ref` (the scan frame) sits at (0, 0, 0.16) in the robot frame, yawed +90 degrees (beam 0 degrees = robot forward, +Y of the model; index 0 = -90 degrees = right side).
2. Sets the render mode of `kinect/rgb` and `kinect/depth` to 0. The stock mode 7 (OpenGL3) leaks about 1.3 MB of memory per render.
3. Patches the scripts:
   - `python_controler` no longer clears the float signals `<handle>leftVel` / `<handle>rightVel` after one step (the two `sim.clearFloatSignal` lines are removed), so the wheels hold their last command until a new one is written.
   - the laser script is replaced by `tools/sim_bench/robot_laser_sweep.lua`: no graph calls, no unused point cloud, and a sweep only every Nth step. Settings are the model customData of `LaserScanner_2D`: `scanEveryN` (default 1) and `scanDensity` (beams per degree, default 2.5, which gives the 452 beams over -90..+90.4 degrees). `distSignal` keeps the same contract (452 floats, 0 for a miss, now set explicitly when the sensor reports no detection) and holds its last value between sweeps. The float signal `laserSweepSeconds` reports the wall time of the last sweep.

Cost of the laser (452 beams, about 0.22 s per sweep on the author's machine) and steps per second for `scanEveryN` = 1, 2, 4: `tools/sim_bench/laser_sweep_bench.sh .sim/myRobotLaserFixed.ttm`.
