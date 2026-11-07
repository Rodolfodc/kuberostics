# CoppeliaSim models

This folder does not version `.ttm` files (see `.gitignore`): the robot model comes from the 2025S2 project (the `IA368II` repository) and is not redistributed here.

To generate `myRobot.ttm` locally:

1. `scripts/setup.sh` fetches the `IA368II` repository into `src/deps/` (read-only, Art. II).
2. `python3 tools/sim_bench/bench.py --model-src <path>/pega_banana.ttt` exports `/myRobot` to `.sim/myRobot.ttm`. The default of `--model-src` is a local path of the author; set it to where `pega_banana.ttt` is (in the `IA368II` repository, `IA368_ws/src/ia368_pkg/yolo_detector/`).
3. Optional: copy the file into this folder. It stays ignored by git.

The model contains `/myRobot/{leftMotor,rightMotor,kinect,bumper,odometry,battery,dockingSensor,proximitySensor,...}`.
