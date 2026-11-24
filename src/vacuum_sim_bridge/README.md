# vacuum_sim_bridge

CoppeliaSim bridges for the combined model `myRobotLaser.ttm`: `my_robot_scan_node`, `my_robot_vision_node`,
`my_robot_cmd_vel_node`, `my_robot_odometry_node`. Same-name drop-in replacements of the stock IA368II
bridges (Art. II). Start `launch/sim_bridges.launch.py` instead of the stock nodes, never both.

## Plain nodes, not LifecycleNodes (deliberate exception to Art. III)

The stock bridges are plain nodes started by launch and are not managed by the IA368II lifecycle manager.
These replacements keep the same shape so they stay drop-in: nothing in `lifecycle_core.yaml` would configure
or activate them. All state logic is in pure-Python cores; the nodes are thin glue.

## Time stamps

* Message stamps (`header.stamp`, TF) use the node clock (ROS wall time). No `/clock` is published (it would be
  a new topic).
* Odometry velocities use simulation time (`getSimulationTime`) for dt, because the simulator runs at about 1
  to 2 steps/s with the kinect and laser, far from real time.
* Therefore `use_sim_time` must stay `false` (the default) on the bridges and on every node that consumes
  their topics or TF, in every launch. With `use_sim_time: true` and no `/clock`, those nodes never get time.

## Parameters

All robot numbers are parameters with documented defaults in `vacuum_sim_bridge/profile.py`, mirrored in
`config/sim_bridge.yaml` (a test keeps both equal). Extra: `odom_source` (`ground_truth` | `wheels`),
`explicit_sensor_handling` (vision, default false: call `handleVisionSensor` on the kinect rgb and depth before
each frame; use it when the sensors are set to explicit handling so they stop rendering every step, to be
verified live), `zmq_host`, `zmq_port`, rates.

## Tests

Cores and nodes run without the simulator (fake `SimClient`). `test/test_sim_live.py` needs CoppeliaSim and is
local only: `VACUUM_SIM_LIVE=1 python3 -m pytest -m sim test/test_sim_live.py`.
