"""Nav2 against the real simulator (spec 005, T510 spike). NEEDS CoppeliaSim, local only (Art. V): skipped unless
VACUUM_SIM_LIVE=1. It is NOT run by CI; the sim-operator runs it (ask for it, do not start CoppeliaSim yourself).

Graph the sim-operator must bring up first (same ROS_DOMAIN_ID as this test, use_sim_time false everywhere):
  1. the simulation with models/myRobotLaser.ttm (or the House scene plus the robot), running;
  2. vacuum_sim_bridge sim_bridges.launch.py (my_robot_scan_node, my_robot_odometry_node, my_robot_cmd_vel_node
     with /cmd_vel remapped to /myRobot/cmd_vel);
  3. twist_mux with config/twist_mux.yaml (output remapped to /myRobot/cmd_vel), stop_guard unlocked;
  4. a map and the TF map -> my_robot_odom: nav2_map_server with a saved map plus a static identity transform
     (the test sends goals in `map` relative to the odometry pose, so the identity is assumed), or SLAM;
  5. ros2 launch vacuum_bringup nav.launch.py.
Run: VACUUM_SIM_LIVE=1 python3 -m pytest -m sim test/test_nav_sim_live.py
Optional: NAV_LIVE_GOAL_M (default 0.6, distance of the first goal straight ahead of the robot).

Checks: /myrobot/odom is nav_msgs/Odometry with a non-zero twist while driving (Q-14); a goal moves the real robot
through /myRobot/mux/nav only (005-R19, R23); cancelling stops publication on mux/nav (005-R34, Nav2 side).
"""
import math
import os
import sys
import time

import pytest

pytestmark = [pytest.mark.sim,
              pytest.mark.skipif(os.environ.get('VACUUM_SIM_LIVE') != '1', reason='needs CoppeliaSim')]

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(scope='module')
def probe():
    import nav_support as ns
    p = ns.NavProbe()
    yield p
    p.close()


def current_xy(probe):
    import nav_support as ns
    assert ns.wait_for(lambda: bool(probe.odom), 20.0), '/myrobot/odom (nav_msgs/Odometry) not publishing'
    with probe.lock:
        _, x, y, _, _ = probe.odom[-1]
    return x, y


def test_live_goal_moves_the_robot_through_mux_nav_with_a_usable_odometry_twist(probe):
    import nav_support as ns
    from action_msgs.msg import GoalStatus
    x0, y0 = current_xy(probe)
    distance = float(os.environ.get('NAV_LIVE_GOAL_M', '0.6'))
    # heading is unknown here: ask for a point `distance` away along +x of the map; Nav2 turns as needed
    handle = probe.send(x0 + distance, y0)
    assert handle.accepted
    result = handle.get_result_async()
    assert ns.wait_for(result.done, 180.0), 'goal not finished in 180 s (simulator runs a few steps/s)'
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    with probe.lock:
        cmds, odom = list(probe.cmds), list(probe.odom)
    assert any(v != 0.0 or w != 0.0 for _, v, w in cmds), 'nothing on /myRobot/mux/nav'
    assert any(abs(v) > 0.01 or abs(w) > 0.01 for _, _, _, v, w in odom), 'odometry twist stayed zero while driving'
    _, x1, y1, _, _ = odom[-1]
    assert math.hypot(x1 - x0, y1 - y0) > 0.5 * distance


def test_live_cancel_stops_mux_nav(probe):
    import nav_support as ns
    x0, y0 = current_xy(probe)
    handle = probe.send(x0 - 2.0, y0)
    assert handle.accepted
    before = probe.n_cmds()
    assert ns.wait_for(lambda: probe.n_cmds() >= before + 5, 60.0), 'no velocity before the cancel'
    cancel = handle.cancel_goal_async()
    assert ns.wait_for(cancel.done, 10.0)
    assert probe.wait_silent(1.0, 10.0), 'controller_server kept publishing on /myRobot/mux/nav after the cancel'
