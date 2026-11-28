"""The real escape_server exits cleanly on SIGINT, also with an escape goal running (005-R12, Art. III).

Runs the real `main()` in a subprocess and sends SIGINT as `ros2 launch` does at teardown: it must exit within a few
seconds with no `context is invalid` errors, and a running goal must end with its brake command and silence. Domain:
range 1-5 of this package (a fresh one per round), LOCALHOST discovery.
"""
import os
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402
from lifecycle_msgs.msg import State, Transition  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from vacuum_interfaces.action import Escape  # noqa: E402

from lifecycle_client import change_state, spin_waiter, wait_ready  # noqa: E402
from world_fixtures import QOS  # noqa: E402

CODE = 'from vacuum_coverage.escape_server_node import main; main()'
EXIT_DEADLINE_S = 10.0
BAD_STDERR = ('context is not valid', 'context is invalid', 'Traceback', 'RCLError', 'Error raised in')


def _spin_until(node, done, timeout):
    end = time.monotonic() + timeout
    while not done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return done()


@pytest.mark.parametrize('goal_running', [False, True])
def test_005_R12_escape_server_exits_on_sigint_with_a_brake_and_no_rcl_errors(goal_running):
    # the conftest's random base domain (range 1-5); restored afterwards, the global environment is not kept
    base = int(os.environ['ROS_DOMAIN_ID'])
    domain = str(1 + (base - 1 + int(goal_running)) % 5)
    saved = os.environ['ROS_DOMAIN_ID']
    os.environ['ROS_DOMAIN_ID'] = domain
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    cmd = [sys.executable, '-c', CODE, '--ros-args', '-p', 'outcome:=hang', '-p', 'reverse_s:=0.1',
           '-p', 'turn_s:=0.1']
    proc = subprocess.Popen(cmd, env=env, stderr=subprocess.PIPE, text=True)
    rclpy.init()
    node = rclpy.create_node('shutdown_probe')
    msgs = []
    node.create_subscription(Twist, '/myRobot/mux/escape', msgs.append, QOS)
    try:
        wait = spin_waiter(node)
        assert wait_ready(node, '/escape_server', wait, total_s=60.0), 'escape_server never came up'
        assert change_state(node, '/escape_server', Transition.TRANSITION_CONFIGURE,
                            State.PRIMARY_STATE_INACTIVE, wait)
        assert change_state(node, '/escape_server', Transition.TRANSITION_ACTIVATE,
                            State.PRIMARY_STATE_ACTIVE, wait)
        if goal_running:
            action = ActionClient(node, Escape, '/myRobot/escape')
            assert action.wait_for_server(timeout_sec=30.0)
            fut = action.send_goal_async(Escape.Goal(timeout_s=60.0))
            assert _spin_until(node, fut.done, 20.0) and fut.result().accepted
            assert _spin_until(node, lambda: len(msgs) > 5, 20.0), 'the goal never published'
        proc.send_signal(signal.SIGINT)
        try:
            err = proc.communicate(timeout=EXIT_DEADLINE_S)[1]
        except subprocess.TimeoutExpired:
            proc.kill()
            err = proc.communicate()[1]
            pytest.fail(f'escape_server did not exit within {EXIT_DEADLINE_S} s of SIGINT:\n{err}')
        assert proc.returncode == 0, err
        for bad in BAD_STDERR:
            assert bad not in err, err
        if goal_running:
            _spin_until(node, lambda: False, 0.5)       # drain what is in flight
            last = msgs[-1]
            assert last.linear.x == 0.0 and last.angular.z == 0.0    # the brake before the process ended
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()
        node.destroy_node()
        rclpy.try_shutdown()
        os.environ['ROS_DOMAIN_ID'] = saved
