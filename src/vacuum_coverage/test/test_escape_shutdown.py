"""The real escape_server exits cleanly on SIGINT, also with an escape goal running (005-R12, Art. III).

Runs the real `main()` in a subprocess and sends SIGINT as `ros2 launch` does at teardown: it must exit within a few
seconds with no `context is invalid` errors, and a running goal must end with its brake command and silence. Domain:
range 1-5 of this package (a fresh one per round), LOCALHOST discovery.
"""
import itertools
import os
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402
from lifecycle_msgs.msg import Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState, GetState  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from vacuum_interfaces.action import Escape  # noqa: E402

from world_fixtures import QOS  # noqa: E402

CODE = 'from vacuum_coverage.escape_server_node import main; main()'
EXIT_DEADLINE_S = 10.0
BAD_STDERR = ('context is not valid', 'context is invalid', 'Traceback', 'RCLError', 'Error raised in')
_domains = itertools.cycle(range(1, 6))


def _spin_until(node, done, timeout):
    end = time.monotonic() + timeout
    while not done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return done()


def _change(node, client, transition_id):
    fut = client.call_async(ChangeState.Request(transition=Transition(id=transition_id)))
    assert _spin_until(node, fut.done, 20.0) and fut.result().success


@pytest.mark.parametrize('goal_running', [False, True])
def test_005_R12_escape_server_exits_on_sigint_with_a_brake_and_no_rcl_errors(goal_running):
    os.environ['ROS_DOMAIN_ID'] = str(next(_domains))
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    cmd = [sys.executable, '-c', CODE, '--ros-args', '-p', 'outcome:=hang', '-p', 'reverse_s:=0.1',
           '-p', 'turn_s:=0.1']
    proc = subprocess.Popen(cmd, env=env, stderr=subprocess.PIPE, text=True)
    rclpy.init()
    node = rclpy.create_node('shutdown_probe')
    msgs = []
    node.create_subscription(Twist, '/myRobot/mux/escape', msgs.append, QOS)
    try:
        get_state = node.create_client(GetState, '/escape_server/get_state')
        assert get_state.wait_for_service(timeout_sec=60.0), 'escape_server never came up'
        client = node.create_client(ChangeState, '/escape_server/change_state')
        assert client.wait_for_service(timeout_sec=60.0)
        _change(node, client, Transition.TRANSITION_CONFIGURE)
        _change(node, client, Transition.TRANSITION_ACTIVATE)
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
