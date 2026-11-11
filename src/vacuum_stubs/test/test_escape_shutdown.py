"""T021: escape_server exits cleanly on SIGINT, also with an escape goal running (regression of
'RCLError: context is not valid'). Runs the real `main()` in a subprocess and sends SIGINT as `ros2 launch`
does at teardown. Requirement: 001-R07 (the escape server must not outlive launch teardown by 5 s, Art. III).
"""
import os
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from lifecycle_msgs.msg import Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState, GetState  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from vacuum_interfaces.action import Escape  # noqa: E402

CODE = ('import rclpy; from rclpy.parameter import Parameter; '
        'from vacuum_stubs.escape_server_node import main; main()')


def _spin_until(node, done, timeout=10.0):
    end = time.monotonic() + timeout
    while not done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return done()


def _warm_up(node, name):
    """Calls the idempotent get_state until a reply arrives: a reply sent before the client's reader is matched
    is lost, which would make the first ChangeState call hang."""
    client = node.create_client(GetState, f'{name}/get_state')
    assert client.wait_for_service(timeout_sec=15.0), f'{name} never came up'
    end = time.monotonic() + 15.0
    while time.monotonic() < end:
        fut = client.call_async(GetState.Request())
        stop = time.monotonic() + 1.0
        while not fut.done() and time.monotonic() < stop:
            rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return
    pytest.fail(f'{name} never answered get_state')


def _change(node, client, transition_id):
    fut = client.call_async(ChangeState.Request(transition=Transition(id=transition_id)))
    assert _spin_until(node, fut.done) and fut.result().success


@pytest.mark.parametrize('goal_running', [False, True])
@pytest.mark.parametrize('rounds', range(2))
def test_001_R07_escape_server_exits_on_sigint_without_rcl_errors(goal_running, rounds):
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
    cmd = [sys.executable, '-c', CODE.split('; ', 2)[2], '--ros-args', '-p', 'outcome:=hang']
    proc = subprocess.Popen(cmd, env=env, stderr=subprocess.PIPE, text=True)
    rclpy.init()
    node = rclpy.create_node('shutdown_probe')
    try:
        client = node.create_client(ChangeState, '/escape_server/change_state')
        _warm_up(node, '/escape_server')
        assert client.wait_for_service(timeout_sec=15.0)
        _change(node, client, Transition.TRANSITION_CONFIGURE)
        _change(node, client, Transition.TRANSITION_ACTIVATE)
        if goal_running:
            action = ActionClient(node, Escape, '/myRobot/escape')
            assert action.wait_for_server(timeout_sec=10.0)
            fut = action.send_goal_async(Escape.Goal(timeout_s=60.0))
            assert _spin_until(node, fut.done) and fut.result().accepted
            time.sleep(0.3)                   # the goal is publishing twists and feedback
        proc.send_signal(signal.SIGINT)
        try:
            err = proc.communicate(timeout=3.0)[1]
        except subprocess.TimeoutExpired:
            proc.kill()
            err = proc.communicate()[1]
            pytest.fail(f'escape_server did not exit within 3 s of SIGINT:\n{err}')
        assert proc.returncode == 0, err
        for bad in ('context is not valid', 'context is invalid', 'Traceback', '[ERROR]', 'RCLError'):
            assert bad not in err, err
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
