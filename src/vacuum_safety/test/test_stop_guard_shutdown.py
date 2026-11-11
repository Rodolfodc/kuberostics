"""T021: stop_guard exits cleanly on SIGINT while active (regression of 'RCLError: context is not valid').

Runs the real `main()` in a subprocess, drives it to ACTIVE through its lifecycle service and sends SIGINT,
as `ros2 launch` does at teardown. Requirement: 001-R19 (the guard is a core node under the lifecycle
manager; a node that ignores SIGINT is killed after 5 s and delays every launch teardown, Art. III).
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

CODE = 'from vacuum_safety.stop_guard_node import main; main()'


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
    end = time.monotonic() + 10.0
    while not fut.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert fut.done() and fut.result().success


@pytest.mark.parametrize('locked', [False, True])
@pytest.mark.parametrize('rounds', range(4))
def test_001_R19_stop_guard_exits_on_sigint_without_rcl_errors(rounds, locked):
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
    proc = subprocess.Popen([sys.executable, '-c', CODE], env=env, stderr=subprocess.PIPE, text=True)
    rclpy.init()
    node = rclpy.create_node('shutdown_probe')
    try:
        client = node.create_client(ChangeState, '/stop_guard/change_state')
        _warm_up(node, '/stop_guard')
        assert client.wait_for_service(timeout_sec=15.0)
        _change(node, client, Transition.TRANSITION_CONFIGURE)
        _change(node, client, Transition.TRANSITION_ACTIVATE)
        if locked:                            # the 10 Hz timer is publishing zeros when SIGINT arrives
            from std_msgs.msg import Bool
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            pub = node.create_publisher(Bool, '/myRobot/mux/lock_stop', qos)
            end = time.monotonic() + 10.0
            while pub.get_subscription_count() < 1 and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            pub.publish(Bool(data=True))
        time.sleep(0.3 + 0.037 * rounds)      # timers running, as in a real run; varies the phase at SIGINT
        proc.send_signal(signal.SIGINT)
        try:
            err = proc.communicate(timeout=3.0)[1]
        except subprocess.TimeoutExpired:
            proc.kill()
            err = proc.communicate()[1]
            pytest.fail(f'stop_guard did not exit within 3 s of SIGINT:\n{err}')
        assert proc.returncode == 0, err
        for bad in ('context is not valid', 'context is invalid', 'Traceback', '[ERROR]', 'RCLError'):
            assert bad not in err, err
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
