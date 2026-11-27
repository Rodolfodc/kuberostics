"""Real-process lifecycle tests of the detection nodes: the node runs as the console script does (rclpy.spin in
main), a client drives its lifecycle services. In-process tests cannot see executor deadlocks of `main()`.

Verifies 009a-R12 (projector must be activatable with no inputs) and D-26 (lifecycle nodes answer their services).
Domain: the per-test domain of conftest.py (160-174).
"""
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
from lifecycle_msgs.msg import Transition                       # noqa: E402
from lifecycle_msgs.srv import ChangeState, GetState             # noqa: E402

BOUND_S = 30.0          # generous: other test runs share the machine; a hang is unbounded


@pytest.fixture()
def ros():
    rclpy.init()
    node = rclpy.create_node('lifecycle_test_client')
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture()
def process():
    procs = []

    def start(module):
        p = subprocess.Popen([sys.executable, '-m', module], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        procs.append(p)
        return p
    yield start
    for p in procs:
        p.terminate()
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()


def _call(node, client, request, bound_s=BOUND_S):
    future = client.call_async(request)
    end = time.monotonic() + bound_s
    while time.monotonic() < end and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.result() if future.done() else None


def _transition(node, name, transition_id):
    client = node.create_client(ChangeState, f'/{name}/change_state')
    assert client.wait_for_service(timeout_sec=BOUND_S), f'{name} lifecycle services never appeared'
    return _call(node, client, ChangeState.Request(transition=Transition(id=transition_id)))


@pytest.mark.parametrize('module,name', [
    ('vacuum_detection.projector_node', 'projector'),
    ('vacuum_detection.image_throttle_node', 'image_throttle'),
    ('vacuum_detection.detection_selector_node', 'detection_selector'),
])
def test_009a_R12_node_activates_alone_with_no_inputs_and_keeps_answering(ros, process, module, name):
    """No camera, no TF, no map: configure and activate complete within the bound and the services stay alive."""
    proc = process(module)
    for tid in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        result = _transition(ros, name, tid)
        assert result is not None, f'{name}: transition {tid} did not complete in {BOUND_S} s (hang)'
        assert result.success, f'{name}: transition {tid} refused'
    state = _call(ros, ros.create_client(GetState, f'/{name}/get_state'), GetState.Request())
    assert state is not None and state.current_state.label == 'active'
    assert proc.poll() is None
