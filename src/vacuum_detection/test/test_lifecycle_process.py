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


def _drive(node, name, transition_id, target_label):
    """Requests the transition until (returns the number of requests sent, None on timeout) the node reports `target_label`, within BOUND_S. A request sent right after
    service discovery can be lost (Lyrical, loaded machine), so a missing answer is retried; a hang never
    reaches the label and fails after the bound."""
    change = node.create_client(ChangeState, f'/{name}/change_state')
    get = node.create_client(GetState, f'/{name}/get_state')
    assert change.wait_for_service(timeout_sec=BOUND_S), f'{name} lifecycle services never appeared'
    end = time.monotonic() + BOUND_S
    sent = 0
    while time.monotonic() < end:
        state = _call(node, get, GetState.Request(), bound_s=3.0)
        if state is not None and state.current_state.label == target_label:
            return sent
        sent += 1
        _call(node, change, ChangeState.Request(transition=Transition(id=transition_id)), bound_s=3.0)
    return None


@pytest.mark.parametrize('module,name', [
    ('vacuum_detection.projector_node', 'projector'),
    ('vacuum_detection.image_throttle_node', 'image_throttle'),
    ('vacuum_detection.detection_selector_node', 'detection_selector'),
])
def test_009a_R12_node_activates_alone_with_no_inputs_and_keeps_answering(ros, process, module, name):
    """No camera, no TF, no map: configure and activate complete within the bound and the services stay alive."""
    proc = process(module)
    for tid, label in ((Transition.TRANSITION_CONFIGURE, 'inactive'), (Transition.TRANSITION_ACTIVATE, 'active')):
        sent = _drive(ros, name, tid, label)
        assert sent is not None, f'{name}: transition to {label} did not complete (hang)'
        # one request is the norm; a second is the tolerated lost-request case; more is a startup race regression
        assert sent <= 2, f'{name}: {sent} requests were needed to reach {label} (startup race regression)'
    assert proc.poll() is None
