"""Test helper: lifecycle requests to a node in another process that survive a lost first request or reply.

On Lyrical, under load, the first lifecycle request sent right after `service_is_ready()` is sometimes never
answered (seen as an unanswered CONFIGURE to a subprocess, and as rclpy's "failed to send response (timeout)").
Root cause: in rclpy/executor and DDS matching, not in the nodes. Two measures, in this order:

1. a deterministic precondition: `wait_ready` does `get_state` round trips until one is answered (a reply proves
   the request reader and the reply writer are matched in both directions, which `service_is_ready()` alone does
   not), and only then is the first `change_state` sent;
2. a bounded safety net: when `change_state` still gets no reply, the node's state is read; if it already reached
   the target the request had been served, else the request is sent again (at most `max_resends` times, `total_s`
   overall). Every resend is logged as a WARNING and counted in `RESENDS`, so a CI run that needed it is visible.
"""
import time
import warnings

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

RESENDS = []          # one entry per resend, for the report of a run


def poll_waiter(done, timeout):
    """For tests whose nodes are spun by an executor thread."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if done():
            return True
        time.sleep(0.01)
    return done()


def spin_waiter(node):
    """For tests that spin the node themselves (rclpy.spin_once)."""
    import rclpy

    def wait(done, timeout):
        end = time.monotonic() + timeout
        while not done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        return done()
    return wait


def get_state(node, name, waiter=poll_waiter, timeout=3.0):
    client = node.create_client(GetState, f'{name}/get_state')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(GetState.Request())
        return future.result().current_state.id if waiter(future.done, timeout) else None
    finally:
        node.destroy_client(client)


def wait_ready(node, name, waiter=poll_waiter, total_s=60.0):
    """Precondition: `name`/get_state answered a round trip (see the module docstring)."""
    client = node.create_client(GetState, f'{name}/get_state')
    try:
        end = time.monotonic() + total_s
        while time.monotonic() < end:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                if waiter(future.done, 1.0):
                    return True
        return False
    finally:
        node.destroy_client(client)


def change_state(node, name, transition_id, target_state, waiter=poll_waiter, total_s=30.0, max_resends=2,
                 attempt_s=10.0):
    """Sends `transition_id` to `name` and returns True once the node is in `target_state` (a State id)."""
    if not wait_ready(node, name, waiter):
        return False
    client = node.create_client(ChangeState, f'{name}/change_state')
    try:
        end = time.monotonic() + total_s
        for attempt in range(max_resends + 1):
            future = client.call_async(ChangeState.Request(transition=Transition(id=transition_id)))
            if waiter(future.done, min(attempt_s, max(0.1, end - time.monotonic()))):
                return bool(future.result().success) or get_state(node, name, waiter) == target_state
            client.remove_pending_request(future)
            if get_state(node, name, waiter) == target_state:        # served, only the reply was lost
                return True
            if attempt == max_resends or time.monotonic() >= end:
                return False
            RESENDS.append((name, transition_id))
            msg = (f'lifecycle workaround #{len(RESENDS)}: no reply from {name}/change_state for transition '
                   f'{transition_id}, resending (request or reply lost under load)')
            warnings.warn(msg)
            print('WARNING: ' + msg, flush=True)
        return False
    finally:
        node.destroy_client(client)
