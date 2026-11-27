"""Node tests of the EXPLORING flow of the supervisor (003-R14, 003-R31) with a fake `explore` server.

The supervisor and a fake Nav2 (`NavigateToPose` server driving `mux/nav`, as in test_supervisor_watchdog.py)
run in this process; the fake `explore` server is a child process. Killing it mid-goal must stop the Nav2
velocity within `feedback_stale_s + nav_cancel_s + 0.5 s` and the supervisor must go on with COVERING.

Only two rclpy tests live here on purpose: conftest.py hands out 30 domains (70-99), one per test, and a package
with more than 30 rclpy tests reuses a domain inside one run (measured: the last tests of test_supervisor_watchdog.py
then fail to discover their servers). The other behaviour (results, stuck, mode changes) is in the pure
test_submode_machine_exploring.py. Keep the total of rclpy tests of this package at 30 or fewer.
"""
import os
import signal
import subprocess
import sys
import threading
import time

import pytest
from test_supervisor_watchdog import BUDGET_S, FakeNav2, NavigateToPose, wait_for

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from vacuum_supervisor.supervisor_node import SupervisorNode  # noqa: E402

FAKE_SERVER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fake_explore_server.py')


@pytest.fixture()
def system():
    rclpy.init()                            # the domain is isolated by conftest.py
    created = {'procs': []}

    def start(*server_args, **params):
        fake = FakeNav2()
        params.setdefault('explore_on_start', True)
        sup = SupervisorNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(sup)
        executor.add_node(fake)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        proc = subprocess.Popen([sys.executable, FAKE_SERVER, *server_args])
        created.update(sup=sup, fake=fake, executor=executor, thread=thread)
        created['procs'].append(proc)
        sup.trigger_configure()
        sup.trigger_activate()
        assert wait_for(lambda: sup._explore_client.server_is_ready()), 'explore server not discovered'
        assert wait_for(lambda: fake.client.server_is_ready()), 'fake Nav2 not discovered'
        return sup, fake, proc

    try:
        yield start
    finally:
        for proc in created['procs']:
            if proc.poll() is None:
                proc.kill()
            proc.wait(timeout=10)
        if 'sup' in created:
            created['fake'].closing = True
            try:
                created['sup'].trigger_deactivate()
                created['sup'].trigger_cleanup()
            except Exception:               # noqa: B902
                pass
            created['executor'].shutdown()
            created['sup'].destroy_node()
            created['fake'].destroy_node()
            created['thread'].join(timeout=2)
        rclpy.try_shutdown()


def begin_exploration_with_nav_goal(sup, fake):
    assert wait_for(lambda: fake.pub_mode.get_subscription_count() >= 1)
    fake.pub_mode.publish(String(data='CLEANING'))
    assert wait_for(lambda: sup._explore_handle is not None), 'explore goal was not accepted'
    assert sup._machine.submode == 'EXPLORING'
    fake.client.send_goal_async(NavigateToPose.Goal())          # what exploration_server does
    assert wait_for(lambda: fake.active and fake.last_nonzero is not None), 'fake Nav2 not driving'


def test_003_R31_lost_explore_server_stops_the_nav_velocity_and_recovers_within_the_budget(system):
    sup, fake, proc = system()
    begin_exploration_with_nav_goal(sup, fake)
    t0 = time.monotonic()
    proc.send_signal(signal.SIGKILL)           # the Nav2 goal is NOT cancelled by the dead server
    proc.wait(timeout=10)
    t0 = time.monotonic()
    assert wait_for(lambda: fake.cancelled, timeout=BUDGET_S + 2.0), 'navigate_to_pose goals were never cancelled'
    assert wait_for(lambda: not fake.active)
    assert wait_for(lambda: time.monotonic() - fake.last_nonzero > 0.3)
    assert fake.last_nonzero - t0 <= BUDGET_S, (
        f'non-zero command {fake.last_nonzero - t0:.2f} s after the kill (budget {BUDGET_S} s)')
    # recovery (003-R14): the failed action continues with the partial map
    assert wait_for(lambda: sup._machine.submode == 'COVERING'), 'supervisor did not go on to COVERING'
    assert sup._machine.exploration_done
    assert sup._explore_handle is None and not sup._explore_watchdog.armed


def test_003_R14_mode_leaving_cleaning_cancels_the_explore_goal_and_the_canceled_result_is_dropped(system):
    sup, fake, proc = system()
    statuses = []
    begin_exploration_with_nav_goal(sup, fake)
    # a second result future on the same goal records the server's answer to the cancel
    handle = sup._explore_handle
    seq = sup._explore_seq
    handle.get_result_async().add_done_callback(lambda fut: statuses.append(fut.result().status))
    fake.pub_mode.publish(String(data='IDLE'))
    assert wait_for(lambda: sup._machine.submode == 'NONE')
    assert wait_for(lambda: GoalStatus.STATUS_CANCELED in statuses), 'the CANCELED result never arrived'
    assert sup._explore_handle is None and not sup._explore_watchdog.armed
    assert not sup._machine.exploration_done        # a cancel by us is not a result (stale sequence)
    assert sup._machine.submode == 'NONE' and sup._explore_seq != seq
