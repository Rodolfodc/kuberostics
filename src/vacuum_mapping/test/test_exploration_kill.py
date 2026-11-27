"""Kill test of exploration (003-R31, T324 of spec 003), PARTIAL: see below.

003-R31 / 005-R34 want: kill `exploration_server` mid-goal and no non-zero command on mux/nav within
feedback_stale_s + nav_cancel_s + 0.5 s = 4.0 s. The real supervisor watchdog (T512, plus the supervisor's
`explore` client, T315) is not in this package or in main_local yet, so a test-local STAND-IN implements the
005-R34 rule: no `explore` feedback for 2.5 s -> cancel all `/navigate_to_pose` goals (zero goal id and
stamp) and expect the fake controller to go quiet within 1.0 s.

What this proves: the REAL node process (spawned here, not through launch_testing, which treats the death of
a launched process as a failure) publishes periodic feedback (node-clock gaps at most 2x the 1 Hz requirement, so a loaded host cannot fail it); SIGKILL leaves an orphan Nav2
goal whose fake controller keeps publishing (the hazard is real); and the cancel-all path silences mux/nav
inside 4.0 s. What it does NOT prove: the supervisor's own cancel path. Replace the stand-in by the real
supervisor when T315/T512 land. No simulator, no Nav2; the fake velocity publisher plays controller_server.
"""
import os
import signal
import subprocess
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from action_msgs.msg import GoalInfo  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from ament_index_python.packages import get_package_prefix  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from lifecycle_msgs.msg import State, Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState, GetState  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from vacuum_interfaces.action import Explore  # noqa: E402

from exploration_fixtures import HANG, FakeNav, MapPublisher, one_frontier_cells, wait_for  # noqa: E402

FEEDBACK_STALE_S = 2.5          # 005-R34 defaults
NAV_CANCEL_S = 1.0
BUDGET_S = FEEDBACK_STALE_S + NAV_CANCEL_S + 0.5
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def _node_executable():
    return os.path.join(get_package_prefix('vacuum_mapping'), 'lib', 'vacuum_mapping', 'exploration_server')


def test_R31_kill_mid_goal_stand_in_watchdog_silences_mux_nav_within_budget():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=domain)
    nav = FakeNav(lambda i, x, y: HANG, context=context)
    maps = MapPublisher(one_frontier_cells(), context=context)
    probe = rclpy.create_node('exploration_kill_probe', context=context)
    twists, feedback, node_elapsed = [], [], []    # twists: (monotonic, non-zero?); node_elapsed: Feedback.elapsed_s
    probe.create_subscription(Twist, '/myRobot/mux/nav',
                              lambda m: twists.append((time.monotonic(), m.linear.x != 0.0)), QOS)
    velocity_pub = probe.create_publisher(Twist, '/myRobot/mux/nav', QOS)

    def fake_controller():                         # fresh non-zero velocity while a Nav2 goal is active
        msg = Twist()
        msg.linear.x = 0.2 if nav.active else 0.0
        velocity_pub.publish(msg)
    probe.create_timer(0.1, fake_controller)
    change_state = probe.create_client(ChangeState, '/exploration_server/change_state')
    get_state = probe.create_client(GetState, '/exploration_server/get_state')
    cancel_all = probe.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
    client = ActionClient(probe, Explore, '/myRobot/explore')
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for n in (nav, maps, probe):
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    proc = subprocess.Popen([_node_executable(), '--ros-args', '-r', '__node:=exploration_server'], env=env)
    try:
        def state():
            future = get_state.call_async(GetState.Request())
            return future.result().current_state.id if wait_for(future.done, 3.0) else None
        # A response proves discovery both ways; a lost first response of change_state is retried and the
        # outcome is checked through get_state (Fast DDS can drop the first response of a fresh client).
        assert wait_for(lambda: state() is not None, 120.0), 'node never appeared'
        for transition, target in ((Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE),
                                   (Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE)):
            def reached(transition=transition, target=target):
                future = change_state.call_async(ChangeState.Request(transition=Transition(id=transition)))
                wait_for(future.done, 5.0)
                return state() == target
            assert wait_for(reached, 60.0), f'transition {transition} did not reach state {target}'
        assert client.wait_for_server(timeout_sec=30.0)

        accepted = []

        def try_goal():                            # the node needs the latched map and the first TF
            future = client.send_goal_async(Explore.Goal(timeout_s=0.0),
                                            feedback_callback=lambda m: (feedback.append(time.monotonic()),
                                                                         node_elapsed.append(m.feedback.elapsed_s)))
            if wait_for(future.done, 10.0) and future.result().accepted:
                accepted.append(future.result())
            return bool(accepted)
        assert wait_for(try_goal, 60.0), 'goal never accepted'
        assert wait_for(lambda: nav.active, 30.0), 'the Nav2 goal never started'
        assert wait_for(lambda: len(feedback) >= 4, 10.0)
        # Measured on the node's own clock (Feedback.elapsed_s), not on arrival in this loaded test process.
        # The requirement is 1 Hz or faster; the bound is 2x that, which still proves the feedback is periodic
        # and keeps the 2.5 s staleness rule of the watchdog meaningful, without depending on CPU luck.
        gaps = [b - a for a, b in zip(node_elapsed, node_elapsed[1:])]
        assert max(gaps) <= 2.0, f'feedback must be periodic, 1 Hz or faster (003-R31), node-clock gaps: {gaps}'

        t_kill = time.monotonic()
        proc.send_signal(signal.SIGKILL)
        proc.wait(timeout=5)
        assert nav.active, 'the Nav2 goal is orphaned by the kill: the hazard is real'

        # stand-in for the supervisor watchdog (005-R34): no feedback for feedback_stale_s -> cancel all goals
        assert wait_for(lambda: time.monotonic() - feedback[-1] > FEEDBACK_STALE_S, FEEDBACK_STALE_S + 2.0)
        assert cancel_all.wait_for_service(timeout_sec=5.0)
        future = cancel_all.call_async(CancelGoal.Request(goal_info=GoalInfo()))   # zero id and stamp: all
        assert wait_for(future.done, NAV_CANCEL_S)
        assert wait_for(lambda: not nav.active, NAV_CANCEL_S)

        deadline = t_kill + BUDGET_S
        wait_for(lambda: time.monotonic() > deadline + 0.3, BUDGET_S + 1.0)
        late = [t for t, nonzero in twists if nonzero and t > deadline]
        assert not late, f'{len(late)} non-zero commands on mux/nav after kill + {BUDGET_S} s'
        assert [t for t, nonzero in twists if nonzero and t > t_kill], \
            'the fake controller published nothing after the kill: the test proves nothing'
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=5)
        nav.stopped = True
        executor.shutdown()
        for n in (nav, maps, probe):
            n.destroy_node()
        context.try_shutdown()
        thread.join(timeout=2)
