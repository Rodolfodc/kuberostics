"""Helper process of test_supervisor_explore.py: a fake `explore` server that the test kills mid-goal.

Usage: fake_explore_server.py [--silent] [--finish success|abort]
  --silent   accepts the goal but never publishes feedback
  --finish   ends the goal by itself after 1 s (success, or abort) instead of running until cancelled
"""
import sys
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from vacuum_interfaces.action import Explore


class FakeExplore(Node):
    def __init__(self, silent, finish):
        super().__init__('fake_explore_server')
        self.silent = silent
        self.finish = finish
        ActionServer(self, Explore, '/myRobot/explore', self._execute, callback_group=ReentrantCallbackGroup(),
                     cancel_callback=lambda _: CancelResponse.ACCEPT)

    def _execute(self, goal_handle):
        t0 = time.monotonic()
        while rclpy.ok() and not goal_handle.is_cancel_requested:
            if self.finish and time.monotonic() - t0 > 1.0:
                if self.finish == 'success':
                    goal_handle.succeed()
                    return Explore.Result(success=True)
                goal_handle.abort()
                return Explore.Result(success=False)
            if not self.silent:
                goal_handle.publish_feedback(Explore.Feedback(frontiers_remaining=3))
            time.sleep(0.5)                      # 2 Hz, faster than the 1 Hz of 003-R31
        goal_handle.canceled()
        return Explore.Result()


def main():
    rclpy.init()
    finish = sys.argv[sys.argv.index('--finish') + 1] if '--finish' in sys.argv else ''
    node = FakeExplore('--silent' in sys.argv, finish)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    main()
