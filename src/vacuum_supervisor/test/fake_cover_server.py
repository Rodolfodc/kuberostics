"""Helper process of test_supervisor_watchdog.py: a fake `cover_room` server that the test kills mid-goal.

Usage: fake_cover_server.py [--silent]   (--silent: accepts the goal but never publishes feedback)
"""
import sys
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from vacuum_interfaces.action import CoverRoom


class FakeCover(Node):
    def __init__(self, silent):
        super().__init__('fake_cover_server')
        self.silent = silent
        ActionServer(self, CoverRoom, '/myRobot/cover_room', self._execute, callback_group=ReentrantCallbackGroup(),
                     cancel_callback=lambda _: CancelResponse.ACCEPT)

    def _execute(self, goal_handle):
        while rclpy.ok() and not goal_handle.is_cancel_requested:
            if not self.silent:
                goal_handle.publish_feedback(CoverRoom.Feedback(covered_ratio=0.1))
            time.sleep(0.5)                      # 2 Hz, faster than the 1 Hz of 005-R34
        goal_handle.canceled()
        return CoverRoom.Result()


def main():
    rclpy.init()
    node = FakeCover('--silent' in sys.argv)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    main()
