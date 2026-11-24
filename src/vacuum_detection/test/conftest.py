"""Isolates the in-process rclpy tests from every other ROS graph (Art. X).

Discovery is restricted to localhost, and every test gets its own domain ID inside the range reserved for this
package (detection 160-189, of which 160-174 here; stubs 10-39, safety 40-69, supervisor 70-99, bringup 100-129), set before the test's
fixtures call rclpy.init(). A second rclpy.init() in the same process and domain creates a participant with the
GUID prefix of the one just destroyed, and for a while the new graph is unreliable (measured at 35 % of contexts on
Jazzy and 10 % on Lyrical at --cpus=1, 0 % with a new domain per context).

The launch tests (test_launch_offload_*.py) pick their own domain in launch_support.py, once per module.
"""
import os
import random

import pytest

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
_RANGE = (160, 175)             # 175-189 belong to the launch tests (launch_support.py)
_base = random.SystemRandom().randrange(*_RANGE)
_count = [0]


@pytest.fixture(autouse=True)
def _fresh_domain_per_test():
    span = _RANGE[1] - _RANGE[0]
    os.environ['ROS_DOMAIN_ID'] = str(_RANGE[0] + (_base - _RANGE[0] + _count[0]) % span)
    _count[0] += 1
    yield
