"""Isolates the in-process rclpy tests from every other ROS graph (Art. X).

Discovery is restricted to localhost; the domain ID is inside the range reserved for this package (mapping
200-229: Fast DDS rejects domain IDs above 232, so 250-279 is unusable) and changes for every test: a second rclpy.init() in the same process and domain
reuses the GUID prefix of the participant just destroyed and the new graph flaps (same fix as the stubs,
safety and supervisor packages).
"""
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))      # grid_fixtures.py is imported by name

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
_RANGE = (200, 230)
_base = random.SystemRandom().randrange(*_RANGE)
_count = [0]


@pytest.fixture(autouse=True)
def _fresh_domain_per_test():
    """Runs before the test's own fixtures call rclpy.init()."""
    span = _RANGE[1] - _RANGE[0]
    os.environ['ROS_DOMAIN_ID'] = str(_RANGE[0] + (_base - _RANGE[0] + _count[0]) % span)
    _count[0] += 1
    yield
