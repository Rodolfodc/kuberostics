"""Isolates the in-process rclpy tests from every other ROS graph (Art. X).

Fixed domain IDs plus the default SUBNET discovery let two containers on the same Docker bridge (parallel CI
jobs, several developers) discover each other and run these tests against each other's nodes. Discovery is
restricted to localhost and the domain ID is picked per process inside a range reserved for this package
(stubs 10-39, safety 40-69, supervisor 70-99), before rclpy.init() reads the environment.

A different domain per test as well: a second rclpy.init() in the same process and domain creates a participant
with the same GUID prefix as the one just destroyed, and for a while the new graph is unreliable (a service or
action server of the same context, or of a child process, flaps to "not ready" or is never discovered; measured
at 35 % of contexts on Jazzy and 10 % on Lyrical at --cpus=1, 0 % with a new domain per context).
"""
import os
import random

import pytest

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
_RANGE = (40, 70)
_base = random.SystemRandom().randrange(*_RANGE)
_count = [0]


@pytest.fixture(autouse=True)
def _fresh_domain_per_test():
    """Runs before the test's own fixtures call rclpy.init() (and before any subprocess is spawned)."""
    span = _RANGE[1] - _RANGE[0]
    os.environ['ROS_DOMAIN_ID'] = str(_RANGE[0] + (_base - _RANGE[0] + _count[0]) % span)
    _count[0] += 1
    yield
