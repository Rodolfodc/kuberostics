"""Isolates the in-process rclpy tests from every other ROS graph (Art. X): localhost discovery only and a
domain ID picked per test in 6-9 (rclpy.init() reads the environment, so it is set before each test).

Domain allocation: 0 free, coverage 1-5, sim bridge 6-9, stubs 10-39, safety 40-69, supervisor 70-99,
bringup 100-129, sequencer graph 130-159, detection 160-189, contract tests 190-219, mapping 220-232
(Fast DDS maximum is 232)."""
import os
import random

import pytest

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)


@pytest.fixture(autouse=True)
def _isolated_domain():
    os.environ['ROS_DOMAIN_ID'] = str(random.SystemRandom().randrange(6, 10))
    yield
