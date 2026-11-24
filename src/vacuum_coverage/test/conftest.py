"""Isolates the in-process rclpy tests of vacuum_coverage from every other ROS graph (Art. X).

Discovery is restricted to localhost and the domain ID is picked per process before rclpy.init() reads the
environment. Range reserved for this package: 1-5, disjoint from every other allocation:
sim bridge 6-9, stubs 10-39, safety 40-69, supervisor 70-99, bringup 100-129, sequencer graph 130-159, detection 160-189,
contract tests 190-219, mapping 220-232 (Fast DDS maximum is 232). Domain 0 is the ROS default and is kept free.
The launch_testing test of this package uses the same range through ROS_DOMAIN_ID set here.
"""
import os
import random

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ['ROS_DOMAIN_ID'] = str(random.SystemRandom().randrange(1, 6))
