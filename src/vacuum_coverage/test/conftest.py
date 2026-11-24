"""Isolates the in-process rclpy tests of vacuum_coverage from every other ROS graph (Art. X).

Discovery is restricted to localhost and the domain ID is picked per process before rclpy.init() reads the
environment. Range reserved for this package: 215-232 (stubs 10-39, safety 40-69, supervisor 70-99, bringup
100-159, detection 160-189). It sits in the band where the default DDS port mapping stays clear of the Linux
ephemeral ports (domain IDs 0-101 and 215-232, per the ROS 2 domain ID documentation), and above the ranges
already taken. The launch_testing test of this package uses the same range (see test_coverage_launch.py).
"""
import os
import random

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ['ROS_DOMAIN_ID'] = str(random.SystemRandom().randrange(215, 233))
