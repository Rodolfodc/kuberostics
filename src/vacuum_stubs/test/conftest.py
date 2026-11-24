"""Isolates the in-process rclpy tests from every other ROS graph (T020, Art. X).

Fixed domain IDs plus the default SUBNET discovery let two containers on the same Docker bridge (parallel CI
jobs, several developers) discover each other and run these tests against each other's nodes. Discovery is
restricted to localhost and the domain ID is picked per process inside a range reserved for this package
(stubs 10-39, safety 40-69, supervisor 70-99), before rclpy.init() reads the environment.
"""
import os
import random

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ['ROS_DOMAIN_ID'] = str(random.SystemRandom().randrange(10, 40))
