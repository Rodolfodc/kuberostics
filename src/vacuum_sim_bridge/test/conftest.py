"""Isolates the in-process rclpy tests from every other ROS graph (Art. X): localhost discovery only and a
domain ID picked per process in 1-9 (stubs 10-39, safety 40-69, supervisor 70-99), before rclpy.init()."""
import os
import random

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ['ROS_DOMAIN_ID'] = str(random.SystemRandom().randrange(1, 10))
