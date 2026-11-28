#!/usr/bin/env python3
"""Publishes the orchestrator mode string on /myRobot/behaviour/mode every 0.2 s with the IA368II QoS (local-only, full-robot live run).
Replaces behaviour_server/myrobot_server when the run is launched with with_ia368ii:=false: their battery model drains 1 % per WALL
second (cleaning ends with AUTODOCKING after ~75 s wall, i.e. 14 s of simulated time at 0.19x), which cancels any coverage.
Usage: full_live_mode_pub.py [MODE=CLEANING] [duration_s=7200]"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
mode = sys.argv[1] if len(sys.argv) > 1 else 'CLEANING'; dur = float(sys.argv[2]) if len(sys.argv) > 2 else 7200.0
rclpy.init(); n = Node('full_live_mode_pub')
pub = n.create_publisher(String, '/myRobot/behaviour/mode', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
t0 = time.time()
while rclpy.ok() and time.time() - t0 < dur:
    pub.publish(String(data=mode)); rclpy.spin_once(n, timeout_sec=0.2)
