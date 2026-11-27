"""Support of the live-graph launch tests of detection.launch.py (not a test module).

`launch_description(offload)` is the real detection.launch.py with backend:=null (no weights, no GPU) on a private
ROS domain in 175-189 (of the 160-189 range of this package); `check_final_topic` applies the contract helpers of
vacuum_contract_tests to what the graph really has on /myRobot/perception/detections.
"""
import os
import random
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from vacuum_contract_tests.contract import (
    build_expectations, check_topic, Endpoint, find_contract, load_contract, QosSpec)

FINAL_TOPIC = '/myRobot/perception/detections'
LOCAL_TOPIC = '/myRobot/perception/detections/local'
STARTUP_TIMEOUT_S = 60.0
DOMAIN = random.SystemRandom().randrange(175, 190)      # 160-174 is for the per-test domains of conftest.py


def isolate_environment() -> None:
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['ROS_DOMAIN_ID'] = str(DOMAIN)


def launch_description(offload: bool) -> LaunchDescription:
    source = os.path.join(get_package_share_directory('vacuum_detection'), 'launch', 'detection.launch.py')
    return LaunchDescription([
        # actions, not os.environ: launch_testing restores the environment after generate_test_description
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(DOMAIN)),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(source), launch_arguments={
            'offload': 'true' if offload else 'false', 'backend': 'null'}.items()),
        ReadyToTest(),
    ])


def _qos(info, expected):
    """Graph QoS of one endpoint; Jazzy reports depth 0 for endpoints it does not own, which takes the contract's."""
    q = info.qos_profile
    depth = int(q.depth)
    if depth == 0 and expected is not None:
        depth = expected.depth
    return QosSpec(q.reliability.name.lower(), q.durability.name.lower(), depth)


class Probe:
    def __init__(self):
        isolate_environment()
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=DOMAIN)
        self.node = rclpy.create_node('detection_launch_probe', context=self.context)
        self._lock = threading.Lock()
        self.received = []
        self.node.create_subscription(Detection2DArray, FINAL_TOPIC, self._on_final, 10)
        self._camera = self.node.create_publisher(Image, '/camera', qos_profile_sensor_data)
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        contract = load_contract(find_contract())
        # build_expectations() only covers `since: "001"` entries (001-R04 scope). The two 009a topics checked here
        # are built the same way, by dropping their `since`: the contract helpers stay the single rule.
        contract['topics'] = [{k: v for k, v in e.items() if k != 'since'} for e in contract['topics']
                              if e['name'] in (FINAL_TOPIC, LOCAL_TOPIC)]
        self.expectations = {e.name: e for e in build_expectations(contract) if e.kind == 'topic'}

    def close(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=5.0)
        self.node.destroy_node()
        self.context.try_shutdown()

    def _on_final(self, msg) -> None:
        with self._lock:
            self.received.append(msg)

    def wait_for(self, predicate, timeout_s, period_s=0.05) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(period_s)
        return predicate()

    def manager_active(self) -> bool:
        client = self.node.create_client(Trigger, '/lifecycle_manager_optional/is_active')
        if not client.wait_for_service(timeout_sec=1.0):
            return False
        future = client.call_async(Trigger.Request())
        return self.wait_for(future.done, 5.0) and bool(future.result().success)

    def endpoints(self, topic):
        exp = self.expectations[topic]
        pubs = [Endpoint(i.node_name, i.topic_type, _qos(i, exp.qos))
                for i in self.node.get_publishers_info_by_topic(topic)]
        subs = [Endpoint(i.node_name, i.topic_type, _qos(i, exp.qos))
                for i in self.node.get_subscriptions_info_by_topic(topic)]
        return exp, pubs, subs

    def publisher_nodes(self, topic):
        return [i.node_name for i in self.node.get_publishers_info_by_topic(topic)]

    def contract_problems(self, topic):
        """check_topic of the contract helpers on the live endpoints; the probe's own subscription is ignored."""
        exp, pubs, subs = self.endpoints(topic)
        return check_topic(exp, pubs, [s for s in subs if s.node != 'detection_launch_probe'])

    def stream_camera_until(self, predicate, timeout_s) -> bool:
        """Publishes /camera frames stamped with the robot clock at 5 Hz until `predicate` holds."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            msg = Image(height=4, width=4, encoding='rgb8', step=12, data=bytes(48))
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self._camera.publish(msg)
            if self.wait_for(predicate, 0.2):
                return True
        return predicate()

    def final_count(self) -> int:
        with self._lock:
            return len(self.received)
