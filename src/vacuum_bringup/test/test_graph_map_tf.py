"""Live-graph test: skeleton with map_tf:=true resolves the TF chain map -> my_robot_odom -> base_footprint
(spec 003-R05, contract `tf`). No simulator: the test process plays the sim bridge by publishing a fake
my_robot_odom -> base_footprint, and a TF listener must resolve the whole chain.
"""
import os
import sys
import unittest

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graph_support as gs  # noqa: E402

MODULE = 23          # domain from the bringup range (graph_support, 100-129); modules run one at a time


@pytest.mark.launch_test
def generate_test_description():
    return gs.launch_skeleton(MODULE, extra_args={'map_tf': 'true'})


class TestMapTfChain(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.probe = gs.Probe(MODULE)
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.probe.node, spin_thread=False)
        cls.broadcaster = TransformBroadcaster(cls.probe.node)
        cls.timer = cls.probe.node.create_timer(0.05, cls._publish_fake_odom)
        cls.x = 1.5

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    @classmethod
    def _publish_fake_odom(cls):
        msg = TransformStamped()
        msg.header.stamp = cls.probe.node.get_clock().now().to_msg()
        msg.header.frame_id = 'my_robot_odom'
        msg.child_frame_id = 'base_footprint'
        msg.transform.translation.x = cls.x
        msg.transform.rotation.w = 1.0
        cls.broadcaster.sendTransform(msg)

    def test_01_chain_resolves(self):
        """003-R05: map -> base_footprint resolves through the static map -> my_robot_odom."""
        def resolved():
            return self.buffer.can_transform('map', 'base_footprint', Time(), Duration(seconds=0.0))
        self.assertTrue(self.probe.wait_for(resolved, gs.STARTUP_TIMEOUT_S, 0.1),
                        'map -> base_footprint never resolved')
        t = self.buffer.lookup_transform('map', 'base_footprint', Time())
        self.assertAlmostEqual(t.transform.translation.x, self.x, places=6)    # identity map -> odom
        self.assertAlmostEqual(t.transform.translation.y, 0.0, places=6)

    def test_02_map_to_odom_is_identity(self):
        """The static fallback is identity (dead reckoning only)."""
        self.assertTrue(self.probe.wait_for(
            lambda: self.buffer.can_transform('map', 'my_robot_odom', Time(), Duration(seconds=0.0)), 30.0, 0.1))
        t = self.buffer.lookup_transform('map', 'my_robot_odom', Time()).transform
        self.assertEqual((t.translation.x, t.translation.y, t.translation.z), (0.0, 0.0, 0.0))
        self.assertEqual((t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w), (0.0, 0.0, 0.0, 1.0))
