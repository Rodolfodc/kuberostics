"""009a-R01: live graph of detection.launch.py with with_manager:=false autostart:=true (no manager, no weights).

Every node of the chain, projector included, must reach `active` driven only by the launch events. Domain:
launch_support.DOMAIN (175-189 of the detection range).
"""
import os
import sys
import unittest

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import launch_support as ls  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402

NODES = ['detector_local', 'detection_selector', 'image_throttle', 'projector']


@pytest.mark.launch_test
def generate_test_description():
    return ls.launch_description(offload=True, extra={
        'with_manager': 'false', 'autostart': 'true', 'with_projector': 'true'})


class TestAutostart(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.probe = ls.Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def _label(self, name):
        client = self.probe.node.create_client(GetState, f'/{name}/get_state')
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(GetState.Request())
        if not self.probe.wait_for(future.done, 5.0):
            return None
        return future.result().current_state.label

    def test_009a_R01_every_node_reaches_active_with_no_manager(self):
        ok = self.probe.wait_for(lambda: all(self._label(n) == 'active' for n in NODES), ls.STARTUP_TIMEOUT_S)
        self.assertTrue(ok, {n: self._label(n) for n in NODES})
