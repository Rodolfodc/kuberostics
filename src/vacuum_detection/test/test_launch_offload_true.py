"""009a-R01: live graph of detection.launch.py with offload:=true and backend:=null (no weights, no simulator)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import launch_support as ls  # noqa: E402
from live_checks import LiveChecks  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return ls.launch_description(offload=True)


class TestOffloadTrue(LiveChecks):
    OFFLOAD = True
