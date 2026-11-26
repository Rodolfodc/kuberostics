import os

from vacuum_contract_tests.graph import isolate_environment


def pytest_configure(config):
    config.addinivalue_line(
        'markers', 'live: needs a running graph (ros2 launch vacuum_bringup skeleton.launch.py); '
                   'run with VACUUM_CONTRACT_LIVE=1 (Art. V)')


# The graph tests that bring up their own graph (test_live_launch.py) are isolated from every other ROS graph.
# With VACUUM_CONTRACT_LIVE=1 the user points the tests at a graph of their own: the environment is left alone.
if os.environ.get('VACUUM_CONTRACT_LIVE') != '1':
    isolate_environment()
