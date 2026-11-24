def pytest_configure(config):
    config.addinivalue_line(
        'markers', 'live: needs a running graph (ros2 launch vacuum_bringup skeleton.launch.py); '
                   'run with VACUUM_CONTRACT_LIVE=1 (Art. V)')
