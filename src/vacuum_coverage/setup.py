from setuptools import find_packages, setup

package_name = 'vacuum_coverage'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/coverage.launch.py', 'launch/stuck_escape.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Rodolfo Dalla Costa',
    maintainer_email='rodolfo.dalla.costa@hotmail.com',
    description='Coverage and escape (spec 005): pure-Python cores and the coverage_server, stuck_detector and escape_server nodes',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'coverage_server = vacuum_coverage.coverage_server_node:main',
        'stuck_detector = vacuum_coverage.stuck_detector_node:main',
        'escape_server = vacuum_coverage.escape_server_node:main',
    ]},
)
