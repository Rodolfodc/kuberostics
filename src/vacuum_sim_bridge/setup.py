from glob import glob

from setuptools import find_packages, setup

package_name = 'vacuum_sim_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Rodolfo Dalla Costa',
    maintainer_email='rodolfo.dalla.costa@hotmail.com',
    description='CoppeliaSim bridge nodes for the combined myRobotLaser model (same-name replacements of the '
                'IA368II 2025S2 bridges)',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'my_robot_scan_node = vacuum_sim_bridge.scan_node:main',
        'my_robot_vision_node = vacuum_sim_bridge.vision_node:main',
        'my_robot_cmd_vel_node = vacuum_sim_bridge.cmd_vel_node:main',
        'my_robot_odometry_node = vacuum_sim_bridge.odometry_node:main',
    ]},
)
