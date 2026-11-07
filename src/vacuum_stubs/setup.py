from setuptools import find_packages, setup

package_name = 'vacuum_stubs'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Rodolfo Dalla Costa',
    maintainer_email='rodolfo.dalla.costa@hotmail.com',
    description='Skeleton stubs: coverage_server, escape_server, stuck_detector',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'coverage_server = vacuum_stubs.coverage_server_node:main',
        'escape_server = vacuum_stubs.escape_server_node:main',
        'stuck_detector = vacuum_stubs.stuck_detector_node:main',
    ]},
)
