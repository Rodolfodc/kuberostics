import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vacuum_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Rodolfo Dalla Costa',
    maintainer_email='rodolfo.dalla.costa@hotmail.com',
    description='Detection offload: selector and projector cores',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'detection_selector = vacuum_detection.detection_selector_node:main',
        'image_throttle = vacuum_detection.image_throttle_node:main',
        'detector = vacuum_detection.detector_node:main',
    ]},
)
