from setuptools import find_packages, setup

package_name = 'vacuum_mapping'

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
    description='Mapping: pure-Python cores and map file I/O',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_diff = vacuum_mapping.map_diff_cli:main',
        ],
    },
)
