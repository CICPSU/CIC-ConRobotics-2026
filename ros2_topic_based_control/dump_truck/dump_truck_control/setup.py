import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'dump_truck_control'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join(
                'share',
                package_name,
                'waypoints',
            ),
            glob('waypoints/*.yaml'),
        ),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='njh5734',
    maintainer_email='njh5734@psu.edu',
    description=(
        'ROS 2 control nodes for the scaled dump truck'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'odometry_node = '
                'dump_truck_control.odometry_node:main'
            ),
            (
                'tag_odom_fusion_node = '
                'dump_truck_control.tag_odom_fusion_node:main'
            ),
            (
                'waypoint_controller_node = '
                'dump_truck_control.waypoint_controller_node:main'
            ),
        ],
    },
)