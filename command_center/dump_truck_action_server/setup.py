from setuptools import (
    find_packages,
    setup,
)


package_name = (
    'dump_truck_action_server'
)


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=[
            'test',
        ]
    ),

    data_files=[
        (
            'share/ament_index/'
            'resource_index/packages',
            [
                'resource/'
                + package_name
            ],
        ),
        (
            'share/'
            + package_name,
            [
                'package.xml',
            ],
        ),
    ],

    install_requires=[
        'setuptools',
        'PyYAML',
    ],

    zip_safe=True,

    maintainer='njh5734',
    maintainer_email='njh5734@psu.edu',

    description=(
        'ROS 2 Action Servers for '
        'dump truck waypoint tasks.'
    ),

    license='TODO',

    tests_require=[
        'pytest',
    ],

    entry_points={
        'console_scripts': [
            (
                'waypoint_action_server_node = '
                'dump_truck_action_server.'
                'waypoint_action_server_node:main'
            ),
            (
                'mock_waypoint_action_server_node = '
                'dump_truck_action_server.'
                'mock_waypoint_action_server_node:main'
            ),
        ],
    },
)