from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'construction_site_control'


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
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name,
            ],
        ),

        (
            'share/' + package_name,
            [
                'package.xml',
            ],
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch',
            ),
            glob(
                'launch/*.launch.py'
            ),
        ),

        (
            os.path.join(
                'share',
                package_name,
                'scenarios',
            ),
            glob(
                'scenarios/*.yaml'
            ),
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
        'ROS 2 action-based command center '
        'for construction robot coordination.'
    ),

    license='TODO',

    tests_require=[
        'pytest',
    ],

    entry_points={
        'console_scripts': [
            (
                'scenario_manager_node = '
                'construction_site_control.'
                'scenario_manager_node:main'
            ),
        ],
    },
)