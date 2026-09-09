from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'excavator_control'


setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=[
            'test',
            'tests',
        ]
    ),

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
                'config',
            ),
            glob('config/*.yaml'),
        ),
    ],

    install_requires=[
        'setuptools',
        'PyYAML',
    ],

    tests_require=[
        'pytest',
    ],

    test_suite='test',

    zip_safe=True,

    maintainer='CIC ConRobotics',
    maintainer_email='njh5734@psu.edu',

    description=(
        'ROS 2 control package for the CIC physical model excavator.'
    ),

    license='MIT',

    entry_points={
        'console_scripts': [
            'validate_excavator_config = '
            'excavator_control.validate_config:main',

            'validate_excavator_trajectory = '
            'excavator_control.validate_trajectory:main',

            'excavator_trajectory_server = '
            'excavator_control.excavator_trajectory_server:main',

            'excavator_trajectory_client = '
            'excavator_control.excavator_trajectory_client:main',
        ],
    },
)