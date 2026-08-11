from setuptools import find_packages, setup

package_name = 'dump_truck_hardware'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='njh5734',
    maintainer_email='njh5734@psu.edu',
    description='ROS 2 hardware interface nodes for the scaled dump truck',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_drive_node = dump_truck_hardware.motor_drive_node:main',
            'bucket_action_node = dump_truck_hardware.bucket_action_node:main',
        ],
    },
)