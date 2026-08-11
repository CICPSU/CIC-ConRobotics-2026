# ROS2 Dump Truck
This repository contains ROS2 nodes for controlling a small differential-drive dump truck using Raspberry Pi, pigpio, wheel encoders, and waypoint-based navigation using pure-pursuit.
This directory contains the verified standalone implementation. Scripts are executed directly with Python and are not installed as ROS 2 packages. The packaged implementation is being developed under: `ros2_topic_based_control/dump_truck/`

## System Architecture
Raspberry Pi:
- Motor_Drive_Node3_clean.py
- bucket_action_node.py

ROS Computer:
- Odom_test_v3.py
- ppwyr.py (update the waypoints.yaml location)
- waypoints.yaml


## ROS Topics
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| /cmd_vel | geometry_msgs/Twist | waypoint_controller | motor_drive_node |
| /odom | nav_msgs/Odometry | odom_node | waypoint_controller |

## Launch
First establish the ROS communication between the RaspberryPi and ROS Computer in the IoT network. Then plan the path and place it in the yaml files.

Run codes with the commands below. Showing example for dump truck 1. Note that every terminal needs to establish the ROS communication.
Nodes.py needs to be updated based on the trucks you want to run.

On Raspberry Pi:
First Terminal
```bash
sudo pigpiod
```
```bash
python3 Nodes.py
```

On ROS Computer:

Run the camera.
```bash
python3 Camera.py
```


Update the file location as needed.
```bash
python3 truck1_run.py -p waypoints_yaml:=/home/xxxxxxx/AprilTag/truck1_waypoints.yaml
```



