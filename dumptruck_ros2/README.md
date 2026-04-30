# ROS2 Dump Truck
This repository contains ROS2 nodes for controlling a small differential-drive dump truck using Raspberry Pi, pigpio, wheel encoders, and waypoint-based navigation using pure-pursuit.

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
First establish the ROS communication between the RaspberryPi and ROS Computer in the IoT network. Then plan the path and place it in the waypoints_with_actions.yaml.
On Raspberry Pi:
First Terminal
```bash
source /opt/ros/jazzy/setup.bash
python3 Motor_Drive_Node3_clean.py
```
Second Terminal
```bash
source /opt/ros/jazzy/setup.bash
python3 bucket_action_node.py
```

On ROS Computer:
First Terminal
```bash
source /opt/ros/jazzy/setup.bash
python3 Odom_test_v3.py
```
Second Terminal
```bash
source /opt/ros/jazzy/setup.bash
python3 ppwyr.py
```



