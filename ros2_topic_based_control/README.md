# ROS 2 Topic-Based Robot Control

This directory contains the ROS 2 topic-based control implementation for individual construction robots.

The purpose of this layer is to provide direct control, localization, hardware interfaces, and manual waypoint execution for each robot independently.

Higher-level multi-robot coordination and scenario execution will be implemented separately under the `command_center/` directory.

---

## 1. Directory Structure

```text
ros2_topic_based_control/
├── construction_robot_perception/
│   ├── config/
│   ├── launch/
│   └── ...
│
└── dump_truck/
    ├── dump_truck_hardware/
    │   ├── dump_truck_hardware/
    │   │   ├── motor_drive_node.py
    │   │   └── bucket_action_node.py
    │   └── ...
    │
    ├── dump_truck_control/
    │   ├── dump_truck_control/
    │   │   ├── odometry_node.py
    │   │   ├── tag_odom_fusion_node.py
    │   │   └── waypoint_controller_node.py
    │   │
    │   ├── waypoints/
    │   │   ├── truck1_waypoints.yaml
    │   │   ├── truck1_waypoints2.yaml
    │   │   └── truck3_waypoints.yaml
    │   └── ...
    │
    └── dump_truck_bringup/
        ├── config/
        │   ├── hardware/
        │   │   ├── truck1.yaml
        │   │   └── truck3.yaml
        │   │
        │   └── localization/
        │       └── landmarks.yaml
        │
        ├── launch/
        │   ├── dump_truck_pi.launch.py
        │   └── dump_truck_ros_pc.launch.py
        └── ...
```

---

## 2. ROS 2 Network Setup

Network configuration is handled by:

```text
network/setup_network.sh
network/devices.sh
```

The network setup script configures:

* `ROS_DOMAIN_ID`
* `ROS_AUTOMATIC_DISCOVERY_RANGE`
* `ROS_STATIC_PEERS`

### FROM ROS PC

For Truck 1 only:

```bash
source network/setup_network.sh ros_pc dumptruck_01
```

For Truck 3 only:

```bash
source network/setup_network.sh ros_pc dumptruck_03
```

For simultaneous Truck 1 and Truck 3 operation:

```bash
source network/setup_network.sh ros_pc dumptruck_01 dumptruck_03
```

### Dump Truck Raspberry Pi

On Truck 1:

```bash
source network/setup_network.sh ros_pc dumptruck_01
```

Each new terminal requires the network setup script to be sourced again.

---

## 3. Initial Setup and Build

### If the Repository Has Not Been Cloned Yet

Create the workspace directory:

```bash
mkdir -p ~/ws_conrobotics
cd ~/ws_conrobotics
```

Clone the repository:

```bash
git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git
```

Enter the repository:

```bash
cd CIC-ConRobotics-2026
```

Switch to the development branch:

```bash
git checkout dev
```

Verify the current branch:

```bash
git branch --show-current
```

Expected output:

```text
dev
```

---

### If the Repository Already Exists

From the repository root:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
```

Make sure the repository is on the `dev` branch:

```bash
git checkout dev
```

Pull the latest changes:

```bash
git pull origin dev
```

---

### Build on the ROS PC

From the repository root:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    construction_robot_perception \
    dump_truck_control \
    dump_truck_hardware \
    dump_truck_bringup
```

After building:

```bash
source install/setup.bash
```

---

### Build on a Raspberry Pi

On a Raspberry Pi, the dump truck system requires the following ROS 2 packages:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

git checkout dev
git pull origin dev

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    dump_truck_control \
    dump_truck_hardware \
    dump_truck_bringup
```

After building:

```bash
source install/setup.bash
```

For a newly configured Raspberry Pi, `git` and `colcon` must also be installed before cloning and building the repository.

---

## 4. Truck-Specific Configuration

Truck-specific parameters are stored in:

```text
dump_truck_bringup/config/hardware/
```

Examples:

```text
truck1.yaml
truck3.yaml
truck4.yaml
...
```

These files contain parameters such as:

* bucket servo calibration
* robot AprilTag frame
* tag yaw offset
* odometry scale factors

Example structure:

```yaml
bucket_action_node:
  ros__parameters:
    servo_center: 900
    servo_dump: 1300

tag_odom_fusion_landmarks_node:
  ros__parameters:
    robot_tag_child_frame: tag36h11_0

    tag_yaw_offset: 1.57079632679

    odom_x_scale: 1.0
    odom_y_scale: 1.0
    odom_yaw_scale: 1.0
```

For Truck 3, the robot tag currently used is:

```text
tag36h11_2
```

---

## 5. Shared AprilTag Landmarks

Fixed overhead-camera calibration landmarks are stored in:

```text
dump_truck_bringup/config/localization/landmarks.yaml
```

Current fixed landmarks:

```text
tag36h11_16
tag36h11_17
tag36h11_18
```

These landmarks are shared by all dump trucks.

---

## 6. Raspberry Pi Bringup

Before starting the ROS nodes, start `pigpiod`:

```bash
sudo pigpiod
```

### Truck 1

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_01

ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck1
```

Expected topics include:

```text
/truck1/cmd_vel
/truck1/wheel_states
/truck1/bucket_action_cmd
/truck1/bucket_action_status
```

Expected nodes include:

```text
/truck1/motor_drive
/truck1/bucket_action
```

### Truck 3

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_03

ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck3
```

Expected topics include:

```text
/truck3/cmd_vel
/truck3/wheel_states
/truck3/bucket_action_cmd
/truck3/bucket_action_status
```

Expected nodes include:

```text
/truck3/motor_drive
/truck3/bucket_action
```

---

## 7. ROS PC Bringup

The same launch file is used for all dump trucks.

### Truck 1

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_01

ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck1
```

### Truck 3

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_03

ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck3
```

The launch starts:

```text
/<truck_name>/odometry
/<truck_name>/tag_odom_fusion
```

and creates:

```text
/<truck_name>/odom
/<truck_name>/fused_odom
```

---

## 8. Overhead Camera and AprilTag Detection

The overhead camera and AprilTag detector must be running before AprilTag-based fusion can initialize.

The camera is provided by the `construction_robot_perception` package.

```bash
ros2 launch \
  construction_robot_perception \
  overhead_camera.launch.py
```

The AprilTag detector must publish TF frames including:

```text
tag36h11_0
tag36h11_2
tag36h11_16
tag36h11_17
tag36h11_18
```

where:

```text
tag36h11_0 = Truck 1
tag36h11_2 = Truck 3

tag36h11_16
tag36h11_17
tag36h11_18 = fixed landmarks
```

---

## 9. Manual Waypoint Execution

The waypoint controller remains available as a topic-based individual robot control method.

### Truck 1

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_01

ros2 run dump_truck_control waypoint_controller_node \
  truck1 \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck1_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck1
```

### Truck 3

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_03

ros2 run dump_truck_control waypoint_controller_node \
  truck3 \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck3_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck3
```

---

## 10. Simultaneous Truck 1 and Truck 3 Operation

### ROS PC Network Setup

```bash
source network/setup_network.sh ros_pc dumptruck_01 dumptruck_03
```

### Truck 1 Pi

```bash
source network/setup_network.sh ros_pc dumptruck_01 dumptruck_03

ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck1
```

### Truck 3 Pi

```bash
source network/setup_network.sh ros_pc dumptruck_01 dumptruck_03

ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck3
```

### ROS PC: Truck 1

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_01 dumptruck_03
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck1
```

### ROS PC: Truck 3

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_01 dumptruck_03
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck3
```

### Truck 1 Waypoint Controller

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_01 dumptruck_03
ros2 run dump_truck_control waypoint_controller_node \
  truck1 \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck1_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck1
```

### Truck 3 Waypoint Controller

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_network.sh ros_pc dumptruck_01 dumptruck_03
ros2 run dump_truck_control waypoint_controller_node \
  truck3 \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck3_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck3
```

Expected node separation:

```text
/truck1/bucket_action
/truck1/motor_drive
/truck1/odometry
/truck1/tag_odom_fusion
/truck1/waypoint_controller_node

/truck3/bucket_action
/truck3/motor_drive
/truck3/odometry
/truck3/tag_odom_fusion
/truck3/waypoint_controller_node
```

Verify with:

```bash
ros2 node list | sort
```

Topics can be checked with:

```bash
ros2 topic list | grep -E 'truck1|truck3' | sort
```

---

## 11. Adding a New Dump Truck

For a new truck, for example Truck 4:

### 1. Add the device to the network configuration

Update:

```text
network/devices.sh
```

and ensure `dumptruck_04` can be resolved by:

```bash
source network/setup_network.sh dumptruck_04
```

### 2. Create the truck configuration

Copy an existing hardware configuration:

```bash
cp \
  ros2_topic_based_control/dump_truck/dump_truck_bringup/config/hardware/truck3.yaml \
  ros2_topic_based_control/dump_truck/dump_truck_bringup/config/hardware/truck4.yaml
```

Update:

* servo calibration
* `robot_tag_child_frame`
* tag yaw offset if required
* odometry scale factors if required

### 3. Create a waypoint file

For example:

```text
dump_truck_control/waypoints/truck4_waypoints.yaml
```

### 4. Commit and Push the New Truck Configuration

After adding the new truck configuration and waypoint files on the ROS PC, verify the changes:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

git status
```

Add the new truck-specific files:

```bash
git add \
  ros2_topic_based_control/dump_truck/dump_truck_bringup/config/hardware/truck4.yaml \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck4_waypoints.yaml
```

If `network/devices.sh` was also updated for the new Raspberry Pi:

```bash
git add network/devices.sh
```

Commit the changes:

```bash
git commit -m "Add Truck 4 configuration and waypoint route"
```

Push to the development branch:

```bash
git push origin dev
```

---

### 5. Prepare the New Truck Raspberry Pi

On the new Truck 4 Raspberry Pi, clone the repository if this is the first setup:

```bash
mkdir -p ~/ws_conrobotics
cd ~/ws_conrobotics

git clone <REPOSITORY_URL>
cd CIC-ConRobotics-2026

git checkout dev
```

If the repository is already installed on the Pi:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

git pull origin dev
```

Confirm that the Truck 4 configuration exists:

```bash
ls \
  ros2_topic_based_control/dump_truck/dump_truck_bringup/config/hardware
```

The output should include:

```text
truck4.yaml
```

---

### 6. Build the ROS 2 Packages on the New Truck Pi

For a new Raspberry Pi, first ensure the ROS 2 workspace dependencies and `colcon` are installed.

Build the dump-truck packages:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    dump_truck_control \
    dump_truck_hardware \
    dump_truck_bringup
```

After a successful build:

```bash
source install/setup.bash
```

If the source directory structure has recently been renamed or moved, perform a clean build:

```bash
rm -rf build install log

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    dump_truck_control \
    dump_truck_hardware \
    dump_truck_bringup

source install/setup.bash
```

---

### 7. Configure the ROS 2 Network on the New Truck Pi

From the repository root:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_network.sh ros_pc
```

Start the GPIO daemon:

```bash
sudo pigpiod
```

---

### 8. Start the Pi

Start the common dump-truck Pi launch file using the new robot name:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck4
```

Expected Truck 4 topics include:

```text
/truck4/cmd_vel
/truck4/wheel_states
/truck4/bucket_action_cmd
/truck4/bucket_action_status
```

Expected Truck 4 nodes include:

```text
/truck4/motor_drive
/truck4/bucket_action
```

---

### 9. Build the Updated Workspace on the ROS PC

After adding the new configuration and waypoint files, rebuild the affected packages on the ROS PC:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    dump_truck_control \
    dump_truck_bringup

source install/setup.bash
```

Because new trucks use the existing generalized launch and control code, a full workspace rebuild is normally unnecessary.

---

### 10. Configure the ROS PC Network

For Truck 4 only:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_network.sh ros_pc dumptruck_04
```

For multi-truck operation, include all required dump trucks:

```bash
source network/setup_network.sh \
  dumptruck_01 \
  dumptruck_03 \
  dumptruck_04
```

---

### 11. Start the ROS PC Control Stack

Start the same generalized ROS PC launch file:

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck4
```

Expected nodes include:

```text
/truck4/odometry
/truck4/tag_odom_fusion
```

Expected topics include:

```text
/truck4/odom
/truck4/fused_odom
```

---

### 12. Run the Truck 4 Waypoint Controller

Run the Truck 4 route using the Truck 4 namespace:

```bash
ros2 run dump_truck_control waypoint_controller_node \
  truck4 \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck4_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck4
```

Verify the node namespace:

```bash
ros2 node list | grep truck4
```

Expected nodes:

```text
/truck4/bucket_action
/truck4/motor_drive
/truck4/odometry
/truck4/tag_odom_fusion
/truck4/waypoint_controller_node
```

No source-code modification should normally be required for additional dump trucks. Adding another truck should primarily require:

* a new entry in `network/devices.sh`
* a new `config/hardware/truckX.yaml`
* a new waypoint YAML if a robot-specific route is needed
* Git commit/push and Pi-side `git pull`
* ROS 2 package build on a newly configured Pi
* launching the common Pi and ROS PC bringup files with the appropriate `truck_name`

---

## 12. Topic-Based Control vs. Command Center

This directory intentionally retains direct topic-based robot control.

It is useful for:

* hardware testing
* calibration
* debugging
* individual robot experiments
* localization testing
* waypoint controller development
* manual multi-robot testing

Higher-level coordinated execution will be implemented separately under:

```text
command_center/
```

The Command Center is intended to support higher-level scenarios such as:

```text
Truck 1 executes Route A
        ↓
Truck 1 completes
        ↓
Truck 3 executes Route B
        ↓
Truck 3 completes
```

and later:

```text
Dump Truck
    +
Excavator
    +
other construction robots
```

using higher-level ROS 2 coordination mechanisms such as Actions.

---

## 13. Useful Diagnostics

List robot nodes:

```bash
ros2 node list | sort
```

List Truck 1 and Truck 3 topics:

```bash
ros2 topic list | grep -E 'truck1|truck3' | sort
```

Check wheel states:

```bash
ros2 topic echo /truck1/wheel_states
```

or:

```bash
ros2 topic echo /truck3/wheel_states
```

Check odometry:

```bash
ros2 topic echo /truck1/odom
```

Check fused odometry:

```bash
ros2 topic echo /truck1/fused_odom
```

Check Truck 3 fused odometry:

```bash
ros2 topic echo /truck3/fused_odom
```

---

## Current Verified Robots

```text
Truck 1
ROS namespace: /truck1
Robot AprilTag: tag36h11_0

Truck 3
ROS namespace: /truck3
Robot AprilTag: tag36h11_2
```

Truck 1 and Truck 3 have been verified for simultaneous operation with isolated ROS namespaces.
