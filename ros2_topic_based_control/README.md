# ROS 2 Topic-Based Robot Control

This directory contains the ROS 2 topic-based control implementation for individual construction robots.

The purpose of this layer is to provide direct control, localization, hardware interfaces, odometry, and manual waypoint execution for each robot independently.

Higher-level multi-robot coordination and scenario execution are implemented separately under the ROS 2 Action-based Command Center.

The current dump truck implementation has been physically validated with:

- Truck 1 (`truck1`)
- Truck 3 (`truck3`)
- Truck 4 (`truck4`)
- Truck 5 (`truck5`)
- shared ROS 2 hardware and control nodes
- per-truck hardware configuration
- wheel-encoder odometry
- overhead AprilTag localization
- Tag/Odom fusion
- manual waypoint execution
- simultaneous multi-truck ROS communication

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
    │   │   ├── truck3_waypoints.yaml
    │   │   ├── truck4_waypoints.yaml
    │   │   └── truck5_waypoints.yaml
    │   └── ...
    │
    └── dump_truck_bringup/
        ├── config/
        │   ├── hardware/
        │   │   ├── truck1.yaml
        │   │   ├── truck3.yaml
        │   │   ├── truck4.yaml
        │   │   └── truck5.yaml
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

The network setup script configures the ROS 2 communication environment for the selected computers.

### ROS PC

For Truck 1 only:

```bash
source network/setup_network.sh ros_pc dumptruck_01
```

For Truck 3 only:

```bash
source network/setup_network.sh ros_pc dumptruck_03
```

For Truck 4 only:

```bash
source network/setup_network.sh ros_pc dumptruck_04
```

For Truck 5 only:

```bash
source network/setup_network.sh ros_pc dumptruck_05
```

For the current four-truck configuration:

```bash
source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03 \
  dumptruck_04 \
  dumptruck_05
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

Verify:

```bash
git branch --show-current
```

Expected:

```text
dev
```

---

### If the Repository Already Exists

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

git checkout dev
git pull origin dev
```

---

### Build on the ROS PC

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

After cloning or pulling updated dump-truck code:

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

Then:

```bash
source install/setup.bash
```

When `motor_drive_node.py`, truck hardware YAML files, or bringup launch files are updated, rebuild the affected packages before restarting the robot.

A typical dump-truck rebuild is:

```bash
colcon build \
  --symlink-install \
  --packages-select \
    dump_truck_hardware \
    dump_truck_bringup

source install/setup.bash
```

For a newly configured Raspberry Pi, `git` and `colcon` must also be installed before cloning and building the repository.

---

## 4. Truck-Specific Configuration

Truck-specific parameters are stored in:

```text
dump_truck_bringup/config/hardware/
```

Current files include:

```text
truck1.yaml
truck3.yaml
truck4.yaml
truck5.yaml
```

The same common ROS 2 hardware, control, and localization nodes are used across the trucks.

Physical differences between trucks are represented in YAML configuration rather than separate robot-specific source code.

Truck-specific parameters can include:

- bucket servo calibration
- robot AprilTag frame
- tag yaw offset
- odometry scale factors
- encoder direction mode
- encoder glitch filtering
- encoder sign inversion
- physical left/right encoder mapping

Example:

```yaml
bucket_action_node:
  ros__parameters:
    servo_center: 900
    servo_dump: 1300

motor_drive_node:
  ros__parameters:
    encoder_direction_mode: commanded
    encoder_glitch_filter_us: 200

    left_encoder_invert: false
    right_encoder_invert: false

    swap_encoders: false

tag_odom_fusion_landmarks_node:
  ros__parameters:
    robot_tag_child_frame: tag36h11_0

    tag_yaw_offset: 1.57079632679

    odom_x_scale: 1.0
    odom_y_scale: 1.0
    odom_yaw_scale: 1.0
```

---

## 5. Encoder Mapping and Physical Truck Differences

Physical encoder installation is not identical across all four dump trucks.

The common `motor_drive_node` therefore supports truck-specific encoder configuration.

Important parameters include:

```text
encoder_direction_mode
encoder_glitch_filter_us
left_encoder_invert
right_encoder_invert
swap_encoders
```

The `swap_encoders` parameter maps the physical encoder GPIO channels to the logical left and right robot wheels.

The currently verified left/right mapping is:

```text
Truck 1: swap_encoders = false
Truck 3: swap_encoders = false
Truck 4: swap_encoders = true
Truck 5: swap_encoders = true
```

For Trucks 4 and 5, the physical encoder left/right mapping is opposite to the logical robot wheel mapping used by the controller.

When:

```yaml
swap_encoders: true
```

the motor-drive implementation corrects both:

1. the encoder values published as logical `left_wheel` and `right_wheel`
2. the commanded encoder-direction mapping used when encoder sign is derived from the commanded wheel direction

Both corrections are necessary.

Correcting only the published left/right wheel values may make basic forward motion or turning appear correct while still producing incorrect odometry during steering corrections.

Incorrect encoder mapping can cause:

- yaw to change in the wrong direction
- continuous turning
- waypoint tracking failure
- left/right corrective oscillation
- disagreement between physical motion and odometry

The current configuration has been physically validated with Trucks 1, 3, 4, and 5.

---

## 6. Truck AprilTag Configuration

Current robot AprilTags are:

```text
Truck 1: tag36h11_0
Truck 3: tag36h11_2
Truck 4: tag36h11_3
Truck 5: tag36h11_4
```

These values are configured through the corresponding:

```text
config/hardware/truckX.yaml
```

files.

---

## 7. Shared AprilTag Landmarks

Fixed overhead-camera calibration landmarks are stored in:

```text
dump_truck_bringup/config/localization/landmarks.yaml
```

Current fixed landmarks include:

```text
tag36h11_16
tag36h11_17
tag36h11_18
```

These landmarks are shared by all dump trucks.

---

## 8. Raspberry Pi Bringup

Before launching a truck, start `pigpiod` if necessary:

```bash
sudo pigpiod
```

Each truck Pi should enter the repository and source the ROS environment:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

For the current four-truck network:

```bash
source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03 \
  dumptruck_04 \
  dumptruck_05
```

### Truck 1

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck1
```

### Truck 3

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck3
```

### Truck 4

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck4
```

### Truck 5

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck5
```

Expected topics follow the same namespace pattern:

```text
/<truck_name>/cmd_vel
/<truck_name>/wheel_states
/<truck_name>/bucket_action_cmd
/<truck_name>/bucket_action_status
```

Expected hardware nodes:

```text
/<truck_name>/motor_drive
/<truck_name>/bucket_action
```

---

## 9. ROS PC Bringup

The same generalized ROS PC launch file is used for all dump trucks.

### Truck 1

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck1
```

### Truck 3

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck3
```

### Truck 4

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck4
```

### Truck 5

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck5
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

## 10. Four-Truck ROS PC Bringup

For manual topic-based operation of all four trucks, source the four-truck network configuration:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03 \
  dumptruck_04 \
  dumptruck_05
```

Then start one ROS PC bringup per truck.

Terminal 1:

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck1
```

Terminal 2:

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck3
```

Terminal 3:

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck4
```

Terminal 4:

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck5
```

Expected localization topics:

```text
/truck1/odom
/truck1/fused_odom

/truck3/odom
/truck3/fused_odom

/truck4/odom
/truck4/fused_odom

/truck5/odom
/truck5/fused_odom
```

---

## 11. Overhead Camera and AprilTag Detection

The overhead camera and AprilTag detector must be running before AprilTag-based fusion can initialize.

Start:

```bash
ros2 launch \
  construction_robot_perception \
  overhead_camera.launch.py
```

Current robot tag frames include:

```text
tag36h11_0
tag36h11_2
tag36h11_3
tag36h11_4
```

Current fixed landmark frames include:

```text
tag36h11_16
tag36h11_17
tag36h11_18
```

The mapping is:

```text
tag36h11_0 = Truck 1
tag36h11_2 = Truck 3
tag36h11_3 = Truck 4
tag36h11_4 = Truck 5

tag36h11_16
tag36h11_17
tag36h11_18 = fixed landmarks
```

TF can be checked directly, for example:

```bash
ros2 run tf2_ros tf2_echo default_cam tag36h11_4
```

---

## 12. Manual `cmd_vel` Testing

Direct `cmd_vel` is useful when validating motor direction, encoder mapping, and odometry before waypoint execution.

### Straight Forward Motion

Truck 1 example:

```bash
ros2 topic pub -r 10 \
  /truck1/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}"
```

Truck 5 example:

```bash
ros2 topic pub -r 10 \
  /truck5/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}"
```

### Forward + Right Turn

```bash
ros2 topic pub -r 10 \
  /truck5/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: -0.35}}"
```

For a right turn, the logical left encoder should advance more than the logical right encoder.

### In-Place Right Rotation

```bash
ros2 topic pub -r 10 \
  /truck5/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.70}}"
```

The physical robot should rotate clockwise.

The odometry yaw should also change in the corresponding negative ROS yaw direction.

---

## 13. Manual Waypoint Execution

The waypoint controller remains available as a topic-based individual robot control method.

### Truck 1

```bash
ros2 run dump_truck_control waypoint_controller_node \
  truck1 \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck1_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck1
```

### Truck 3

```bash
ros2 run dump_truck_control waypoint_controller_node \
  truck3 \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck3_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck3
```

Truck 4 and Truck 5 follow the same convention:

```bash
ros2 run dump_truck_control waypoint_controller_node \
  truck4 \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck4_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck4
```

```bash
ros2 run dump_truck_control waypoint_controller_node \
  truck5 \
  ros2_topic_based_control/dump_truck/dump_truck_control/waypoints/truck5_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck5
```

---

## 14. Adding a New Dump Truck

For a new truck, for example Truck 6:

### 1. Add the Device to the Network Configuration

Update:

```text
network/devices.sh
```

Add the new Raspberry Pi device entry.

---

### 2. Create the Hardware Configuration

Copy an existing configuration:

```bash
cp \
  ros2_topic_based_control/dump_truck/dump_truck_bringup/config/hardware/truck5.yaml \
  ros2_topic_based_control/dump_truck/dump_truck_bringup/config/hardware/truck6.yaml
```

Update:

- servo calibration
- `robot_tag_child_frame`
- tag yaw offset if required
- odometry scale factors if required
- encoder direction configuration
- encoder inversion if required
- `swap_encoders` after physical validation

Do not assume encoder left/right mapping is identical between trucks.

Verify the new truck using manual `cmd_vel` and `wheel_states` tests before waypoint operation.

---

### 3. Create a Waypoint File

For example:

```text
dump_truck_control/waypoints/truck6_waypoints.yaml
```

---

### 4. Commit and Push

From the ROS PC:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

git status
git add .
git commit -m "Add Truck 6 configuration"
git push origin dev
```

---

### 5. Prepare the New Raspberry Pi

For a new Pi:

```bash
mkdir -p ~/ws_conrobotics
cd ~/ws_conrobotics

git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git

cd CIC-ConRobotics-2026
git checkout dev
```

For an existing clone:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

git checkout dev
git pull origin dev
```

---

### 6. Build on the New Pi

```bash
source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    dump_truck_control \
    dump_truck_hardware \
    dump_truck_bringup

source install/setup.bash
```

If a clean rebuild is necessary:

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

### 7. Start the New Truck Pi

```bash
sudo pigpiod
```

Then:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck6
```

---

### 8. Build the Updated ROS PC Workspace

After adding a new hardware configuration or waypoint file:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

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

### 9. Start the ROS PC Stack

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck6
```

Expected:

```text
/truck6/odometry
/truck6/tag_odom_fusion
/truck6/odom
/truck6/fused_odom
```

---

### 10. Validate Encoder Mapping

Before waypoint operation, first check:

```bash
ros2 topic echo /truck6/wheel_states
```

Then send:

```bash
ros2 topic pub -r 10 \
  /truck6/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: -0.35}}"
```

For a correct right turn:

```text
logical LEFT wheel  > logical RIGHT wheel
```

If the opposite occurs, inspect the truck-specific encoder mapping before modifying downstream odometry logic.

---

## 15. Topic-Based Control vs. Action-Based Command Center

This directory intentionally retains direct topic-based robot control.


It is useful for:

- hardware testing
- calibration
- debugging
- individual robot experiments
- localization testing
- encoder diagnostics
- odometry diagnostics
- waypoint controller development
- manual multi-robot testing


For higher-level multi-robot coordination, task sequencing, and construction
scenario execution, see the
[ROS 2 Action-Based Command Center](../ros2_action_based_command_center/README.md).
The Action-Based Command Center provides the higher-level coordination layer.


Conceptually:

```text
Topic-Based Layer
│
├── Hardware
├── cmd_vel
├── wheel_states
├── odometry
├── fused odometry
└── individual robot control
        │
        ▼
Action-Based Command Center
│
├── robot tasks
├── sequencing
├── synchronization
├── conditions
├── parallel execution
└── construction scenarios

---

## 16. Useful Diagnostics

List all robot nodes:

```bash
ros2 node list | sort
```

List all four truck topics:

```bash
ros2 topic list \
  | grep -E 'truck1|truck3|truck4|truck5' \
  | sort
```

Check wheel states:

```bash
ros2 topic echo /truck1/wheel_states
```

```bash
ros2 topic echo /truck3/wheel_states
```

```bash
ros2 topic echo /truck4/wheel_states
```

```bash
ros2 topic echo /truck5/wheel_states
```

Check odometry:

```bash
ros2 topic echo /truck1/odom
```

Check fused odometry:

```bash
ros2 topic echo /truck1/fused_odom
```

Check Truck 5 fused odometry:

```bash
ros2 topic echo /truck5/fused_odom
```

Check `cmd_vel` connectivity:

```bash
ros2 topic info /truck1/cmd_vel -v
```

Check a truck's hardware parameters:

```bash
ros2 param dump /truck1/motor_drive
```

Check odometry angular velocity:

```bash
ros2 topic echo \
  /truck5/odom \
  --field twist.twist.angular.z
```

Check accumulated orientation:

```bash
ros2 topic echo \
  /truck5/odom \
  --field pose.pose.orientation
```

---

## 17. Current Verified Robots

```text
Truck 1
ROS namespace: /truck1
Robot AprilTag: tag36h11_0
Encoder swap: false

Truck 3
ROS namespace: /truck3
Robot AprilTag: tag36h11_2
Encoder swap: false

Truck 4
ROS namespace: /truck4
Robot AprilTag: tag36h11_3
Encoder swap: true

Truck 5
ROS namespace: /truck5
Robot AprilTag: tag36h11_4
Encoder swap: true
```

All four trucks have been verified with isolated ROS namespaces.

The current common software architecture supports:

```text
Truck-specific hardware YAML
          │
          ▼
Common Motor Drive Node
          │
          ▼
/<truck>/wheel_states
          │
          ▼
Common Odometry Node
          │
          ▼
/<truck>/odom
          │
          ▼
Common Tag/Odom Fusion
          │
          ▼
/<truck>/fused_odom
```

Truck-specific hardware differences are handled primarily through configuration rather than duplicated source code.

The current topic-based implementation provides the validated low-level foundation for the four-truck ROS 2 Action-based Command Center.