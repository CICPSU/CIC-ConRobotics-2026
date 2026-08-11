# ROS 2 Action-Based Command Center

This directory contains the ROS 2 action-based command center for coordinating multiple construction robots.

The command center provides a higher-level control layer above the individual topic-based robot controllers. Its main purpose is to coordinate robot tasks, execute multi-robot scenarios, and reduce the number of ROS PC terminals required during experiments.

The current implementation has been validated with:

- Dump Truck 1 (`truck1`)
- Dump Truck 3 (`truck3`)
- Overhead camera localization using AprilTags
- Sequential multi-robot scenario execution

---

## 1. Architecture

The repository separates low-level / individual robot control from high-level multi-robot coordination.

```text
ros2_topic_based_control/
│
├── construction_robot_perception/
│   └── Camera and AprilTag perception
│
└── dump_truck/
    ├── dump_truck_hardware/
    ├── dump_truck_control/
    └── dump_truck_bringup/

ros2_action_based_command_center/
│
├── construction_site_interfaces/
│   └── Shared ROS 2 Action definitions
│
├── dump_truck_action_server/
│   └── Action interface for dump truck waypoint execution
│
└── construction_site_control/
    ├── launch/
    │   └── command_center.launch.py
    ├── scenarios/
    │   └── truck1_then_truck3.yaml
    └── construction_site_control/
        └── scenario_manager_node.py
```

The topic-based system remains available for individual robot operation, debugging, and low-level testing.

The action-based command center is used for higher-level task execution and multi-robot coordination.

---

## 2. Current Control Flow

The current system follows this architecture:

```text
Scenario YAML
     │
     ▼
Scenario Manager
     │
     ├── /truck1/execute_robot_task
     │           │
     │           ▼
     │    Truck 1 Action Server
     │           │
     │           ▼
     │    Existing topic-based control
     │
     └── /truck3/execute_robot_task
                 │
                 ▼
          Truck 3 Action Server
                 │
                 ▼
          Existing topic-based control
```

Localization is provided by:

```text
Overhead Camera
      │
      ▼
AprilTag Detector
      │
      ▼
Tag/Odom Fusion
      │
      ├── /truck1/fused_odom
      └── /truck3/fused_odom
```

---

## 3. Initial Build

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
    dump_truck_bringup \
    construction_site_interfaces \
    dump_truck_action_server \
    construction_site_control
```

Then:

```bash
source install/setup.bash
```

---

## 4. Network Setup

The network helper must be sourced before starting the ROS 2 system.

For the current Truck 1 + Truck 3 configuration on the ROS PC:

```bash
source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03
```

The ROS PC name is intentionally specified explicitly so that another ROS PC, such as `ros_pc_backup`, can be used in the future.

---

## 5. Start Truck 1 Raspberry Pi

On the Truck 1 Raspberry Pi:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03
```

Start `pigpiod` if it is not already running:

```bash
sudo pigpiod
```

Then:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck1
```

---

## 6. Start Truck 3 Raspberry Pi

On the Truck 3 Raspberry Pi:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03
```

Start `pigpiod` if it is not already running:

```bash
sudo pigpiod
```

Then:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck3
```

---

## 7. Start the Command Center

On the ROS PC:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03
```

### Full Truck 1 + Truck 3 scenario

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:="truck1,truck3" \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=true \
  scenario:=truck1_then_truck3.yaml
```

This starts the ROS PC side of the system, including:

- overhead camera
- AprilTag detector
- Truck 1 odometry and localization
- Truck 3 odometry and localization
- Truck 1 Action Server
- Truck 3 Action Server
- Scenario Manager

---

## 8. Command Center Launch Parameters

The command center is intentionally parameterized so that different experiment configurations can be launched without modifying source code.

### `trucks`

Comma-separated list of dump trucks.

Example:

```bash
trucks:="truck1,truck3"
```

Single truck:

```bash
trucks:=truck1
```

### `start_camera`

Start the overhead USB camera.

```bash
start_camera:=true
```

### `start_apriltag`

Start the AprilTag detector.

```bash
start_apriltag:=true
```

### `start_localization`

Start odometry and Tag/Odom fusion for each selected truck.

```bash
start_localization:=true
```

### `start_action_servers`

Start the Action Server for each selected truck.

```bash
start_action_servers:=true
```

### `start_scenario_manager`

Automatically execute a scenario YAML.

```bash
start_scenario_manager:=true
```

Set this to `false` when testing Action Servers manually.

### `scenario`

Scenario YAML file to execute.

Example:

```bash
scenario:=truck1_then_truck3.yaml
```

---

## 9. Example Launch Configurations

### Truck 1 only, without automatic scenario execution

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

### Truck 1 and Truck 3, without scenario execution

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:="truck1,truck3" \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

This is useful for manually sending Action goals.

---

## 10. ROS 2 Action Interface

The shared Action definition is:

```text
construction_site_interfaces/action/ExecuteRobotTask.action
```

Current definition:

```text
# Goal
string robot_name
string task_type
string task_file
---
# Result
bool success
string message
---
# Feedback
string state
float32 progress
string detail
```

Each truck exposes its own Action interface.

For example:

```text
/truck1/execute_robot_task
/truck3/execute_robot_task
```

Check available Actions with:

```bash
ros2 action list | sort
```

---

## 11. Manual Action Test

With the Command Center running and:

```bash
start_scenario_manager:=false
```

a task can be sent manually.

Truck 1 example:

```bash
ros2 action send_goal \
  /truck1/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck1, task_type: waypoint, task_file: truck1_waypoints.yaml}" \
  --feedback
```

Truck 3 example:

```bash
ros2 action send_goal \
  /truck3/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck3, task_type: waypoint, task_file: truck3_waypoints.yaml}" \
  --feedback
```

---

## 12. Scenario YAML

Scenarios are stored under:

```text
construction_site_control/scenarios/
```

Example:

```yaml
scenario_name: truck1_then_truck3

steps:

  - id: truck1_route
    robot: truck1
    task_type: waypoint
    task_file: truck1_waypoints.yaml

  - id: truck3_route
    robot: truck3
    task_type: waypoint
    task_file: truck3_waypoints.yaml
```

The Scenario Manager executes the steps sequentially.

Current behavior:

```text
Truck 1 task
     │
     ▼
SUCCESS
     │
     ▼
Truck 3 task
     │
     ▼
SUCCESS
     │
     ▼
SCENARIO COMPLETE
```

If a task fails, the current scenario is aborted rather than continuing to the next step.

---

## 13. Verify the Running System

Check nodes:

```bash
ros2 node list | sort
```

For Truck 1 + Truck 3, the ROS PC should include nodes similar to:

```text
/apriltag
/usb_cam
/truck1/odometry
/truck1/tag_odom_fusion
/truck1/waypoint_action_server
/truck3/odometry
/truck3/tag_odom_fusion
/truck3/waypoint_action_server
```

Check Actions:

```bash
ros2 action list | sort
```

Expected:

```text
/truck1/execute_robot_task
/truck3/execute_robot_task
```

---

## 14. Design Philosophy

The command center should remain independent from robot-specific low-level implementation as much as possible.

The intended hierarchy is:

```text
Construction Scenario
        │
        ▼
Scenario Manager
        │
        ▼
Robot Action Interface
        │
        ▼
Robot-Specific Controller
        │
        ▼
Hardware
```

This allows future robot types, including excavators, to participate in construction scenarios through the same high-level coordination architecture.

Future extensions may include:

- excavator Action Servers
- waiting steps
- timed delays
- parallel robot tasks
- synchronization between robots
- conditional task execution
- abnormal-condition handling
- scenario pause / resume / cancel
- additional construction robot types

---

## 15. Current Verified Configuration

The following sequence has been verified with physical robots:

```text
Truck 1 executes truck1_waypoints.yaml
              │
              ▼
           SUCCESS
              │
              ▼
Truck 3 executes truck3_waypoints.yaml
              │
              ▼
           SUCCESS
              │
              ▼
       SCENARIO COMPLETE
```

The ROS PC side is launched through a single parameterized Command Center launch file, while each Raspberry Pi runs its own robot hardware bringup.