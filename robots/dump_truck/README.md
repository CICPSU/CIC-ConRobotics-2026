# ROS 2 Dump Truck System

This directory contains the ROS 2 implementation for the physical dump truck robots used in the CIC construction robotics platform.

The system supports both individual robot operation and higher-level multi-robot coordination.

The current implementation has been physically validated with:

- Truck 1 (`truck1`)
- Truck 3 (`truck3`)
- Truck 4 (`truck4`)
- Truck 5 (`truck5`)
- shared ROS 2 hardware and control nodes
- per-truck hardware configuration
- wheel-encoder odometry
- overhead AprilTag localization
- Tag/Odom fusion
- manual `cmd_vel` control
- waypoint execution
- ROS 2 Action-based task execution
- multi-truck ROS communication
- sequential multi-robot execution
- timed wait steps
- parallel robot execution
- conditional execution
- direct ROS topic publishing
- runtime logging and scenario review

Truck-specific physical differences are handled primarily through YAML configuration rather than duplicated source code.

---

# 1. Repository Architecture

The current repository separates robot software, perception, operational data, shared interfaces, and high-level coordination by responsibility.

```text
CIC-ConRobotics-2026/
│
├── robots/
│   └── dump_truck/
│       ├── README.md
│       │
│       ├── dump_truck_hardware/
│       │   └── dump_truck_hardware/
│       │       ├── motor_drive_node.py
│       │       └── bucket_action_node.py
│       │
│       ├── dump_truck_control/
│       │   └── dump_truck_control/
│       │       ├── odometry_node.py
│       │       ├── tag_odom_fusion_node.py
│       │       └── waypoint_controller_node.py
│       │
│       └── dump_truck_bringup/
│           ├── config/
│           │   ├── hardware/
│           │   │   ├── truck1.yaml
│           │   │   ├── truck3.yaml
│           │   │   ├── truck4.yaml
│           │   │   └── truck5.yaml
│           │   │
│           │   └── localization/
│           │       └── landmarks.yaml
│           │
│           └── launch/
│               ├── dump_truck_pi.launch.py
│               ├── dump_truck_ros_pc.launch.py
│               ├── truck1_pi.launch.py
│               └── truck1_ros_pc.launch.py
│
├── perception/
│   └── construction_robot_perception/
│       ├── config/
│       │   ├── tags_multi_truck.yaml
│       │   ├── usb_cam_obsbot.yaml
│       │   └── legacy_obsbot/
│       └── launch/
│           └── overhead_camera.launch.py
│
├── common/
│   └── construction_site_interfaces/
│       ├── action/
│       │   └── ExecuteRobotTask.action
│       └── msg/
│           └── RobotStatus.msg
│
├── command_center/
│   ├── dump_truck_action_server/
│   │   └── dump_truck_action_server/
│   │       └── waypoint_action_server_node.py
│   │
│   └── construction_site_control/
│       ├── launch/
│       │   └── command_center.launch.py
│       └── construction_site_control/
│           └── scenario_manager_node.py
│
├── operations/
│   ├── dump_truck/
│   │   └── waypoints/
│   │       ├── truck1_waypoints.yaml
│   │       ├── truck1_waypoints2.yaml
│   │       ├── truck1_waypoints3.yaml
│   │       ├── truck3_waypoints.yaml
│   │       ├── truck3_waypoints2.yaml
│   │       ├── truck4_waypoints.yaml
│   │       ├── truck4_waypoints2.yaml
│   │       ├── truck5_waypoints.yaml
│   │       └── truck5_waypoints2.yaml
│   │
│   └── scenarios/
│
├── network/
│   ├── devices.sh
│   └── setup_network.sh
│
├── docs/
│   └── perception/
│       └── apriltag/
│           └── Calibration.md
│
└── tools/
    └── perception/
        └── read_tag_tf.py
```

The major responsibilities are:

```text
robots/dump_truck/
    Robot hardware, control, configuration, and bringup

perception/
    Shared camera and AprilTag perception

operations/
    Waypoints and construction scenario definitions

common/
    Shared ROS 2 messages and actions

command_center/
    Task execution and multi-robot coordination

network/
    ROS 2 network configuration
```

---

# 2. Control Architecture

The dump truck system supports several levels of operation.

At the individual robot level:

```text
Truck-Specific Hardware YAML
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
Tag/Odom Fusion
          │
          ▼
/<truck>/fused_odom
          │
          ▼
Waypoint Controller
          │
          ▼
/<truck>/cmd_vel
          │
          ▼
Physical Dump Truck
```

At the higher coordination level:

```text
Scenario YAML
operations/scenarios/
          │
          ▼
Scenario Manager
          │
          ├── task
          ├── wait
          ├── parallel
          ├── condition
          └── topic_publish
          │
          ▼
/<truck>/execute_robot_task
          │
          ▼
Dump Truck Action Server
          │
          ▼
Waypoint YAML
operations/dump_truck/waypoints/
          │
          ▼
Waypoint Controller
          │
          ▼
/<truck>/cmd_vel
          │
          ▼
Physical Dump Truck
```

This allows the same robot software to be used for:

- hardware testing
- calibration
- manual robot operation
- localization testing
- waypoint navigation
- task-level execution
- multi-robot construction scenarios

---

# 3. ROS 2 Packages

## `dump_truck_hardware`

Location:

```text
robots/dump_truck/dump_truck_hardware/
```

Key nodes:

```text
motor_drive_node.py
bucket_action_node.py
```

Primary responsibilities:

- motor control
- wheel encoder acquisition
- wheel-state publishing
- bucket servo control
- physical hardware interfaces

This package normally runs on the Raspberry Pi installed on each truck.

---

## `dump_truck_control`

Location:

```text
robots/dump_truck/dump_truck_control/
```

Key nodes:

```text
odometry_node.py
tag_odom_fusion_node.py
waypoint_controller_node.py
```

Primary responsibilities:

- wheel-encoder odometry
- AprilTag/Odom fusion
- waypoint tracking
- robot-level motion control

Operational waypoint files are stored separately under:

```text
operations/dump_truck/waypoints/
```

---

## `dump_truck_bringup`

Location:

```text
robots/dump_truck/dump_truck_bringup/
```

This package contains:

- ROS 2 launch files
- truck-specific hardware configuration
- localization configuration

Hardware configurations:

```text
robots/dump_truck/dump_truck_bringup/config/hardware/
```

Shared localization landmarks:

```text
robots/dump_truck/dump_truck_bringup/config/localization/landmarks.yaml
```

---

## `dump_truck_action_server`

Location:

```text
command_center/dump_truck_action_server/
```

The Action Server provides task-level waypoint execution for each dump truck.

Key node:

```text
waypoint_action_server_node.py
```

Each selected truck exposes:

```text
/<truck_name>/execute_robot_task
```

---

## `construction_site_control`

Location:

```text
command_center/construction_site_control/
```

Key files:

```text
launch/command_center.launch.py
construction_site_control/scenario_manager_node.py
```

This package provides the higher-level Command Center and Scenario Manager.

---

# 4. Initial Repository Setup

If the repository has not been cloned:

```bash
mkdir -p ~/ws_conrobotics
cd ~/ws_conrobotics

git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git

cd CIC-ConRobotics-2026
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

If the repository already exists:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

git checkout dev
git pull origin dev
```

---

# 5. Build

## ROS PC

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

## Raspberry Pi

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

source install/setup.bash
```

For a typical hardware update:

```bash
colcon build \
  --symlink-install \
  --packages-select \
    dump_truck_hardware \
    dump_truck_bringup

source install/setup.bash
```

If a clean rebuild is necessary:

```bash
rm -rf build install log

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

# 6. ROS 2 Network Setup

Network configuration is handled by:

```text
network/setup_network.sh
network/devices.sh
```

Each new terminal requires the appropriate network setup to be sourced.

For Truck 1:

```bash
source network/setup_network.sh ros_pc dumptruck_01
```

For Truck 3:

```bash
source network/setup_network.sh ros_pc dumptruck_03
```

For Truck 4:

```bash
source network/setup_network.sh ros_pc dumptruck_04
```

For Truck 5:

```bash
source network/setup_network.sh ros_pc dumptruck_05
```

For the complete four-truck system:

```bash
source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03 \
  dumptruck_04 \
  dumptruck_05
```

---

# 7. Truck-Specific Configuration

Truck-specific parameters are stored under:

```text
robots/dump_truck/dump_truck_bringup/config/hardware/
```

Current files:

```text
truck1.yaml
truck3.yaml
truck4.yaml
truck5.yaml
```

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

# 8. Encoder Mapping

Physical encoder installation is not identical across all trucks.

Important parameters include:

```text
encoder_direction_mode
encoder_glitch_filter_us
left_encoder_invert
right_encoder_invert
swap_encoders
```

The currently verified mapping is:

```text
Truck 1: swap_encoders = false
Truck 3: swap_encoders = false
Truck 4: swap_encoders = true
Truck 5: swap_encoders = true
```

For Trucks 4 and 5, physical encoder left/right mapping is opposite to the logical wheel mapping.

When:

```yaml
swap_encoders: true
```

the implementation corrects:

1. encoder values published as logical `left_wheel` and `right_wheel`
2. commanded encoder-direction mapping

Incorrect encoder mapping can cause:

- incorrect yaw direction
- continuous turning
- waypoint tracking failure
- steering oscillation
- disagreement between physical motion and odometry

---

# 9. AprilTag Configuration

Current robot AprilTags:

```text
Truck 1: tag36h11_0
Truck 3: tag36h11_2
Truck 4: tag36h11_3
Truck 5: tag36h11_4
```

Fixed overhead-camera landmarks:

```text
tag36h11_16
tag36h11_17
tag36h11_18
```

Landmarks are stored in:

```text
robots/dump_truck/dump_truck_bringup/config/localization/landmarks.yaml
```

Shared perception configuration is stored under:

```text
perception/construction_robot_perception/
```

Additional calibration documentation:

```text
docs/perception/apriltag/Calibration.md
```

---

# 10. Start Each Raspberry Pi

Each physical truck must have its Raspberry Pi hardware stack running before the complete Command Center system is used.

On each Pi:

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

Start `pigpiod` if necessary:

```bash
sudo pigpiod
```

Then launch the appropriate truck.

Truck 1:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck1
```

Truck 3:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck3
```

Truck 4:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck4
```

Truck 5:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck5
```

Expected hardware nodes follow the pattern:

```text
/<truck_name>/motor_drive
/<truck_name>/bucket_action
```

Expected topics include:

```text
/<truck_name>/cmd_vel
/<truck_name>/wheel_states
/<truck_name>/bucket_action_cmd
/<truck_name>/bucket_action_status
```

---

# 11. Individual ROS PC Bringup

For debugging or individual robot operation, each truck can be started separately on the ROS PC.

Example:

```bash
ros2 launch dump_truck_bringup dump_truck_ros_pc.launch.py \
  truck_name:=truck1
```

The same command can be used for:

```text
truck3
truck4
truck5
```

The launch starts localization components including:

```text
/<truck_name>/odometry
/<truck_name>/tag_odom_fusion
```

and produces:

```text
/<truck_name>/odom
/<truck_name>/fused_odom
```

---

# 12. Overhead Camera and AprilTag Detection

For standalone perception testing:

```bash
ros2 launch \
  construction_robot_perception \
  overhead_camera.launch.py
```

TF can be checked directly.

Example:

```bash
ros2 run tf2_ros tf2_echo default_cam tag36h11_4
```

The complete Command Center launch can also start the camera and AprilTag detector automatically.

---

# 13. Complete Four-Truck System

This is the primary launch procedure for the integrated four-truck system.

Before starting the Command Center:

1. Start the Raspberry Pi hardware stack on Truck 1.
2. Start the Raspberry Pi hardware stack on Truck 3.
3. Start the Raspberry Pi hardware stack on Truck 4.
4. Start the Raspberry Pi hardware stack on Truck 5.

Then, on the ROS PC:

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

Launch the complete four-truck system:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:="truck1,truck3,truck4,truck5" \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=true \
  scenario:=truck1_3_4_5.yaml
```

This launch may start:

```text
Overhead USB Camera
AprilTag Detector

Truck 1 Odometry
Truck 1 Tag/Odom Fusion
Truck 1 Action Server

Truck 3 Odometry
Truck 3 Tag/Odom Fusion
Truck 3 Action Server

Truck 4 Odometry
Truck 4 Tag/Odom Fusion
Truck 4 Action Server

Truck 5 Odometry
Truck 5 Tag/Odom Fusion
Truck 5 Action Server

Scenario Manager
```

Conceptually:

```text
                 Overhead Camera
                       │
                       ▼
                AprilTag Detector
                       │
        ┌──────────────┼──────────────┐
        │              │              │
        ▼              ▼              ▼
   Localization   Localization   Localization ...
        │              │              │
        ▼              ▼              ▼
   Action Server  Action Server  Action Server
        │              │              │
        └──────────────┼──────────────┘
                       │
                       ▼
                Scenario Manager
```

The scenario itself is stored under:

```text
operations/scenarios/
```

Individual robot paths are stored separately under:

```text
operations/dump_truck/waypoints/
```

---

# 14. Command Center Launch Parameters

The Command Center launch is intentionally parameterized.

## `trucks`

Comma-separated list of dump trucks.

```bash
trucks:="truck1,truck3,truck4,truck5"
```

Single truck:

```bash
trucks:=truck1
```

---

## `start_camera`

Start the overhead USB camera:

```bash
start_camera:=true
```

---

## `start_apriltag`

Start the AprilTag detector:

```bash
start_apriltag:=true
```

---

## `start_localization`

Start odometry and Tag/Odom fusion for each selected truck:

```bash
start_localization:=true
```

---

## `start_action_servers`

Start an Action Server for each selected truck:

```bash
start_action_servers:=true
```

---

## `start_scenario_manager`

Automatically execute a scenario YAML:

```bash
start_scenario_manager:=true
```

For manual Action testing:

```bash
start_scenario_manager:=false
```

---

## `scenario`

Scenario YAML to execute.

Example:

```bash
scenario:=truck1_3_4_5.yaml
```

The Scenario Manager resolves scenario files from:

```text
operations/scenarios/
```

---

# 15. Command Center Without Automatic Scenario Execution

To start the system but manually control Action execution:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:="truck1,truck3,truck4,truck5" \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

This is useful for:

- Action Server testing
- individual task testing
- waypoint debugging
- integration testing

---

# 16. ROS 2 Action Interface

The shared Action definition is located at:

```text
common/construction_site_interfaces/action/ExecuteRobotTask.action
```

Definition:

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

Each truck exposes:

```text
/<truck_name>/execute_robot_task
```

For example:

```text
/truck1/execute_robot_task
/truck3/execute_robot_task
/truck4/execute_robot_task
/truck5/execute_robot_task
```

Check available Actions:

```bash
ros2 action list | sort
```

---

# 17. Manual Action Test

Start the Command Center with:

```bash
start_scenario_manager:=false
```

Then send a task manually.

Truck 1:

```bash
ros2 action send_goal \
  /truck1/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck1, task_type: waypoint, task_file: truck1_waypoints.yaml}" \
  --feedback
```

Truck 3:

```bash
ros2 action send_goal \
  /truck3/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck3, task_type: waypoint, task_file: truck3_waypoints.yaml}" \
  --feedback
```

Truck 4:

```bash
ros2 action send_goal \
  /truck4/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck4, task_type: waypoint, task_file: truck4_waypoints.yaml}" \
  --feedback
```

Truck 5:

```bash
ros2 action send_goal \
  /truck5/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck5, task_type: waypoint, task_file: truck5_waypoints.yaml}" \
  --feedback
```

The Action Server resolves waypoint files from:

```text
operations/dump_truck/waypoints/
```

---

# 18. Shared Robot Status

The shared robot status message is located at:

```text
common/construction_site_interfaces/msg/RobotStatus.msg
```

Definition:

```text
string robot_name
string state
string detail
```

The general topic convention is:

```text
/<robot_name>/status
```

For the current trucks:

```text
/truck1/status
/truck3/status
/truck4/status
/truck5/status
```

Example:

```yaml
robot_name: truck1
state: navigating
detail: Executing waypoint task: truck1_waypoints.yaml
```

Common states include:

```text
idle
waiting
navigating
performing_action
completed
fault
```

Robot status can be used by the Scenario Manager for state-based synchronization.

---

# 19. Scenario YAML

Scenario files are stored under:

```text
operations/scenarios/
```

Current scenarios include:

```text
test_condition.yaml
test_robot_status_condition.yaml
test_topic_publish.yaml
test_truck1_cmd_vel.yaml

truck1_3_4_5.yaml
truck1_complete_then_truck3.yaml
truck1_dump_test.yaml
truck1_then_truck3.yaml
truck1_truck3_parallel.yaml
truck1_wait_truck3.yaml
truck5_then_truck4.yaml
```

The Scenario Manager currently supports:

```text
task
wait
parallel
condition
topic_publish
```

---

# 20. `task` Step

A `task` executes a robot task through its ROS 2 Action Server.

Example:

```yaml
- id: truck1_route
  type: task
  robot: truck1
  task_type: waypoint
  task_file: truck1_waypoints.yaml
```

The task is sent to:

```text
/truck1/execute_robot_task
```

The Scenario Manager waits for the Action result before continuing.

---

# 21. `wait` Step

A `wait` step introduces a timed delay.

Example:

```yaml
- id: wait_after_truck1
  type: wait
  duration: 3.0
```

Flow:

```text
Truck 1 Task
     │
     ▼
   SUCCESS
     │
     ▼
 WAIT 3 sec
     │
     ▼
 Next Step
```

---

# 22. `parallel` Step

A `parallel` step starts multiple scenario branches simultaneously.

Example:

```yaml
- id: move_both_trucks
  type: parallel

  tasks:

    - id: truck1_parallel_route
      type: task
      robot: truck1
      task_type: waypoint
      task_file: truck1_waypoints2.yaml

    - id: truck3_parallel_route
      type: task
      robot: truck3
      task_type: waypoint
      task_file: truck3_waypoints.yaml
```

Execution:

```text
              PARALLEL START
                    │
          ┌─────────┴─────────┐
          ▼                   ▼
      Truck 1             Truck 3
          │                   │
          ▼                   ▼
       SUCCESS             SUCCESS
          │                   │
          └─────────┬─────────┘
                    ▼
             PARALLEL COMPLETE
                    │
                    ▼
                Next Step
```

The scenario continues after all parallel child tasks complete successfully.

---

# 23. `condition` Step

A `condition` waits for a ROS topic value to satisfy a specified condition.

The current Scenario Manager supports conditions using:

```text
std_msgs/String
construction_site_interfaces/msg/RobotStatus
```

This allows scenarios to synchronize robot execution using shared robot state.

For example:

```text
Truck 1 Action
      │
      ▼
Truck 1 navigating
      │
      ▼
Truck 1 Action SUCCESS
      │
      ▼
Check /truck1/status
      │
      ▼
state == completed
      │
      ▼
CONDITION SATISFIED
      │
      ▼
Truck 3 Action
```

A validated example scenario is:

```text
truck1_complete_then_truck3.yaml
```

---

# 24. `topic_publish` Step

A `topic_publish` step allows a scenario to publish directly to a ROS topic.

This is useful when an operation does not require a complete Action Server abstraction.

Examples include:

- direct `cmd_vel`
- actuator commands
- simple test commands
- integration testing

This provides a lightweight path for including direct ROS commands inside higher-level scenarios.

---

# 25. Manual `cmd_vel` Testing

Direct `cmd_vel` remains useful for validating motor direction, encoder mapping, and odometry.

## Straight Forward

```bash
ros2 topic pub -r 10 \
  /truck1/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}"
```

## Forward + Right

```bash
ros2 topic pub -r 10 \
  /truck5/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: -0.35}}"
```

For a right turn:

```text
logical LEFT wheel > logical RIGHT wheel
```

## In-Place Right Rotation

```bash
ros2 topic pub -r 10 \
  /truck5/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.70}}"
```

The truck should rotate clockwise and odometry yaw should change in the corresponding negative ROS yaw direction.

---

# 26. Manual Waypoint Execution

Waypoint files are stored under:

```text
operations/dump_truck/waypoints/
```

Run commands from the repository root.

Truck 1:

```bash
ros2 run dump_truck_control waypoint_controller_node \
  truck1 \
  operations/dump_truck/waypoints/truck1_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck1
```

Truck 3:

```bash
ros2 run dump_truck_control waypoint_controller_node \
  truck3 \
  operations/dump_truck/waypoints/truck3_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck3
```

Truck 4:

```bash
ros2 run dump_truck_control waypoint_controller_node \
  truck4 \
  operations/dump_truck/waypoints/truck4_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck4
```

Truck 5:

```bash
ros2 run dump_truck_control waypoint_controller_node \
  truck5 \
  operations/dump_truck/waypoints/truck5_waypoints.yaml \
  --ros-args \
  -r __ns:=/truck5
```

---

# 27. Runtime Logging

Command Center output can be saved while still being displayed normally using `tee`.

Create the runtime log directory:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

mkdir -p runtime_logs
```

Create a timestamped filename:

```bash
LOG_FILE="runtime_logs/command_center_$(date +%Y%m%d_%H%M%S).log"
```

Then launch the Command Center:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:="truck1,truck3,truck4,truck5" \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=true \
  scenario:=truck1_3_4_5.yaml \
  2>&1 | tee "$LOG_FILE"
```

Runtime logs are useful for reviewing:

```text
Scenario start
Task start
Action acceptance
Task success / failure
Conditions
Timeouts
Scenario completion
Errors
```

The `runtime_logs/` directory should not be committed.

Recommended `.gitignore` entry:

```gitignore
runtime_logs/
```

---

# 28. Review the Latest Runtime Log

Find the latest log:

```bash
LATEST_LOG=$(ls -t runtime_logs/command_center_*.log | head -1)

echo "$LATEST_LOG"
```

View it:

```bash
less "$LATEST_LOG"
```

Show Scenario Manager output only:

```bash
grep -F "[scenario_manager]" "$LATEST_LOG"
```

Show important scenario events:

```bash
grep -F "[scenario_manager]" "$LATEST_LOG" \
  | grep -E \
  "SCENARIO START|Goal accepted|: SUCCESS|CONDITION SATISFIED|CONDITION TIMEOUT|SCENARIO COMPLETE|SCENARIO ABORTED|SCENARIO ERROR"
```

Typical output:

```text
SCENARIO START
truck1_run: Goal accepted.
truck1_run: SUCCESS
verify_truck1_completed: CONDITION SATISFIED
truck3_run: Goal accepted.
truck3_run: SUCCESS
SCENARIO COMPLETE
```

---

# 29. Runtime Logs vs. ROS Bag

Runtime text logs and ROS bags serve different purposes.

## Runtime Text Log

Useful for:

```text
Scenario execution
Action execution
Conditions
Timeouts
Success / failure
Errors
```

## ROS Bag

Useful for time-series ROS data such as:

```text
/truck1/cmd_vel
/truck1/odom
/truck1/fused_odom
/truck1/status

/truck3/cmd_vel
/truck3/odom
/truck3/fused_odom
/truck3/status

/image_raw
/camera_info
```

Conceptually:

```text
Experiment
   │
   ├── Runtime text log
   │      └── system execution and errors
   │
   └── ROS bag
          └── ROS message time-series data
```

ROS bag recording can be added independently from the Command Center launch.

---

# 30. Verify the Running System

Check nodes:

```bash
ros2 node list | sort
```

Check Actions:

```bash
ros2 action list | sort
```

Check status topics:

```bash
ros2 topic list | grep status
```

Check all current truck topics:

```bash
ros2 topic list \
  | grep -E 'truck1|truck3|truck4|truck5' \
  | sort
```

Check wheel states:

```bash
ros2 topic echo /truck1/wheel_states
```

Check odometry:

```bash
ros2 topic echo /truck1/odom
```

Check fused odometry:

```bash
ros2 topic echo /truck1/fused_odom
```

Check status:

```bash
ros2 topic echo /truck1/status
```

Check `cmd_vel` connectivity:

```bash
ros2 topic info /truck1/cmd_vel -v
```

Check hardware parameters:

```bash
ros2 param dump /truck1/motor_drive
```

Check angular velocity:

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

# 31. Adding a New Dump Truck

For a new truck, for example Truck 6:

## 1. Add the Device

Update:

```text
network/devices.sh
```

Add the Raspberry Pi device entry.

---

## 2. Create Hardware Configuration

```bash
cp \
  robots/dump_truck/dump_truck_bringup/config/hardware/truck5.yaml \
  robots/dump_truck/dump_truck_bringup/config/hardware/truck6.yaml
```

Update:

- servo calibration
- `robot_tag_child_frame`
- tag yaw offset
- odometry scale factors
- encoder direction configuration
- encoder inversion
- `swap_encoders`

Do not assume encoder mapping is identical between trucks.

---

## 3. Create Waypoints

Create:

```text
operations/dump_truck/waypoints/truck6_waypoints.yaml
```

---

## 4. Update the Raspberry Pi

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

source install/setup.bash
```

---

## 5. Start Truck 6

```bash
sudo pigpiod
```

Then:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck6
```

---

## 6. Start the ROS PC Stack

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

## 7. Validate Encoder Mapping

```bash
ros2 topic echo /truck6/wheel_states
```

Then:

```bash
ros2 topic pub -r 10 \
  /truck6/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: -0.35}}"
```

For a correct right turn:

```text
logical LEFT wheel > logical RIGHT wheel
```

If the opposite occurs, inspect the truck-specific encoder configuration before modifying downstream odometry logic.

---

# 32. Current Verified Robots

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

---

# 33. Where New Files Belong

Use the following rule when extending the system:

```text
Robot hardware node
    → robots/dump_truck/dump_truck_hardware/

Robot control or navigation node
    → robots/dump_truck/dump_truck_control/

Launch file or truck configuration
    → robots/dump_truck/dump_truck_bringup/

Dump truck waypoint
    → operations/dump_truck/waypoints/

Construction scenario
    → operations/scenarios/

Shared camera or AprilTag configuration
    → perception/construction_robot_perception/

Dump truck Action logic
    → command_center/dump_truck_action_server/

Multi-robot coordination
    → command_center/construction_site_control/

Shared ROS message or Action
    → common/construction_site_interfaces/

ROS network configuration
    → network/
```

The general design rule is:

> Keep reusable robot software separate from operational data such as waypoint and scenario YAML files.