# ROS 2 Action-Based Command Center

This directory contains the ROS 2 action-based command center for coordinating multiple construction robots.

The command center provides a higher-level control layer above the individual topic-based robot controllers. Its main purpose is to coordinate robot tasks, execute multi-robot scenarios, monitor robot states, support conditional execution, and reduce the number of ROS PC terminals required during experiments.

The current implementation has been validated with:

- Dump Truck 1 (`truck1`)
- Dump Truck 3 (`truck3`)
- Dump Truck 4 (`truck4`)
- Dump Truck 5 (`truck5`)
- Overhead camera localization using AprilTags
- Wheel-encoder odometry
- Tag/Odom fusion
- ROS 2 Action-based task execution
- Sequential multi-robot execution
- Four-truck physical sequence execution
- Timed wait steps
- Parallel robot execution
- Conditional execution based on ROS topics
- Conditional execution based on shared robot status
- Direct ROS topic publishing
- Direct `cmd_vel` control
- Runtime log recording and scenario review

The current physical system has successfully executed a four-truck scenario involving Trucks 1, 3, 4, and 5 through a single ROS PC Command Center.

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
│   ├── action/
│   │   └── ExecuteRobotTask.action
│   └── msg/
│       └── RobotStatus.msg
│
├── dump_truck_action_server/
│   └── Action interface for dump truck waypoint execution
│
└── construction_site_control/
    ├── launch/
    │   └── command_center.launch.py
    │
    ├── scenarios/
    │   ├── truck1_then_truck3.yaml
    │   ├── truck1_wait_truck3.yaml
    │   ├── truck1_truck3_parallel.yaml
    │   ├── truck1_complete_then_truck3.yaml
    │   ├── truck1_3_4_5.yaml
    │   ├── test_condition.yaml
    │   ├── test_topic_publish.yaml
    │   ├── test_truck1_cmd_vel.yaml
    │   └── test_robot_status_condition.yaml
    │
    └── construction_site_control/
        └── scenario_manager_node.py
```

The topic-based system remains available for:

- individual robot operation
- low-level testing
- debugging
- direct topic-based commands
- hardware verification

The Action-based command center is used for:

- task-level robot execution
- multi-robot coordination
- scenario execution
- state-based synchronization
- conditional execution
- direct ROS command execution when needed

---

## 2. Current Control Architecture

The current high-level control flow is:

```text
Scenario YAML
     │
     ▼
Scenario Manager
     │
     ├── Action task
     │      │
     │      ▼
     │   /<robot>/execute_robot_task
     │      │
     │      ▼
     │   Robot Action Server
     │      │
     │      ▼
     │   Existing topic-based control
     │
     ├── wait
     │
     ├── parallel
     │
     ├── condition
     │      │
     │      ▼
     │   ROS topic / RobotStatus
     │
     └── topic_publish
            │
            ▼
       Direct ROS topic command
```

For the four currently integrated dump trucks:

```text
Scenario Manager
     │
     ├── /truck1/execute_robot_task
     │           │
     │           ▼
     │    Truck 1 Action Server
     │           │
     │           ▼
     │    /truck1/cmd_vel
     │
     ├── /truck3/execute_robot_task
     │           │
     │           ▼
     │    Truck 3 Action Server
     │           │
     │           ▼
     │    /truck3/cmd_vel
     │
     ├── /truck4/execute_robot_task
     │           │
     │           ▼
     │    Truck 4 Action Server
     │           │
     │           ▼
     │    /truck4/cmd_vel
     │
     └── /truck5/execute_robot_task
                 │
                 ▼
          Truck 5 Action Server
                 │
                 ▼
          /truck5/cmd_vel
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
      ├── /truck3/fused_odom
      ├── /truck4/fused_odom
      └── /truck5/fused_odom
```

Each truck independently publishes wheel-based odometry:

```text
Truck Encoder Hardware
      │
      ▼
Wheel States
      │
      ▼
Odometry Node
      │
      ├── /truck1/odom
      ├── /truck3/odom
      ├── /truck4/odom
      └── /truck5/odom
```

---

## 3. Per-Truck Hardware Configuration

The dump truck software uses common ROS 2 nodes across multiple physical trucks.

Hardware-specific differences are handled through per-truck configuration files rather than separate source code for each robot.

The hardware configuration files are organized by truck, for example:

```text
config/
└── hardware/
    ├── truck1.yaml
    ├── truck3.yaml
    ├── truck4.yaml
    └── truck5.yaml
```

These configuration files can contain robot-specific parameters such as:

- bucket servo center position
- bucket servo dump position
- AprilTag ID
- Tag yaw offset
- odometry scale parameters
- encoder direction mode
- encoder glitch filtering
- encoder sign inversion
- physical left/right encoder mapping

Example structure:

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

Physical encoder installation can differ between trucks.

The common `motor_drive_node` therefore supports truck-specific encoder configuration through the hardware YAML.

Important encoder-related parameters include:

- `encoder_direction_mode`
- `encoder_glitch_filter_us`
- `left_encoder_invert`
- `right_encoder_invert`
- `swap_encoders`

The `swap_encoders` parameter maps the physical encoder GPIO channels to the logical left and right robot wheels.

The currently verified configuration is:

```text
Truck 1: swap_encoders = false
Truck 3: swap_encoders = false
Truck 4: swap_encoders = true
Truck 5: swap_encoders = true
---

For Trucks 4 and 5, the physical encoder left/right mapping is opposite to the logical robot wheel mapping used by the ROS controller.
We will need to fix this later from the hardware.

When swap_encoders: true, the motor-drive implementation corrects both:

the left/right encoder values published in /truckX/wheel_states
the commanded encoder direction mapping used when determining encoder sign

This is necessary because correcting only the published left/right values can make basic turning appear correct while still causing unstable odometry during steering corrections.

The final configuration allows all four trucks to use the same common ROS 2 hardware and odometry implementation while accounting for physical hardware differences through YAML configuration.



## 4. Initial Repository Setup

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

## 5. Initial Build

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

### Rebuild After Code or Configuration Updates

When pulling updated ROS packages or modifying the truck hardware / bringup implementation, rebuild the relevant packages before launching the system.

For dump truck Raspberry Pis:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    dump_truck_hardware \
    dump_truck_bringup

source install/setup.bash
```

For the ROS PC, rebuild the affected packages as necessary before starting the Command Center.

---

## 6. Network Setup

The network helper must be sourced before starting the ROS 2 system.

For the current four-truck configuration on the ROS PC:

```bash
source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03 \
  dumptruck_04 \
  dumptruck_05
```

The ROS PC is intentionally specified explicitly.

This allows other ROS computers, such as:

```text
ros_pc_backup
```

to be introduced later without changing the overall network architecture.

All computers participating in the experiment must use compatible ROS 2 network settings.

---

## 7. Start the Raspberry Pi on Each Truck

Each truck independently runs its own hardware bringup.

Before launching a truck, enter the repository and source ROS 2:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Source the network configuration:

```bash
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

### Truck 1

On the Truck 1 Raspberry Pi:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck1
```

### Truck 3

On the Truck 3 Raspberry Pi:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck3
```

### Truck 4

On the Truck 4 Raspberry Pi:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck4
```

### Truck 5

On the Truck 5 Raspberry Pi:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck5
```

Each truck should publish its own namespaced hardware topics, including:

```text
/truck1/cmd_vel
/truck1/wheel_states

/truck3/cmd_vel
/truck3/wheel_states

/truck4/cmd_vel
/truck4/wheel_states

/truck5/cmd_vel
/truck5/wheel_states
```

---

## 8. Start the Four-Truck Command Center

On the ROS PC:

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

To launch the complete four-truck system and automatically execute the verified four-truck scenario:

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

- overhead USB camera
- AprilTag detector
- Truck 1 odometry
- Truck 1 Tag/Odom fusion
- Truck 1 Action Server
- Truck 3 odometry
- Truck 3 Tag/Odom fusion
- Truck 3 Action Server
- Truck 4 odometry
- Truck 4 Tag/Odom fusion
- Truck 4 Action Server
- Truck 5 odometry
- Truck 5 Tag/Odom fusion
- Truck 5 Action Server
- Scenario Manager

The four Raspberry Pis continue to independently handle their respective physical robot hardware.

---

## 9. Start the Command Center Without a Scenario

For testing localization, Action Servers, or individual robots without automatically executing a scenario:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:="truck1,truck3,truck4,truck5" \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

This is particularly useful for:

- checking AprilTag localization
- checking odometry
- manually testing Actions
- testing `cmd_vel`
- verifying newly added trucks
- debugging individual robot behavior

---

## 10. Command Center Launch Parameters

### `trucks`

Comma-separated list of dump trucks.

Four-truck example:

```bash
trucks:="truck1,truck3,truck4,truck5"
```

Two trucks:

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

Set this to:

```bash
start_scenario_manager:=false
```

when testing Action Servers or individual robots manually.

### `scenario`

Scenario YAML file to execute.

Four-truck example:

```bash
scenario:=truck1_3_4_5.yaml
```

Two-truck example:

```bash
scenario:=truck1_complete_then_truck3.yaml
```

---

## 11. ROS 2 Action Interface

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

Each robot exposes its own Action interface.

Currently:

```text
/truck1/execute_robot_task
/truck3/execute_robot_task
/truck4/execute_robot_task
/truck5/execute_robot_task
```

Check available Actions with:

```bash
ros2 action list | sort
```

---

## 12. Shared Robot Status Interface

A shared robot status message is defined as:

```text
construction_site_interfaces/msg/RobotStatus.msg
```

Current definition:

```text
string robot_name
string state
string detail
```

Current dump truck status topics include:

```text
/truck1/status
/truck3/status
/truck4/status
/truck5/status
```

Future robots should follow the same convention:

```text
/excavator1/status
/excavator2/status
```

The general naming convention is:

```text
/<robot_name>/status
```

Example message:

```yaml
robot_name: truck1
state: navigating
detail: Executing waypoint task
```

Common states currently include:

```text
idle
waiting
navigating
performing_action
completed
fault
```

The status publisher periodically republishes the current state so that monitoring and coordination nodes can determine the current robot state.

---

## 13. Manual Action Test

Start the Command Center with:

```bash
start_scenario_manager:=false
```

Then tasks can be sent manually.

### Truck 1

```bash
ros2 action send_goal \
  /truck1/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck1, task_type: waypoint, task_file: truck1_waypoints.yaml}" \
  --feedback
```

### Truck 3

```bash
ros2 action send_goal \
  /truck3/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck3, task_type: waypoint, task_file: truck3_waypoints.yaml}" \
  --feedback
```

Truck 4 and Truck 5 follow the same Action interface and naming convention.

---

## 14. Scenario YAML

Scenarios are stored under:

```text
ros2_action_based_command_center/
└── construction_site_control/
    └── scenarios/
```

The Scenario Manager currently supports the following step types:

```text
task
wait
parallel
condition
topic_publish
```

The current verified four-truck scenario is:

```text
truck1_3_4_5.yaml
```

---

## 15. Task Step

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

## 16. Wait Step

A `wait` step introduces a timed delay.

Example:

```yaml
- id: wait_after_truck1
  type: wait
  duration: 3.0
```

Example flow:

```text
Truck 1
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

## 17. Parallel Step

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

Execution behavior:

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

The scenario continues only after all parallel child tasks complete successfully.

---

## 18. Condition Step

A `condition` waits for a ROS topic value to match a specified condition.

The Scenario Manager currently supports:

- `std_msgs/String`
- `construction_site_interfaces/msg/RobotStatus`

### String Condition

Example:

```yaml
- id: wait_for_ready
  type: condition

  condition:
    source: topic
    topic: /test_condition
    msg_type: std_msgs/String
    equals: ready
    timeout: 10.0

  then:

    - id: success_wait
      type: wait
      duration: 2.0

  else:

    - id: timeout_wait
      type: wait
      duration: 5.0
```

### RobotStatus Condition

Example:

```yaml
- id: wait_for_truck1_completed
  type: condition

  condition:
    source: topic
    topic: /truck1/status
    msg_type: construction_site_interfaces/msg/RobotStatus
    field: state
    equals: completed
    timeout: 10.0

  then:

    - id: continue_work
      type: wait
      duration: 1.0

  else:

    - id: completion_timeout
      type: wait
      duration: 2.0
```

This mechanism supports robot-to-robot synchronization.

Future examples include:

```text
/excavator1/status.state == waiting_for_truck
```

followed by:

```text
Truck enters the loading area
```

or:

```text
/truck1/status.state == waiting_for_loading
```

followed by:

```text
Excavator begins loading
```

---

## 19. Direct Topic Publishing

The Scenario Manager can publish directly to ROS topics when low-level or experimental commands are required.

Supported message types currently include:

```text
std_msgs/String
geometry_msgs/Twist
```

### String Example

```yaml
- id: publish_test_message
  type: topic_publish
  topic: /test_command
  msg_type: std_msgs/String

  message:
    data: hello_from_command_center
```

### Direct `cmd_vel` Example

```yaml
- id: direct_move_truck1
  type: topic_publish
  topic: /truck1/cmd_vel
  msg_type: geometry_msgs/Twist
  duration: 1.0
  rate_hz: 10.0

  message:

    linear:
      x: 0.10
      y: 0.0
      z: 0.0

    angular:
      x: 0.0
      y: 0.0
      z: 0.0
```

For timed `Twist` commands, the Scenario Manager automatically publishes a zero `Twist` when the specified duration ends.

This provides a safety stop after direct motion commands.

Direct topic commands are rejected when the same robot already has an active Action task managed by the Scenario Manager.

---

## 20. Verified Four-Truck Physical Sequence

The current system has been physically validated with all four integrated dump trucks.

The verified scenario file is:

```text
truck1_3_4_5.yaml
```

The scenario executes the trucks sequentially through the ROS 2 Action-based Command Center:

```text
SCENARIO START
      │
      ▼
   Truck 1
      │
      ▼
   SUCCESS
      │
      ▼
   Truck 3
      │
      ▼
   SUCCESS
      │
      ▼
   Truck 4
      │
      ▼
   SUCCESS
      │
      ▼
   Truck 5
      │
      ▼
   SUCCESS
      │
      ▼
SCENARIO COMPLETE
```

All four physical trucks were successfully integrated into the same Command Center architecture and executed their assigned sequence.

This verifies:

- communication between the ROS PC and four Raspberry Pis
- namespaced robot hardware interfaces
- per-truck hardware configuration
- four independent wheel-encoder interfaces
- wheel-based odometry
- overhead AprilTag localization
- Tag/Odom fusion
- per-robot Action Servers
- waypoint-based navigation
- Scenario Manager execution across four physical robots

---

## 21. Encoder and Odometry Configuration

During multi-truck integration, differences in physical encoder installation were identified between trucks.

Trucks 4 and 5 required different left/right encoder mapping from Trucks 1 and 3.

The software architecture therefore keeps the odometry implementation common while allowing physical hardware differences to be specified through per-truck configuration.

The general data flow is:

```text
Physical Encoders
      │
      ▼
Truck-Specific Hardware Mapping
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
```

This approach avoids creating separate odometry implementations for each truck.

Correct encoder mapping is important because incorrect left/right interpretation can cause:

- incorrect yaw direction
- continuous turning behavior
- oscillation after turns
- disagreement between wheel odometry and AprilTag localization
- unstable waypoint tracking

The current configuration has been physically tested on Trucks 1, 3, 4, and 5.

---

## 22. Runtime Logging

Command Center output can be saved while still displaying normally in the terminal using `tee`.

Create the runtime log directory:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

mkdir -p runtime_logs
```

Create a timestamped log filename:

```bash
LOG_FILE="runtime_logs/command_center_$(date +%Y%m%d_%H%M%S).log"
```

Launch the four-truck Command Center and append:

```bash
2>&1 | tee "$LOG_FILE"
```

Full example:

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

The scenario configuration remains fully editable for each experiment.

Only terminal output is redirected into the log file.

The `runtime_logs/` directory should not be committed to Git.

Recommended `.gitignore` entry:

```gitignore
runtime_logs/
```

---

## 23. Review the Latest Runtime Log

To find the latest Command Center log:

```bash
LATEST_LOG=$(ls -t runtime_logs/command_center_*.log | head -1)

echo "$LATEST_LOG"
```

View the entire log:

```bash
less "$LATEST_LOG"
```

### Show Only Scenario Manager Output

```bash
grep -F "[scenario_manager]" "$LATEST_LOG"
```

### Show Important Scenario Events

```bash
grep -F "[scenario_manager]" "$LATEST_LOG" \
  | grep -E \
  "SCENARIO START|Goal accepted|: SUCCESS|CONDITION SATISFIED|CONDITION TIMEOUT|SCENARIO COMPLETE|SCENARIO ABORTED|SCENARIO ERROR"
```

This provides a concise summary of scenario execution without requiring the full camera, AprilTag, odometry, localization, and Action feedback logs to be reviewed manually.

---

## 24. Runtime Logs vs ROS Bag

Runtime text logs and ROS bags serve different purposes.

### Runtime Text Log

The Command Center text log records human-readable system execution information.

Useful for reviewing:

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

### ROS Bag

ROS bag records ROS message data.

For the current four-truck system, useful topics include:

```text
/truck1/cmd_vel
/truck1/odom
/truck1/fused_odom
/truck1/status

/truck3/cmd_vel
/truck3/odom
/truck3/fused_odom
/truck3/status

/truck4/cmd_vel
/truck4/odom
/truck4/fused_odom
/truck4/status

/truck5/cmd_vel
/truck5/odom
/truck5/fused_odom
/truck5/status

/image_raw
/camera_info
```

The intended experiment logging architecture is:

```text
Experiment
   │
   ├── Human-readable runtime log
   │      └── scenario execution and errors
   │
   └── ROS bag
          └── ROS message time-series data
```

ROS bag recording can be added independently from the Command Center launch configuration.

---

## 25. Verify the Running Four-Truck System

Check nodes:

```bash
ros2 node list | sort
```

The ROS PC should include nodes similar to:

```text
/apriltag
/usb_cam

/truck1/odometry
/truck1/tag_odom_fusion
/truck1/waypoint_action_server

/truck3/odometry
/truck3/tag_odom_fusion
/truck3/waypoint_action_server

/truck4/odometry
/truck4/tag_odom_fusion
/truck4/waypoint_action_server

/truck5/odometry
/truck5/tag_odom_fusion
/truck5/waypoint_action_server
```

Check Actions:

```bash
ros2 action list | sort
```

Expected:

```text
/truck1/execute_robot_task
/truck3/execute_robot_task
/truck4/execute_robot_task
/truck5/execute_robot_task
```

Check fused odometry:

```bash
ros2 topic list | grep fused_odom
```

Expected:

```text
/truck1/fused_odom
/truck3/fused_odom
/truck4/fused_odom
/truck5/fused_odom
```

Check robot status topics:

```bash
ros2 topic list | grep status
```

Expected:

```text
/truck1/status
/truck3/status
/truck4/status
/truck5/status
```

Check wheel-state topics:

```bash
ros2 topic list | grep wheel_states
```

Expected:

```text
/truck1/wheel_states
/truck3/wheel_states
/truck4/wheel_states
/truck5/wheel_states
```

---

## 26. Basic Troubleshooting

If a truck does not move even though ROS topics are visible, first verify that its `cmd_vel` topic has the correct subscriber.

Example for Truck 1:

```bash
ros2 topic info /truck1/cmd_vel -v
```

Check the command:

```bash
ros2 topic echo /truck1/cmd_vel
```

Check wheel encoder output:

```bash
ros2 topic echo /truck1/wheel_states
```

Check odometry:

```bash
ros2 topic echo /truck1/odom
```

Check fused localization:

```bash
ros2 topic echo /truck1/fused_odom
```

For a quick manual forward-motion test:

```bash
ros2 topic pub -r 10 /truck1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.0}}"
```

For a rotation test:

```bash
ros2 topic pub -r 10 /truck1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

When diagnosing odometry, verify that the reported yaw changes in the same direction as the physical robot.

Incorrect left/right encoder mapping can produce physically plausible wheel counts while still causing incorrect odometry and unstable waypoint control.

---

## 27. Design Philosophy

The Command Center should remain independent from robot-specific low-level implementation as much as possible.

The intended hierarchy is:

```text
Construction Scenario
        │
        ▼
Scenario Manager
        │
        ├── Action
        ├── Condition
        ├── Parallel
        ├── Wait
        └── Direct Topic Command
        │
        ▼
Common Robot Interface
        │
        ├── ExecuteRobotTask
        └── RobotStatus
        │
        ▼
Robot-Specific Controller
        │
        ▼
Per-Robot Hardware Configuration
        │
        ▼
Physical Hardware
```

The intended robot status naming convention is:

```text
/<robot_name>/status
```

For example:

```text
/truck1/status
/truck3/status
/truck4/status
/truck5/status
/excavator1/status
/excavator2/status
```

This allows future robot types to participate in construction scenarios using the same high-level coordination architecture.

The Command Center should not require detailed knowledge of the robot's internal low-level controller when a common Action and status interface is sufficient.

Similarly, physical differences between robots of the same type should preferably be represented through configuration rather than duplicated controller implementations.

---

## 28. Current Scenario Capabilities

The current Scenario Manager supports:

```text
task
wait
parallel
condition
topic_publish
```

These enable sequential execution:

```text
Truck 1
↓
Truck 3
↓
Truck 4
↓
Truck 5
```

Timed coordination:

```text
Truck 1
↓
WAIT
↓
Truck 3
```

Parallel execution:

```text
Truck 1 ─────┐
             ├── continue after both finish
Truck 3 ─────┘
```

State-based coordination:

```text
Truck 1 completed
↓
Condition satisfied
↓
Truck 3 starts
```

Future excavation coordination:

```text
Excavator 1 digging
        │
        ▼
waiting_for_truck
        │
        ▼
Truck 1 enters loading area
        │
        ▼
Excavator 1 loads Truck 1
        │
        ▼
Truck 1 leaves
        │
        ▼
Truck 3 enters loading area
```

---

## 29. Future Extensions

Planned or possible extensions include:

- Excavator Action Servers
- `/excavator1/status`
- `/excavator2/status`
- Excavator / dump truck synchronization
- heterogeneous multi-robot scenarios
- robot event topics
- service-call scenario steps
- parameter-setting scenario steps
- scenario pause / resume / cancel
- abnormal-condition handling
- fault recovery branches
- localization-ready conditions
- task timeout handling
- richer robot status messages
- ROS bag experiment recording
- automatic experiment metadata recording
- additional construction robot types

The current four-dump-truck implementation provides the foundation for extending the same Command Center architecture to heterogeneous construction equipment.

---

## 30. Current Verified Configuration

The current physical system consists of:

```text
ROS PC
  │
  ├── Overhead Camera
  ├── AprilTag Detection
  ├── Odometry / Tag Fusion
  ├── Dump Truck Action Servers
  └── Scenario Manager
          │
          ├── Truck 1 Raspberry Pi
          ├── Truck 3 Raspberry Pi
          ├── Truck 4 Raspberry Pi
          └── Truck 5 Raspberry Pi
```

The following physical four-truck sequence has been successfully verified:

```text
truck1_3_4_5.yaml

SCENARIO START
      │
      ▼
   TRUCK 1
      │
      ▼
   SUCCESS
      │
      ▼
   TRUCK 3
      │
      ▼
   SUCCESS
      │
      ▼
   TRUCK 4
      │
      ▼
   SUCCESS
      │
      ▼
   TRUCK 5
      │
      ▼
   SUCCESS
      │
      ▼
SCENARIO COMPLETE
```

The ROS PC side is launched through a single parameterized Command Center launch file:

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

Each Raspberry Pi independently runs its own robot hardware bringup using the appropriate truck-specific configuration.

The Command Center coordinates robot behavior at the task and construction-scenario level while maintaining compatibility with the existing topic-based robot control system.

As of the current verified implementation, Trucks 1, 3, 4, and 5 can participate in a common ROS 2 Action-based multi-robot construction scenario using shared software architecture and per-robot hardware configuration.