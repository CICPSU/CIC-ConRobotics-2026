# ROS 2 Action-Based Command Center

This directory contains the ROS 2 action-based command center for coordinating multiple construction robots.

The command center provides a higher-level control layer above the individual topic-based robot controllers. Its main purpose is to coordinate robot tasks, execute multi-robot scenarios, monitor robot states, support conditional execution, and reduce the number of ROS PC terminals required during experiments.

The current implementation has been validated with:

- Dump Truck 1 (`truck1`)
- Dump Truck 3 (`truck3`)
- Overhead camera localization using AprilTags
- ROS 2 Action-based task execution
- Sequential multi-robot execution
- Timed wait steps
- Parallel robot execution
- Conditional execution based on ROS topics
- Conditional execution based on shared robot status
- Direct ROS topic publishing
- Direct `cmd_vel` control
- Runtime log recording and scenario review

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
    │   ├── test_condition.yaml
    │   ├── test_topic_publish.yaml
    │   ├── test_truck1_cmd_vel.yaml
    │   ├── test_robot_status_condition.yaml
    │   └── truck1_complete_then_truck3.yaml
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

For the dump trucks:

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
     └── /truck3/execute_robot_task
                 │
                 ▼
          Truck 3 Action Server
                 │
                 ▼
          /truck3/cmd_vel
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

## 3. Initial Repository Setup

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

## 4. Initial Build

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

## 5. Network Setup

The network helper must be sourced before starting the ROS 2 system.

For the current Truck 1 + Truck 3 configuration on the ROS PC:

```bash
source network/setup_network.sh \
  ros_pc \
  dumptruck_01 \
  dumptruck_03
```

The ROS PC is intentionally specified explicitly.

This allows other ROS computers, such as:

```text
ros_pc_backup
```

to be introduced later without changing the overall network architecture.

---

## 6. Start Truck 1 Raspberry Pi

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

Start `pigpiod` if necessary:

```bash
sudo pigpiod
```

Then:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck1
```

---

## 7. Start Truck 3 Raspberry Pi

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

Start `pigpiod` if necessary:

```bash
sudo pigpiod
```

Then:

```bash
ros2 launch dump_truck_bringup dump_truck_pi.launch.py \
  truck_name:=truck3
```

---

## 8. Start the Command Center

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

Example:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:="truck1,truck3" \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=true \
  scenario:=truck1_complete_then_truck3.yaml
```

The launch command is intentionally parameterized.

The scenario, robot list, perception configuration, localization, and Action Server configuration can therefore be changed for each experiment without modifying source code.

This launch may start:

- overhead USB camera
- AprilTag detector
- Truck 1 odometry
- Truck 1 Tag/Odom fusion
- Truck 1 Action Server
- Truck 3 odometry
- Truck 3 Tag/Odom fusion
- Truck 3 Action Server
- Scenario Manager

---

## 9. Command Center Launch Parameters

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

Set this to:

```bash
start_scenario_manager:=false
```

when testing Action Servers manually.

### `scenario`

Scenario YAML file to execute.

Example:

```bash
scenario:=truck1_complete_then_truck3.yaml
```

---

## 10. Example Launch Configurations

### Truck 1 only without automatic scenario execution

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

### Truck 1 and Truck 3 without scenario execution

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:="truck1,truck3" \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

This configuration is useful for manually sending Action goals.

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
detail: Executing waypoint task: truck1_waypoints.yaml
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

With:

```bash
start_scenario_manager:=false
```

a task can be sent manually.

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

This mechanism is intended to support robot-to-robot synchronization.

Future examples include:

```text
/excavator1/status.state == waiting_for_truck
```

followed by:

```text
Truck 1 enters the loading area
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

## 20. Verified State-Based Multi-Robot Scenario

The following scenario has been validated with physical robots:

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
Check:
 /truck1/status.state
      ==
   completed
      │
      ▼
CONDITION SATISFIED
      │
      ▼
Truck 3 Action
      │
      ▼
Truck 3 SUCCESS
      │
      ▼
SCENARIO COMPLETE
```

The corresponding scenario is:

```text
truck1_complete_then_truck3.yaml
```

The runtime log confirmed:

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

## 21. Runtime Logging

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

Example:

```text
runtime_logs/command_center_20260812_141558.log
```

Launch the Command Center normally and append:

```bash
2>&1 | tee "$LOG_FILE"
```

Example:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:="truck1,truck3" \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=true \
  scenario:=truck1_complete_then_truck3.yaml \
  2>&1 | tee "$LOG_FILE"
```

The scenario configuration remains fully editable for each experiment.

Only the terminal output is redirected into the log file.

The `runtime_logs/` directory should not be committed to Git.

Recommended `.gitignore` entry:

```gitignore
runtime_logs/
```

---

## 22. Review the Latest Runtime Log

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

### Show the Important Scenario Events

```bash
grep -F "[scenario_manager]" "$LATEST_LOG" \
  | grep -E \
  "SCENARIO START|Goal accepted|: SUCCESS|CONDITION SATISFIED|CONDITION TIMEOUT|SCENARIO COMPLETE|SCENARIO ABORTED|SCENARIO ERROR"
```

Example:

```text
SCENARIO START
truck1_run: Goal accepted.
truck1_run: SUCCESS
verify_truck1_completed: CONDITION SATISFIED
truck3_run: Goal accepted.
truck3_run: SUCCESS
SCENARIO COMPLETE
```

This provides a concise summary of scenario execution without requiring the full camera, AprilTag, odometry, localization, and Action feedback logs to be reviewed manually.

---

## 23. Runtime Logs vs ROS Bag

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

Example:

```text
command_center_20260812_141558.log
```

### ROS Bag

ROS bag records ROS message data.

This is useful for later analysis of topics such as:

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

The intended experiment logging architecture is therefore:

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

## 24. Verify the Running System

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

Check robot status topics:

```bash
ros2 topic list | grep status
```

Examples:

```text
/truck1/status
/truck3/status
```

Inspect Truck 1 status:

```bash
ros2 topic echo /truck1/status
```

Example:

```yaml
robot_name: truck1
state: idle
detail: Waiting for task
```

---

## 25. Design Philosophy

The command center should remain independent from robot-specific low-level implementation as much as possible.

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
Hardware
```

The intended robot status naming convention is:

```text
/<robot_name>/status
```

For example:

```text
/truck1/status
/truck3/status
/excavator1/status
/excavator2/status
```

This allows future robot types to participate in construction scenarios using the same high-level coordination architecture.

The Command Center should not require detailed knowledge of the robot's internal low-level controller when a common Action and status interface is sufficient.

---

## 26. Current Scenario Capabilities

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

## 27. Future Extensions

Planned or possible extensions include:

- Excavator Action Servers
- `/excavator1/status`
- `/excavator2/status`
- Excavator / dump truck synchronization
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

---

## 28. Current Verified Configuration

The following physical multi-robot sequence has been verified:

```text
Truck 1 executes truck1_waypoints.yaml
              │
              ▼
           SUCCESS
              │
              ▼
 /truck1/status.state == completed
              │
              ▼
     CONDITION SATISFIED
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

The ROS PC side is launched through a single parameterized Command Center launch file.

Each Raspberry Pi independently runs its own robot hardware bringup.

The Command Center coordinates robot behavior at the task and construction-scenario level while maintaining compatibility with the existing topic-based robot control system.