# Construction Robotics Command Center

The `command_center` directory contains the ROS 2 components used to coordinate multiple construction robots in the CIC ConRobotics system.

The Command Center provides a higher-level execution layer above individual robot controllers. It allows construction operations to be described as YAML scenarios and executed as sequences of robot tasks, excavator trajectories, waits, topic commands, parallel operations, and conditions.

The current system supports:

- Dump truck waypoint tasks using `ExecuteRobotTask`
- Excavator joint trajectories using `FollowJointTrajectory`
- Sequential multi-robot scenarios
- Parallel scenario steps
- Conditional execution based on ROS 2 topics
- Direct topic publishing
- Software-only testing using a mock dump truck and simulated excavator
- Physical dump truck operation
- Physical excavator communication
- Multi-machine ROS 2 communication using Zenoh

---

# System Architecture

```text
                         Scenario YAML
                              │
                              ▼
                     Scenario Manager
                              │
              ┌───────────────┴───────────────┐
              │                               │
              ▼                               ▼
      ExecuteRobotTask              FollowJointTrajectory
              │                               │
              ▼                               ▼
        Dump Truck                     Excavator
              │                               │
        Physical / Mock                  Physical / SIM
```

The Command Center does not directly control motors.

Instead, it sends higher-level commands to robot-specific ROS 2 interfaces.

For physical multi-machine operation, the Command Center communicates with robot computers through a shared Zenoh router.

```text
                            ROS PC
                               │
                        Zenoh Router
                               │
             ┌─────────────────┼─────────────────┐
             │                 │                 │
             ▼                 ▼                 ▼
         Dump Truck        Dump Truck        Excavator
             Pi                Pi                Pi
```

---

# Directory Structure

```text
command_center/
├── construction_site_control/
│   ├── construction_site_control/
│   │   ├── scenario_manager_node.py
│   │   └── excavator_task_client.py
│   │
│   ├── launch/
│   │   └── command_center.launch.py
│   │
│   ├── package.xml
│   └── setup.py
│
└── dump_truck_action_server/
    ├── dump_truck_action_server/
    │   ├── waypoint_action_server_node.py
    │   └── mock_waypoint_action_server_node.py
    │
    ├── package.xml
    └── setup.py
```

Operational files are stored separately from the ROS 2 packages:

```text
operations/
├── dump_truck/
│   └── waypoints/
│
├── excavator/
│   └── trajectories/
│
└── scenarios/
```

This separates reusable ROS 2 software from site- and operation-specific task definitions.

---

# 1. Scenario Manager

The main Command Center node is:

```bash
ros2 run construction_site_control scenario_manager_node
```

The Scenario Manager reads a YAML scenario and executes its steps in order.

Currently supported step types include:

```text
task
excavator_trajectory
wait
topic_publish
parallel
condition
```

A scenario can therefore coordinate different types of construction robots without requiring them to use the same low-level controller.

---

# 2. Dump Truck Tasks

Dump trucks use the custom ROS 2 Action:

```text
construction_site_interfaces/action/ExecuteRobotTask
```

The Action Server for each truck is:

```text
/<truck_name>/execute_robot_task
```

For example:

```text
/truck1/execute_robot_task
```

A dump truck scenario step looks like:

```yaml
- id: truck1_route
  type: task
  robot: truck1
  task_type: waypoint
  task_file: truck1_waypoints.yaml
```

Waypoint files are stored under:

```text
operations/dump_truck/waypoints/
```

The real Action Server uses the truck localization and control stack to execute the requested waypoint file.

A mock Action Server is also available for software-only integration testing.

---

# 3. Excavator Tasks

The excavator uses the standard ROS 2:

```text
control_msgs/action/FollowJointTrajectory
```

Action interface:

```text
/upper_arm_controller/follow_joint_trajectory
```

An excavator scenario step looks like:

```yaml
- id: excavator_load
  type: excavator_trajectory
  robot: excavator1
  task_file: boom_small_test.yaml
  seconds_per_waypoint: 2.0
```

Trajectory files are stored under:

```text
operations/excavator/trajectories/
```

Excavator trajectories may command all joints or only a subset of joints.

For example, a boom-only trajectory can contain:

```yaml
trajectory_name: boom_small_test

joints:
  - boom

waypoints:
  - name: boom_target
    positions:
      boom: -14.3
```

This allows individual joints to be tested without unintentionally commanding the other excavator joints.

---

# 4. Mixed-Robot Scenarios

A single scenario can coordinate both dump trucks and the excavator.

Example:

```yaml
scenario_name: truck1_excavator_cycle

steps:
  - id: truck1_arrive
    type: task
    robot: truck1
    task_type: waypoint
    task_file: truck1_waypoints3.yaml

  - id: excavator_load
    type: excavator_trajectory
    robot: excavator1
    task_file: boom_small_test.yaml
    seconds_per_waypoint: 2.0

  - id: truck1_depart
    type: task
    robot: truck1
    task_type: waypoint
    task_file: truck1_waypoints.yaml
```

Because scenario steps are sequential by default, the next step begins only after the previous Action has completed successfully.

Therefore, the example above represents:

```text
Truck 1 arrives
      │
      ▼
Excavator operates
      │
      ▼
Truck 1 departs
```

---

# 5. Software-Only Integration Testing

The Command Center can be tested without physical robots.

This is the recommended development workflow before testing physical hardware.

The current software-only configuration uses:

```text
Dump Truck  → Mock Action Server

Excavator   → SIM mode
```

This allows the complete scenario orchestration layer to be tested independently of sensors, motors, Raspberry Pis, and physical calibration.

---

# 6. Build

From the repository root:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    construction_site_interfaces \
    dump_truck_action_server \
    excavator_control \
    construction_site_control

source install/setup.bash
```

A setuptools warning related to `pytest-repeat` may appear during the build. If the packages finish successfully, this warning does not indicate a build failure.

---

# 7. Start the Excavator in SIM Mode

**Terminal 1**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch excavator_control \
  excavator.launch.py \
  mode:=sim
```

The excavator server should report:

```text
[SIM MODE] ExcavatorTrajectoryServer ready
```

The Action interface should be:

```text
/upper_arm_controller/follow_joint_trajectory
```

---

# 8. Start a Mock Dump Truck

**Terminal 2**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run dump_truck_action_server \
  mock_waypoint_action_server_node \
  --ros-args \
  -p truck_name:=truck1 \
  -p seconds_per_waypoint:=0.5
```

The mock server provides:

```text
/truck1/execute_robot_task
```

and publishes status on:

```text
/truck1/status
```

The mock server loads the real waypoint YAML but does not command motors or require odometry.

It simulates completion of each waypoint and returns a successful `ExecuteRobotTask` result.

---

# 9. Run a Mixed-Robot Software Scenario

**Terminal 3**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run construction_site_control \
  scenario_manager_node \
  --ros-args \
  -p scenario:=truck1_excavator_cycle.yaml \
  -p trucks:=truck1
```

The expected execution sequence is:

```text
SCENARIO START
      │
      ▼
truck1_arrive
      │
      ▼
Dump Truck Mock
      │
      ▼
SUCCESS
      │
      ▼
excavator_load
      │
      ▼
Excavator SIM
      │
      ▼
EXCAVATOR SUCCESS
      │
      ▼
truck1_depart
      │
      ▼
Dump Truck Mock
      │
      ▼
SUCCESS
      │
      ▼
SCENARIO COMPLETE
```

This workflow has been successfully tested as a software-only mixed-robot integration test.

---

# 10. Excavator-Only Scenario Test

An excavator-only scenario is also available:

```text
operations/scenarios/excavator_sim_test.yaml
```

Run:

```bash
ros2 run construction_site_control \
  scenario_manager_node \
  --ros-args \
  -p scenario:=excavator_sim_test.yaml
```

This is useful for testing:

```text
Scenario YAML
      │
      ▼
Scenario Manager
      │
      ▼
FollowJointTrajectory
      │
      ▼
Excavator SIM
```

without starting any dump truck components.

---

# 11. Direct Excavator Command Center Test

The Command Center also contains a standalone excavator Action client:

```bash
ros2 run construction_site_control \
  excavator_task_client \
  boom_small_test.yaml \
  --seconds-per-waypoint 2.0
```

This bypasses the Scenario Manager and is useful for isolating the interface between the Command Center and `excavator_control`.

The execution path is:

```text
Excavator Trajectory YAML
          │
          ▼
excavator_task_client
          │
          ▼
FollowJointTrajectory
          │
          ▼
excavator_control
```

---

# 12. Physical Robot Operation

The same high-level interfaces are used for both simulation and physical operation.

For the excavator:

```text
Command Center
      │
      │ FollowJointTrajectory
      ▼
excavator_control
      │
      ├── SIM mode
      │
      └── PI mode
```

For the dump trucks:

```text
Command Center
      │
      │ ExecuteRobotTask
      ▼
dump_truck_action_server
      │
      ▼
dump_truck_control
      │
      ▼
Physical Dump Truck
```

The purpose of this architecture is to keep scenario definitions independent from the underlying hardware implementation.

---

# 13. ROS 2 Network Architecture

Zenoh (`rmw_zenoh_cpp`) is the primary ROS 2 communication method for physical multi-machine operation.

The normal network topology is:

```text
                              ROS PC
                                 │
                            Zenoh Router
                                 │
              ┌──────────────────┼──────────────────┐
              │                  │                  │
              ▼                  ▼                  ▼
          Dumptruck1         Dumptruck3        Excavator1
              Pi                 Pi                 Pi
```

The system uses:

```text
ONE Zenoh Router
      +
ONE Command Center
      +
N Physical Robot Clients
```

The robot Raspberry Pis do not need to be configured as direct ROS peers of one another.

Each robot connects independently to the router running on the ROS PC.

Network configuration is managed through:

```text
network/
├── devices.sh
├── setup_zenoh.sh
└── setup_network.sh
```

The files have the following roles:

```text
devices.sh
    Central device-name and IP registry

setup_zenoh.sh
    Primary Zenoh configuration

setup_network.sh
    DDS-based fallback configuration
```

Detailed network instructions are provided in:

```text
network/README.md
```

---

# 14. Standard Physical System Startup

The recommended physical system uses only two primary ROS PC terminals.

```text
ROS PC
├── T1 → Zenoh Router
└── T2 → Command Center

Robot Raspberry Pis
└── T1 → Robot Hardware
```

---

## ROS PC T1 — Zenoh Router

**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router

ros2 run rmw_zenoh_cpp rmw_zenohd
```

Only one router is required.

Do not start one router per robot.

---

## Robot Raspberry Pi

Each physical robot independently connects to the same router.

Example for Truck 1:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

sudo pigpiod

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

Example for Excavator 1:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client excavator1

sudo pigpiod

ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi
```

Additional dump trucks use their corresponding device profiles.

For example:

```bash
source network/setup_zenoh.sh client dumptruck3
```

```bash
source network/setup_zenoh.sh client dumptruck4
```

```bash
source network/setup_zenoh.sh client dumptruck5
```

---

## ROS PC T2 — Command Center

**Keep this terminal running.**

Example for Truck 1:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc

ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

Example for multiple dump trucks:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1,truck3,truck4,truck5 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

This allows the ROS PC to start:

```text
Overhead Camera
AprilTag Detection
Dump Truck Odometry
Tag/Odom Fusion
Dump Truck Action Servers
```

while all selected physical trucks communicate through the shared Zenoh router.

---

# 15. Multi-Robot Communication

Adding more robots does not require additional ROS PC routers.

For example:

```text
ROS PC
│
├── Zenoh Router
│
└── Command Center
       │
       ├── Truck 1
       ├── Truck 3
       ├── Truck 4
       ├── Truck 5
       └── Excavator 1
```

Each physical robot runs its own hardware-side process.

The high-level Command Center remains centralized on the ROS PC.

Conceptually:

```text
                      Scenario Manager
                            │
             ┌──────────────┴──────────────┐
             │                             │
             ▼                             ▼
      Dump Truck Actions           Excavator Action
             │                             │
             ▼                             ▼
       Zenoh Router                  Zenoh Router
             │                             │
             └──────────────┬──────────────┘
                            ▼
                    Physical Robots
```

Both robot types share the same ROS 2 communication backbone.

---

# 16. Current Communication Status

The current project-wide physical communication configuration uses:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Normal operation should use:

```bash
source network/setup_zenoh.sh ...
```

rather than manually exporting middleware configuration.

Zenoh has been validated for:

- ROS 2 node and topic discovery
- dump truck communication
- dump truck `/wheel_states`
- dump truck `/cmd_vel`
- overhead camera communication
- AprilTag detection messages
- compressed camera image transport
- excavator `/joint_states`
- excavator `FollowJointTrajectory` Action discovery
- excavator `FollowJointTrajectory` Action payload transport
- Command Center integration
- physical dump truck end-to-end operation

The physical dump truck stack has been successfully operated through the following path:

```text
Overhead Camera
      │
      ▼
AprilTag Detection
      │
      ▼
Localization / Fused Odom
      │
      ▼
Waypoint / Action Control
      │
      ▼
/<truck>/cmd_vel
      │
      ▼
Dump Truck Raspberry Pi
      │
      ▼
Physical Robot Motion
```

Zenoh is therefore the standard communication layer for the integrated system.

---

# 17. Previous DDS Testing

Earlier system integration work evaluated DDS-based communication.

CycloneDDS successfully transported excavator ROS 2 topics and `FollowJointTrajectory` Actions during testing.

Other DDS configurations showed communication or library compatibility issues on some machines.

These results remain useful for development history and troubleshooting, but CycloneDDS-specific configuration is no longer the standard startup method.

The previous DDS helper remains available at:

```text
network/setup_network.sh
```

for fallback and future communication testing.

---

# 18. Running a Scenario Automatically

The Command Center can start the Scenario Manager automatically.

Example:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1,truck3,truck4,truck5 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=true \
  scenario:=truck1_3_4_5.yaml
```

Scenario files are resolved from:

```text
operations/scenarios/
```

It is generally preferable to first start the system with:

```bash
start_scenario_manager:=false
```

verify communication and robot state, and then execute a scenario.

---

# 19. Scenario Step Types

The Scenario Manager currently supports:

```text
task
excavator_trajectory
wait
parallel
condition
topic_publish
```

---

## `task`

Executes a robot task through `ExecuteRobotTask`.

Example:

```yaml
- id: truck1_route
  type: task
  robot: truck1
  task_type: waypoint
  task_file: truck1_waypoints.yaml
```

---

## `excavator_trajectory`

Executes an excavator trajectory through `FollowJointTrajectory`.

Example:

```yaml
- id: excavator_load
  type: excavator_trajectory
  robot: excavator1
  task_file: boom_small_test.yaml
  seconds_per_waypoint: 2.0
```

---

## `wait`

Introduces a timed delay.

Example:

```yaml
- id: wait_after_truck
  type: wait
  duration: 3.0
```

---

## `parallel`

Runs multiple child steps simultaneously.

Example:

```yaml
- id: parallel_move
  type: parallel
  tasks:
    - id: truck1_move
      type: task
      robot: truck1
      task_type: waypoint
      task_file: truck1_waypoints.yaml

    - id: truck3_move
      type: task
      robot: truck3
      task_type: waypoint
      task_file: truck3_waypoints.yaml
```

The scenario continues after all parallel child steps complete successfully.

---

## `condition`

Waits for a ROS topic value to satisfy a condition.

This can be used to coordinate execution based on robot state.

Example conceptual flow:

```text
Truck 1 Action
      │
      ▼
Truck 1 status
      │
      ▼
state == completed
      │
      ▼
Condition satisfied
      │
      ▼
Next task
```

---

## `topic_publish`

Publishes directly to a ROS topic.

This is useful for:

- simple actuator commands
- direct test commands
- integration testing
- operations that do not require an Action abstraction

---

# 20. Current Validation Status

## Verified

The following software and integration components have been successfully tested:

```text
Excavator configuration loading

Excavator trajectory loading

Subset-joint trajectories

Trajectory limit validation

FollowJointTrajectory goal validation

Excavator SIM Action Server

Excavator Action client

Excavator launch file

Command Center → Excavator SIM

Scenario Manager → Excavator SIM

Mock Dump Truck Action Server

Mixed Dump Truck + Excavator scenario

Zenoh ROS 2 communication

Excavator FollowJointTrajectory over Zenoh

Dump Truck ROS communication over Zenoh

Overhead Camera and AprilTag communication over Zenoh

Physical Dump Truck end-to-end operation over Zenoh
```

The excavator package currently passes:

```text
24 tests
0 errors
0 failures
0 skipped
```

A software-only mixed-robot scenario has also been successfully executed:

```text
Mock Dump Truck
      │
      ▼
Excavator SIM
      │
      ▼
Mock Dump Truck
      │
      ▼
SCENARIO COMPLETE
```

---

## Pending Hardware Validation

Physical closed-loop excavator control is **not yet considered fully validated**.

Additional hardware work is still required for:

- boom sensor/calibration behavior
- swing sensing/control behavior
- physical trajectory execution
- multi-joint physical trajectories
- final physical safety and motion validation

These items are separate from the Command Center and communication architecture.

Software development and integration can continue using SIM mode while the remaining excavator hardware work is addressed.

---

# 21. Recommended Development Workflow

Use the system in progressively higher-risk stages:

```text
1. Unit Tests
      │
      ▼
2. Excavator SIM
      │
      ▼
3. Mock Truck + Excavator SIM
      │
      ▼
4. Single Physical Robot
      │
      ▼
5. Multi-Robot Physical Integration
      │
      ▼
6. Integrated Construction Scenario
```

Do not move directly from an untested trajectory or scenario to full multi-robot physical operation.

For excavator trajectories in particular, validate the YAML before physical execution.

Example:

```bash
ros2 run excavator_control \
  validate_excavator_trajectory \
  robots/excavator/excavator_control/config/excavator1.yaml \
  operations/excavator/trajectories/boom_small_test.yaml
```

Expected:

```text
TRAJECTORY IS VALID
```

YAML validation confirms configuration and trajectory consistency.

It does not prove that physical sensor calibration or resulting robot motion is correct.

---

# 22. Current Architecture

The current architecture supports:

```text
Operational YAML
      │
      ▼
Scenario Manager
      │
      ├─────────────────────────────┐
      │                             │
      ▼                             ▼
ExecuteRobotTask             FollowJointTrajectory
      │                             │
      ▼                             ▼
Dump Truck                       Excavator
      │                             │
Mock / Physical                SIM / Physical
      │                             │
      └──────────────┬──────────────┘
                     ▼
                Zenoh Network
```

The current project direction is:

1. Continue refining scenario definitions.
2. Expand validated operational trajectories.
3. Complete physical excavator calibration and closed-loop testing.
4. Continue multi-robot physical integration.
5. Execute integrated construction scenarios.

---

# 23. Where New Command Center Files Belong

Use the following rule when extending the system:

```text
Scenario Manager logic
    → command_center/construction_site_control/

Excavator Command Center client logic
    → command_center/construction_site_control/

Dump truck Action Server logic
    → command_center/dump_truck_action_server/

Scenario YAML
    → operations/scenarios/

Dump truck waypoint YAML
    → operations/dump_truck/waypoints/

Excavator trajectory YAML
    → operations/excavator/trajectories/

Shared ROS messages and Actions
    → common/construction_site_interfaces/

Robot-specific hardware and control
    → robots/

ROS network configuration
    → network/
```

The general design principle is:

> The Command Center coordinates robot tasks, while robot-specific packages remain responsible for executing the physical motion.