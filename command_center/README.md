# Construction Robotics Command Center

The `command_center` directory contains the ROS 2 components used to coordinate multiple construction robots in the CIC ConRobotics system.

The Command Center provides a higher-level execution layer above individual robot controllers. It allows construction operations to be described as YAML scenarios and executed as sequences of robot tasks, waits, topic commands, parallel operations, and conditions.

The current system supports:

- Dump truck waypoint tasks using `ExecuteRobotTask`
- Excavator joint trajectories using `FollowJointTrajectory`
- Sequential multi-robot scenarios
- Parallel scenario steps
- Conditional execution based on ROS 2 topics
- Direct topic publishing
- Software-only testing using a mock dump truck and simulated excavator

---

## System Architecture

```text
                     Scenario YAML
                          |
                          v
                 +------------------+
                 | Scenario Manager |
                 +------------------+
                    |            |
                    |            |
        ExecuteRobotTask          FollowJointTrajectory
                    |            |
                    v            v
              +-----------+   +-----------+
              | Dump Truck|   | Excavator |
              +-----------+   +-----------+
                    |            |
              Physical / Mock   Physical / SIM
```

The Command Center does not directly control motors.

Instead, it sends higher-level commands to robot-specific ROS 2 Action Servers.

---

## Directory Structure

```text
command_center/
├── construction_site_control/
│   └── construction_site_control/
│       ├── scenario_manager_node.py
│       └── excavator_task_client.py
│
└── dump_truck_action_server/
    └── dump_truck_action_server/
        ├── waypoint_action_server_node.py
        └── mock_waypoint_action_server_node.py
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
      |
      v
Excavator operates
      |
      v
Truck 1 departs
```

---

# 5. Software-Only Integration Testing

The Command Center can be tested without physical robots.

This is the recommended development workflow before testing physical hardware.

The current software-only configuration uses:

```text
Dump Truck  -> Mock Action Server
Excavator   -> SIM mode
```

This allows the complete scenario orchestration layer to be tested independently of sensors, motors, Raspberry Pis, and physical calibration.

---

## Build

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

# 6. Start the Excavator in SIM Mode

Terminal 1:

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

# 7. Start a Mock Dump Truck

Terminal 2:

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

# 8. Run a Mixed-Robot Scenario

Terminal 3:

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

truck1_arrive
    |
    v
Dump Truck Mock
    |
    v
SUCCESS

excavator_load
    |
    v
Excavator SIM
    |
    v
EXCAVATOR SUCCESS

truck1_depart
    |
    v
Dump Truck Mock
    |
    v
SUCCESS

SCENARIO COMPLETE
```

This workflow has been successfully tested as a software-only multi-robot integration test.

---

# 9. Excavator-Only Scenario Test

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
      |
      v
Scenario Manager
      |
      v
FollowJointTrajectory
      |
      v
Excavator SIM
```

without starting any dump truck components.

---

# 10. Direct Excavator Command Center Test

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
Excavator trajectory YAML
          |
          v
excavator_task_client
          |
          v
FollowJointTrajectory
          |
          v
excavator_control
```

---

# 11. Physical Robot Operation

The same high-level interfaces are intended to be used for both simulation and physical operation.

For the excavator:

```text
Command Center
      |
      | FollowJointTrajectory
      v
excavator_control
      |
      +---- SIM mode
      |
      +---- PI mode
```

For the dump trucks:

```text
Command Center
      |
      | ExecuteRobotTask
      v
dump_truck_action_server
      |
      v
dump_truck_control
      |
      v
Physical Dump Truck
```

The purpose of this architecture is to keep scenario definitions independent from the underlying hardware implementation.

---

# 12. ROS 2 Network Setup

For multi-machine operation, each computer must be configured for the project ROS 2 network.

Use:

```bash
source network/setup_network.sh \
  <this_device> \
  <peer_device_1> \
  <peer_device_2>
```

For example, during excavator testing:

Desktop:

```bash
source network/setup_network.sh \
  ros_laptop_backup \
  excavator_01
```

Excavator Raspberry Pi:

```bash
source network/setup_network.sh \
  excavator_01 \
  ros_laptop_backup
```

The project currently uses:

```text
ROS_DOMAIN_ID=10
```

---

# 13. RMW / DDS Status

Multi-machine ROS 2 communication has been verified for the excavator using CycloneDDS on both endpoints:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

With CycloneDDS configured on both the ROS computer and excavator Raspberry Pi:

- ROS 2 topic communication was verified.
- `/joint_states` communication was verified.
- `FollowJointTrajectory` Action discovery was verified.
- Action goal transmission was verified.

Current communication testing also identified two unresolved issues:

### Default / Unspecified RMW

Topic communication worked, but `FollowJointTrajectory` Action goal data was not transmitted correctly during testing.

### Fast DDS on the Excavator Raspberry Pi

Explicit Fast DDS startup currently fails because of a Fast DDS / Fast CDR library compatibility issue on the excavator Raspberry Pi.

Therefore:

> Excavator ROS 2 Action communication is currently verified with CycloneDDS on both endpoints.

This does **not** yet establish CycloneDDS as the required RMW implementation for the entire construction robotics system.

Compatibility with the camera, AprilTag perception system, and dump truck system still needs to be evaluated.

Possible future communication investigations include:

- Re-testing the excavator using the existing Python virtual environment with the default RMW and Fast DDS.
- Evaluating ROS 2 Zenoh / `rmw_zenoh` for multi-machine communication.
- Evaluating a consistent RMW configuration across the excavator, dump trucks, camera, perception system, and Command Center.

---

# 14. Current Validation Status

## Verified

The following software components have been successfully tested:

```text
Excavator configuration loading
Excavator trajectory loading
Subset-joint trajectories
Trajectory limit validation
FollowJointTrajectory goal validation
Excavator SIM Action Server
Excavator Action client
Excavator launch file
Command Center -> Excavator SIM
Scenario Manager -> Excavator SIM
Mock Dump Truck Action Server
Mixed Dump Truck + Excavator scenario
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
      |
      v
Excavator SIM
      |
      v
Mock Dump Truck
      |
      v
SCENARIO COMPLETE
```

---

## Pending Hardware Validation

Physical closed-loop excavator control is **not yet considered validated**.

In particular, additional hardware work is required for:

- Boom sensor/calibration behavior
- Swing sensing/control behavior
- Physical trajectory execution
- Multi-joint physical trajectories
- Final physical safety and motion validation

Software development and system integration can continue using SIM mode while these hardware items are addressed.

---

# 15. Recommended Development Workflow

Use the system in progressively higher-risk stages:

```text
1. Unit Tests
      |
      v
2. Excavator SIM
      |
      v
3. Mock Truck + Excavator SIM
      |
      v
4. Single Physical Robot
      |
      v
5. Multi-Robot Physical Integration
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

Note that YAML validation confirms configuration and trajectory consistency. It does not prove that the physical sensor calibration or resulting robot motion is correct.

---

# 16. Current Development Status

The current architecture supports:

```text
Operational YAML
      |
      v
Scenario Manager
      |
      +-----------------------+
      |                       |
      v                       v
ExecuteRobotTask      FollowJointTrajectory
      |                       |
      v                       v
Dump Truck             Excavator
      |                       |
 Mock / Physical          SIM / Physical
```

The next development stages are:

1. Continue refining scenario definitions.
2. Replace test trajectories with validated operational excavator trajectories.
3. Complete physical excavator calibration and closed-loop testing.
4. Verify multi-machine communication across all robot and perception systems.
5. Execute integrated physical construction scenarios.