# Construction Robotics Command Center

This directory contains the high-level ROS 2 coordination layer for the
CIC-ConRobotics construction robotics platform.

For normal physical operation, **use the Command Center as the ROS PC
entry point**. The Command Center coordinates shared perception,
dump-truck localization and task execution, excavator perception, and
construction scenarios.

> **Normal operation:** `command_center.launch.py`\
> **Debug / subsystem validation only:** `excavator_system.launch.py`,
> `dump_truck_system.launch.py`, and lower-level component launch files.

------------------------------------------------------------------------

## 1. Quick Start

This section is the shortest path from a powered system to a running
construction scenario.

The normal physical architecture is:

``` text
                    ACTIVE ROS PC
                         │
                ┌────────┴────────┐
                │                 │
          Zenoh Router       Command Center
                │                 │
        ┌───────┼─────────┐       │
        │       │         │       │
        ▼       ▼         ▼       ▼
     Truck Pi  Truck Pi  Excavator Pi
        │       │         │
        └───────┴─────────┘
                │
          Physical Robots
```

The overhead camera and AprilTag detector are **shared site
perception**. They are started by the Command Center and are not owned
by any individual robot.

### 1.1 Start the Zenoh Router

On the active ROS PC:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh router ros-pc
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Keep this terminal running.

Only one Zenoh router should normally be active.

For complete network setup, device profiles, backup-router operation,
and network troubleshooting, see:

``` text
network/README.md
```

### 1.2 Start the Physical Robots

Each participating robot runs its hardware-side software on its own
Raspberry Pi. In normal operation, the robot Raspberry Pis can be accessed from the ROS PC or your laptop through SSH; for SSH setup, host configuration, and connection details, see `network/README.md`.

#### Dump Truck Example --- Truck 1

On the Truck 1 Raspberry Pi:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client dumptruck1
sudo pigpiod
ros2 launch dump_truck_bringup \
  dump_truck_pi.launch.py \
  truck_name:=truck1
```

Other dump trucks use the same launch file with their own device profile
and robot name.

Examples:

``` text
dumptruck1 → truck1
dumptruck3 → truck3
dumptruck4 → truck4
dumptruck5 → truck5
```

#### Excavator Example --- Excavator 3

On the Excavator 3 Raspberry Pi:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client excavator3
sudo pigpiod
ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi \
  robot_name:=excavator3
```

During physical startup, the excavator trajectory server waits for fresh
swing feedback from the shared overhead AprilTag system before
completing initialization.

Conceptually:

``` text
Fresh Swing Feedback
        │
        ▼
     Preflight
        │
        ▼
 Initial Position
        │
        ▼
Final Joint Verification
        │
        ▼
      READY
```

Trajectory goals are rejected until initialization completes
successfully.

### 1.3 Run the Command Center and Scenario

In the second terminal on the active ROS PC:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:=truck1 \
  excavators:=excavator3 \
  start_scenario_manager:=true \
  scenario:=YOUR_SCENARIO.yaml
```

Replace the robot lists and scenario file with the robots participating
in the operation.

Scenario files are stored in:

``` text
operations/scenarios/
```

The Command Center starts the shared ROS PC infrastructure required by
the selected robots, including:

-   overhead camera
-   AprilTag detection
-   excavator swing-position adapters
-   dump-truck odometry and localization
-   dump-truck waypoint Action Servers
-   Scenario Manager when requested

For a scenario that uses only one robot type, explicitly provide the
intended robot selection.

Examples:

``` bash
ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:=truck1 \
  excavators:="" \
  start_scenario_manager:=true \
  scenario:=YOUR_TRUCK_SCENARIO.yaml
```

``` bash
ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:="" \
  excavators:=excavator3 \
  start_scenario_manager:=true \
  scenario:=YOUR_EXCAVATOR_SCENARIO.yaml
```

------------------------------------------------------------------------

## 2. Run a Scenario

A scenario is the standard way to coordinate construction operations.

Scenario YAML files live in:

``` text
operations/scenarios/
```

The Scenario Manager currently supports:

``` text
task
excavator_trajectory
wait
parallel
condition
topic_publish
```

A scenario may contain dump-truck tasks, excavator trajectories, waits,
synchronization conditions, direct topic publications, and parallel
robot operations.

Before combining robots into a new physical scenario, validate each dump-truck task and excavator trajectory independently. Only integrate individually verified robot motions into a multi-robot scenario.

### 2.1 Example Mixed-Robot Scenario

``` yaml
scenario_name: truck1_excavator3_example

steps:
  - id: truck1_route
    type: task
    robot: truck1
    task_type: waypoint
    task_file: truck1_waypoints.yaml

  - id: excavator3_move
    type: excavator_trajectory
    robot: excavator3
    task_file: excavator3_excavation_cycle_test.yaml
    seconds_per_waypoint: 5.0
```

Sequential Action steps proceed only after the previous step completes
successfully.

Conceptually:

``` text
Scenario Start
     │
     ▼
Truck 1 Task
     │
     ▼
   SUCCESS
     │
     ▼
Excavator 3 Trajectory
     │
     ▼
   SUCCESS
     │
     ▼
Scenario Complete
```

If an Action step reports failure, the sequential scenario is aborted
rather than continuing as though the step succeeded.

### 2.2 Select the Participating Robots

The `trucks` and `excavators` launch arguments determine which
robot-specific ROS PC components are started.

Examples:

``` text
trucks:=truck1
trucks:=truck1,truck3
trucks:=truck1,truck3,truck4,truck5

excavators:=excavator3
excavators:=excavator1,excavator3
```

The selected robots should match the robots referenced by the scenario.

### 2.3 Start a Scenario

Example:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:=truck1 \
  excavators:=excavator3 \
  start_scenario_manager:=true \
  scenario:=YOUR_SCENARIO.yaml
```

For a new physical scenario, it is often useful to first start the
Command Center without automatically starting the Scenario Manager:

``` bash
ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:=truck1 \
  excavators:=excavator3 \
  start_scenario_manager:=false
```

Verify the intended ROS interfaces and physical system before initiating
motion.

------------------------------------------------------------------------

## 3. Excavator Trajectory

Excavators use the standard ROS 2:

``` text
control_msgs/action/FollowJointTrajectory
```

Each excavator is independently namespaced.

For Excavator 3:

``` text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

Trajectory files are stored in:

``` text
operations/excavator/trajectories/
```

### 3.1 Trajectory YAML Format

Excavator trajectories are written in degrees.

Example:

``` yaml
trajectory_name: example_trajectory
description: >
  Example excavator trajectory.

joints:
  - swing
  - boom
  - arm
  - bucket

waypoints:
  - name: position_1
    positions:
      swing: 90.0
      boom: -40.0
      arm: 100.0
      bucket: 15.0

  - name: position_2
    positions:
      swing: 45.0
      boom: -35.0
      arm: 95.0
      bucket: 25.0
```

Supported logical joints are:

``` text
swing
boom
arm
bucket
```

The trajectory client converts the trajectory values from degrees to
radians before sending the ROS 2 Action goal.

### 3.2 Subset-Joint Trajectories

A trajectory does not need to command all four joints.

Example:

``` yaml
trajectory_name: boom_only_example
description: >
  Example trajectory that commands only the boom.

joints:
  - boom

waypoints:
  - name: position_1
    positions:
      boom: -40.0

  - name: position_2
    positions:
      boom: -45.0
```

Joints that are not listed are not included in the resulting
`FollowJointTrajectory` goal.

Subset-joint trajectories are useful for:

-   isolated joint testing
-   calibration
-   hardware debugging
-   operations that do not require all joints

### 3.3 Send a Trajectory Directly

For direct testing without the Scenario Manager:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
ros2 run construction_site_control \
  excavator_task_client \
  operations/excavator/trajectories/YOUR_TRAJECTORY.yaml \
  --robot excavator3 \
  --seconds-per-waypoint 5.0
```

This targets:

``` text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

### 3.4 Use a Trajectory in a Scenario

``` yaml
- id: excavator3_move
  type: excavator_trajectory
  robot: excavator3
  task_file: excavator3_excavation_cycle_test.yaml
  seconds_per_waypoint: 5.0
```

Unless explicitly overridden, the target Action is resolved from the
`robot` field.

For example:

``` text
robot: excavator3
        ↓
/excavator3/upper_arm_controller/follow_joint_trajectory
```

### 3.5 Physical Startup and READY State

The physical excavator trajectory server does not immediately accept
motion goals at startup.

It waits for fresh swing feedback from the overhead AprilTag system and
then performs startup initialization.

``` text
Server Starts
     │
     ▼
Wait for Fresh Swing Feedback
     │
     ▼
Preflight
     │
     ▼
Initialize Swing
     │
     ▼
Initialize Boom
     │
     ▼
Initialize Arm
     │
     ▼
Initialize Bucket
     │
     ▼
Final Four-Joint Verification
     │
     ▼
READY
```

If fresh swing feedback is unavailable, the motors remain off while the
server waits.

Do not send a physical trajectory until the excavator reports that
initialization completed successfully.

### 3.6 Trajectory Safety

Trajectory validation checks software structure and configured limits.
It does **not** independently prove that a physical motion is safe.

Before physical execution, confirm:

-   the intended excavator
-   trajectory joint names
-   waypoint positions
-   configured joint limits
-   physical workspace clearance
-   sensor behavior
-   motor direction
-   mechanical clearance

Immediately stop testing if a joint moves in the wrong direction, an
unexpected joint moves, motion becomes unstable, a mechanical limit is
approached, or motion does not stop as expected.

------------------------------------------------------------------------

## 4. Dump Truck Task

Dump trucks use the custom ROS 2 Action:

``` text
construction_site_interfaces/action/ExecuteRobotTask
```

Each truck exposes a namespaced Action.

For Truck 1:

``` text
/truck1/execute_robot_task
```

Waypoint files are stored in:

``` text
operations/dump_truck/waypoints/
```

### 4.1 Waypoint Task

A scenario executes a dump-truck waypoint task using:

``` yaml
- id: truck1_route
  type: task
  robot: truck1
  task_type: waypoint
  task_file: truck1_waypoints.yaml
```

The execution path is:

``` text
Scenario Manager
      │
      ▼
/truck1/execute_robot_task
      │
      ▼
Dump Truck Action Server
      │
      ▼
Waypoint YAML
      │
      ▼
Waypoint Controller
      │
      ▼
/truck1/cmd_vel
      │
      ▼
Physical Truck
```

### 4.2 Dump Truck Localization

The normal ROS PC stack for a selected truck includes:

``` text
Wheel Encoder Odometry
        │
        ▼
     /truck/odom
        │
        ▼
AprilTag / Odom Fusion
        │
        ▼
 /truck/fused_odom
        │
        ▼
Waypoint Controller
```

The overhead camera and AprilTag detector are shared with the rest of
the construction site.

### 4.3 Robot Status

Dump-truck task execution publishes robot status under:

``` text
/<truck_name>/status
```

For example:

``` text
/truck1/status
```

Common states include:

``` text
idle
waiting
navigating
performing_action
completed
fault
```

Robot status may also be used by the Scenario Manager for state-based
synchronization.

------------------------------------------------------------------------

## 5. How to Create a Scenario

Create scenario YAML files under:

``` text
operations/scenarios/
```

A scenario contains a `scenario_name` and a sequence of `steps`.

Example:

``` yaml
scenario_name: example_construction_scenario

steps:
  - id: truck1_route
    type: task
    robot: truck1
    task_type: waypoint
    task_file: truck1_waypoints.yaml

  - id: wait_after_truck
    type: wait
    duration: 3.0

  - id: excavator3_move
    type: excavator_trajectory
    robot: excavator3
    task_file: excavator3_excavation_cycle_test.yaml
    seconds_per_waypoint: 5.0
```

### 5.1 `task`

Executes a robot task through `ExecuteRobotTask`.

``` yaml
- id: truck1_route
  type: task
  robot: truck1
  task_type: waypoint
  task_file: truck1_waypoints.yaml
```

For Truck 1, this targets:

``` text
/truck1/execute_robot_task
```

### 5.2 `excavator_trajectory`

Executes an excavator trajectory through `FollowJointTrajectory`.

``` yaml
- id: excavator3_move
  type: excavator_trajectory
  robot: excavator3
  task_file: excavator3_excavation_cycle_test.yaml
  seconds_per_waypoint: 5.0
```

The `robot` field normally determines the target Action namespace.

### 5.3 `wait`

Introduces a timed delay.

``` yaml
- id: wait_after_truck
  type: wait
  duration: 3.0
```

### 5.4 `parallel`

Runs multiple child tasks simultaneously.

``` yaml
- id: parallel_operation
  type: parallel
  tasks:
    - id: truck1_move
      type: task
      robot: truck1
      task_type: waypoint
      task_file: truck1_waypoints.yaml

    - id: excavator3_move
      type: excavator_trajectory
      robot: excavator3
      task_file: excavator3_excavation_cycle_test.yaml
      seconds_per_waypoint: 5.0
```

The scenario continues after all parallel child tasks complete
successfully.

### 5.5 `condition`

Waits for a supported ROS topic value to satisfy a condition.

The current Scenario Manager supports condition handling for:

``` text
std_msgs/String
construction_site_interfaces/msg/RobotStatus
```

This can be used to synchronize scenario execution with robot or system
state.

Use an existing scenario under `operations/scenarios/` as the template
when creating a new condition step.

### 5.6 `topic_publish`

Publishes directly to a supported ROS topic.

This is useful for:

-   simple commands
-   integration testing
-   operations that do not require a complete Action abstraction

Use an existing `topic_publish` scenario under `operations/scenarios/`
as the template for the exact message definition required by the target
topic.

### 5.7 Scenario Design Rules

When creating a new scenario:

1.  Use a unique `id` for each step.
2.  Use the correct `robot` name.
3.  Keep waypoint files under `operations/dump_truck/waypoints/`.
4.  Keep excavator trajectories under
    `operations/excavator/trajectories/`.
5.  Keep scenario files under `operations/scenarios/`.
6.  Test individual robot tasks before combining them into a new
    physical multi-robot scenario.
7.  Confirm the participating robots are included in the `trucks` and
    `excavators` Command Center launch arguments.
8.  Treat Action failure as a real scenario failure; do not assume the
    next sequential step will run.

------------------------------------------------------------------------

## 6. Troubleshooting

Start with the smallest layer that can prove where the problem is.

### 6.1 Basic ROS 2 Check

In a terminal configured as a Zenoh client:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
ros2 node list
ros2 topic list
ros2 action list
```

If remote robot nodes are missing, check the Zenoh setup first:

``` text
network/README.md
```

### 6.2 Visualize the ROS Graph

For teaching and diagnostics:

``` bash
rqt_graph
```

This is useful for confirming which nodes, topics, and interfaces are
currently connected.

### 6.3 Excavator Action Is Missing

For Excavator 3:

``` bash
ros2 action info \
  /excavator3/upper_arm_controller/follow_joint_trajectory
```

The physical excavator Raspberry Pi must be running the trajectory
server.

Expected Action:

``` text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

### 6.4 Excavator Does Not Reach READY

Check that the Command Center is running the shared overhead camera,
AprilTag detector, and the excavator perception adapter.

Check the swing feedback:

``` bash
ros2 topic echo /excavator3/swing_joint_state
```

The excavator startup sequence requires fresh swing feedback before
initialization can proceed.

If feedback is unavailable, inspect:

``` text
Camera
  ↓
AprilTag Detector
  ↓
Swing Position Adapter
  ↓
/excavator3/swing_joint_state
```

The motors should remain off while the trajectory server waits for fresh
startup swing feedback.

### 6.5 Excavator Trajectory Is Rejected

Check:

-   trajectory joint names
-   duplicate joints
-   waypoint structure
-   missing joint positions
-   configured joint limits
-   target robot name
-   excavator READY state

Use a small subset-joint trajectory when isolating a joint-level
problem.

### 6.6 Excavator SIM Test

This section is primarily for Isaac Sim and software-integration development. It is not part of the normal physical-robot operating workflow and can generally be ignored during routine field operation.

SIM mode does not access Raspberry Pi hardware.

Start a simulated excavator:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch excavator_control \
  excavator.launch.py \
  mode:=sim \
  robot_name:=excavator1
```

Send a trajectory:

``` bash
ros2 run construction_site_control \
  excavator_task_client \
  operations/excavator/trajectories/YOUR_TRAJECTORY.yaml \
  --robot excavator1 \
  --seconds-per-waypoint 3.0
```

Observe SIM commands:

``` bash
ros2 topic echo /excavator1/joint_command
```

If no trajectory is executing, `ros2 topic echo` may wait without
displaying data. This is normal.

### 6.7 Dump Truck Action Is Missing

For Truck 1:

``` bash
ros2 action info /truck1/execute_robot_task
```

The Command Center must have been started with Truck 1 selected.

### 6.8 Dump Truck Localization Is Missing

Check:

``` bash
ros2 topic echo /truck1/wheel_states
ros2 topic echo /truck1/odom
ros2 topic echo /truck1/fused_odom
```

Use the results to isolate the failing layer:

``` text
No wheel_states
    → Pi / encoder / hardware layer

wheel_states but no odom
    → odometry layer

odom but no fused_odom
    → AprilTag / fusion layer

fused_odom available but task fails
    → waypoint / Action layer
```

### 6.9 Manual Dump Truck Motion Test

Direct `cmd_vel` is a diagnostic tool, not the normal operational
interface.

Example:

``` bash
ros2 topic pub -r 10 \
  /truck1/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}"
```

Use manual motion only when validating motor direction, encoder mapping,
odometry, or low-level motion behavior.

### 6.10 Camera / AprilTag Problems

Check:

``` bash
ros2 node list
ros2 topic list
```

Expected shared perception nodes include the overhead camera and
AprilTag detector.

For AprilTag calibration procedures, use:

``` text
docs/perception/apriltag/Calibration.md
```

Do not place camera or AprilTag calibration procedures in this
operational README.

### 6.11 Network Problems

Network configuration and Zenoh troubleshooting belong in:

``` text
network/README.md
```

Normal physical operation uses:

``` text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Do not switch middleware as a troubleshooting shortcut.

### 6.12 Shutdown Warnings

Some perception processes may emit warnings during shutdown even after
normal runtime operation.

Evaluate runtime behavior separately from Ctrl+C shutdown messages. If
the system operated correctly but a process reports a shutdown-only
error, record it as a cleanup issue rather than immediately treating it
as a startup or communication failure.

------------------------------------------------------------------------

## 7. Architecture

The platform separates robot hardware, robot control, shared perception,
operations, communication, and high-level coordination.

### 7.1 High-Level System

``` text
                         Scenario YAML
                              │
                              ▼
                       Scenario Manager
                              │
             ┌────────────────┴────────────────┐
             │                                 │
             ▼                                 ▼
      Dump Truck Task                 Excavator Trajectory
             │                                 │
             ▼                                 ▼
   ExecuteRobotTask                  FollowJointTrajectory
             │                                 │
             ▼                                 ▼
   Waypoint Controller              Excavator Controller
             │                                 │
             └──────────────┬──────────────────┘
                            │
                            ▼
                     Physical Robots


                    Shared Perception
                            │
                 ┌──────────┴──────────┐
                 │                     │
                 ▼                     ▼
           Overhead Camera        AprilTag Detection
                                       │
                          ┌────────────┴────────────┐
                          │                         │
                          ▼                         ▼
                 Truck Localization        Excavator Swing
                                             Feedback
```

The overhead camera and AprilTag detector are **site-wide shared
infrastructure**.

They support both dump-truck localization and excavator swing feedback.

### 7.2 ROS Interfaces

Dump trucks use:

``` text
construction_site_interfaces/action/ExecuteRobotTask
```

Example:

``` text
/truck1/execute_robot_task
```

Excavators use:

``` text
control_msgs/action/FollowJointTrajectory
```

Example:

``` text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

Robot namespaces allow multiple trucks and excavators to coexist on the
same ROS 2 network.

### 7.3 Repository Responsibilities

``` text
robots/
    Robot-specific hardware, control, configuration, and bringup

common/
    Shared ROS 2 interfaces

perception/
    Shared camera, AprilTag, and perception components

command_center/
    High-level task execution and multi-robot coordination

operations/
    Waypoints, excavator trajectories, and construction scenarios

network/
    Zenoh and multi-machine ROS 2 communication

docs/
    Supporting technical documentation that does not belong in normal operation

tools/
    Diagnostic and utility scripts
```

Operational data is intentionally separated from reusable robot
software.

``` text
operations/
├── dump_truck/
│   └── waypoints/
├── excavator/
│   └── trajectories/
└── scenarios/
```

------------------------------------------------------------------------

## 8. Development

This section is for software development, debugging, integration
testing, and adding new robot behavior. It is not the normal
student/operator startup path.

### 8.1 Adding a New Robot

Add and validate a new physical robot **before** integrating it into a
multi-robot scenario.

Use this development sequence:

``` text
Create Robot Configuration
        │
        ▼
Validate Physical Hardware
        │
        ▼
Validate ROS Interfaces
        │
        ▼
Validate Individual Task / Motion
        │
        ▼
Integrate into a Scenario
```

Do not use a multi-robot scenario as the first test of a newly added
robot. Hardware, sensing, ROS communication, and individual robot motion
should already be working independently.

#### 8.1.1 Common Setup

For every new physical robot:

1. Assign a unique device name and ROS robot name.
2. Add the Raspberry Pi to the device registry in:

``` text
network/devices.sh
```

3. Configure and verify SSH and Zenoh communication using:

``` text
network/README.md
```

4. Create a robot-specific machine configuration under `robots/`.
5. Verify physical sensors, wiring, motor direction, and actuator
   behavior.
6. Verify the robot's ROS 2 topics and Action interface.
7. Create and validate an individual waypoint task or trajectory.
8. Only after the individual robot works correctly, add it to a
   scenario under:

``` text
operations/scenarios/
```

#### 8.1.2 Adding a New Excavator

Excavator machine configurations are stored in:

``` text
robots/excavator/excavator_control/config/
```

**Always start from the excavator configuration template.**

For example, when adding Excavator 6:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

cp   robots/excavator/excavator_control/config/excavator_template.yaml   robots/excavator/excavator_control/config/excavator6.yaml
```

Then edit:

``` text
robots/excavator/excavator_control/config/excavator6.yaml
```

At minimum, verify and configure the following for the physical
machine:

``` text
excavator_name
    Unique robot name, for example excavator6

joints
    Boom / arm / bucket ADC channels
    Boom / arm / bucket physical limits
    Boom / arm / bucket raw ADC calibration
    Swing physical limits

gpio
    GPIO pin assignments
    Motor direction for every joint

initial_position
    Safe startup target for all four joints
    Startup PWM and pulse timing
    Per-joint timeout
    Wrong-way detection

joint_control
    Swing feedback topic
    Swing commanded range
    Joint control parameters and tolerances
```

Boom, arm, and bucket calibration values are **machine-specific**.
Measure the actual physical excavator; do not copy raw ADC values from
another excavator.

Swing feedback uses the site perception system rather than the
excavator's ADS1115 calibration. Set the Swing feedback topic to match
the new robot name.

For Excavator 6:

``` yaml
excavator_name: excavator6
```

and:

``` yaml
joint_control:
  swing:
    position_topic: /excavator6/swing_joint_state
```

Before normal operation, verify the new excavator in stages:

``` text
1. Sensor feedback
2. GPIO wiring
3. Motor direction
4. Joint limits
5. Swing perception feedback
6. Startup initialization
7. Individual joint motion
8. Multi-joint trajectory
9. Scenario integration
```

Do not assume that the template's startup target, PWM values, motor
directions, or control parameters are physically correct for a new
machine. Begin with small, controlled tests.

After hardware validation, create an operational trajectory under:

``` text
operations/excavator/trajectories/
```

Validate that trajectory directly as described in Section 3 before
using the excavator in a scenario.

#### 8.1.3 Adding a New Dump Truck

Dump-truck hardware configurations are stored in:

``` text
robots/dump_truck/dump_truck_bringup/config/hardware/
```

When adding a new truck, start from a **verified existing truck
configuration** and create a new robot-specific YAML file.

For example, when adding Truck 6:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

cp   robots/dump_truck/dump_truck_bringup/config/hardware/truck5.yaml   robots/dump_truck/dump_truck_bringup/config/hardware/truck6.yaml
```

Do not assume that the copied machine-specific values are correct for
the new truck.

At minimum, verify and configure:

``` text
robot identity / namespace
servo calibration
robot_tag_child_frame
AprilTag yaw offset
odometry scale factors
encoder direction
encoder inversion
swap_encoders
```

Encoder mapping and motor direction must be checked physically for each
truck.

Then verify the truck in stages:

``` text
1. Motor and steering response
2. Wheel encoder feedback
3. Odometry
4. AprilTag detection
5. Fused localization
6. Manual motion
7. Individual waypoint task
8. Scenario integration
```

Create operational waypoint files under:

``` text
operations/dump_truck/waypoints/
```

Validate the waypoint task independently as described in Section 4
before adding the truck to a multi-robot scenario.

### 8.2 Normal vs. Debug Launch Files

Normal ROS PC operation:

``` text
command_center.launch.py
```

Subsystem launch files such as:

``` text
excavator_system.launch.py
dump_truck_system.launch.py
dump_truck_ros_pc.launch.py
overhead_camera.launch.py
excavator_perception.launch.py
```

remain useful for debugging and component-level validation, but they
should not be presented as the normal operational entry point.

### 8.3 Build the Command Center Stack

From the repository root:

``` bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For targeted development, package-specific builds may be used when
appropriate.

### 8.4 Software-Only Integration Testing

A useful software-only configuration is:

``` text
Dump Truck  → Mock Action Server
Excavator   → SIM Mode
```

Example mock Truck 1 Action Server:

``` bash
ros2 run dump_truck_action_server \
  mock_waypoint_action_server_node \
  --ros-args \
  -p truck_name:=truck1 \
  -p seconds_per_waypoint:=0.5
```

Example Excavator 1 SIM server:

``` bash
ros2 launch excavator_control \
  excavator.launch.py \
  mode:=sim \
  robot_name:=excavator1
```

The Scenario Manager can then be tested independently of physical
motors, sensors, Raspberry Pis, and mechanical calibration.

### 8.5 Adding Operational Files

Use the following rule:

``` text
New dump-truck route
    → operations/dump_truck/waypoints/

New excavator motion
    → operations/excavator/trajectories/

New multi-robot workflow
    → operations/scenarios/
```

Do not place operation-specific waypoint or trajectory data inside
reusable ROS 2 packages.

### 8.6 Repository Locations

Use the following ownership rule when adding or modifying robot content:

```text
network/
    Network device definitions and Zenoh configuration

robots/
    Robot-specific software, launch files, and machine configuration

operations/
    Waypoints, excavator trajectories, and scenarios
```

Keep machine configuration separate from operational task definitions.

### 8.7 Documentation Rule

This README is the primary operational manual for the construction
robotics system.

Keep normal operating procedures here rather than duplicating them
across robot-specific directories.

Use:

``` text
README.md
```

for repository entry, initial setup, and branch guidance.

Use:

``` text
command_center/README.md
```

for system operation, scenarios, robot tasks, troubleshooting,
architecture, and development guidance.

Use:

``` text
network/README.md
```

for Zenoh and multi-machine network configuration.

Keep standalone documents under `docs/` only when the topic is
specialized and would unnecessarily clutter the normal operational
workflow, such as AprilTag calibration.

------------------------------------------------------------------------

## Summary

For normal physical operation:

``` text
1. Start one Zenoh router.
2. Start the participating robot Raspberry Pis.
3. Start command_center.launch.py on the ROS PC.
4. Run the intended scenario.
```

The central rule is:

``` text
Normal ROS PC operation → command_center.launch.py
```

Use subsystem launch files only for debugging and component-level
validation.