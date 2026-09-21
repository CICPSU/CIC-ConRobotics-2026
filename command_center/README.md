# Construction Robotics Command Center

The `command_center` directory contains the ROS 2 components used to coordinate multiple construction robots in the CIC ConRobotics system.

The Command Center provides a higher-level execution layer above individual robot controllers. Construction operations can be described as YAML scenarios and executed as sequences of robot tasks, excavator trajectories, waits, topic commands, parallel operations, and conditions.

The current system supports:

- Dump truck waypoint tasks using `ExecuteRobotTask`

- Excavator joint trajectories using `FollowJointTrajectory`

- Multiple independently namespaced excavators

- Sequential multi-robot scenarios

- Parallel scenario steps

- Conditional execution based on ROS 2 topics

- Direct topic publishing

- Software-only testing using mock dump trucks and simulated excavators

- Physical dump truck operation

- Physical excavator operation

- Mixed dump truck + excavator scenarios

- Multi-machine ROS 2 communication using Zenoh

---

# System Architecture

```text

                         Scenario YAML

                              │

                              ▼

                       Scenario Manager

                              │

              ┌───────────────┴────────────────┐

              │                                │

              ▼                                ▼

      ExecuteRobotTask                FollowJointTrajectory

              │                                │

              ▼                                ▼

       /truckN/...                    /excavatorN/...

              │                                │

              ▼                                ▼

         Dump Truck                       Excavator

              │                                │

       Physical / Mock                  Physical / SIM

```

The Command Center does not directly control motors.

Instead, it sends higher-level commands to robot-specific ROS 2 interfaces.

For physical multi-machine operation, the Command Center and robot computers communicate through one active Zenoh router.

The router can run on either `ros-pc` (primary) or `ros-backup-pc` (backup).

```text

                      ACTIVE ROUTER HOST

                   ros-pc OR ros-backup-pc

                              │

                         Zenoh Router

                              │

          ┌───────────────────┼───────────────────┐

          │                   │                   │

          ▼                   ▼                   ▼

      Dump Truck          Excavator 1         Excavator 3

          Pi                  Pi                  Pi

```

The intended architecture is:

```text

1 Active Router Host

      \+

1 Zenoh Router

      \+

1 Command Center

      \+

N Physical Robot Clients

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

│   │   ├── excavator_system.launch.py

│   │   ├── dump_truck_system.launch.py

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

This separates reusable ROS 2 software from operation-specific task definitions.

---

# 1. Scenario Manager

The main Command Center node is:

```bash

ros2 run construction_site_control scenario_manager_node

```

The Scenario Manager reads a YAML scenario and executes its steps.

Currently supported step types are:

```text

task

excavator_trajectory

wait

topic_publish

parallel

condition

```

A scenario can therefore coordinate different types of construction robots without requiring them to use the same low-level controller.

Sequential steps proceed only after the previous step completes successfully.

If an Action step reports failure, the Scenario Manager does not treat the step as successful and the sequential scenario is aborted.

---

# 2. Dump Truck Tasks

Dump trucks use the custom ROS 2 Action:

```text

construction_site_interfaces/action/ExecuteRobotTask

```

The Action interface for each truck is:

```text

/\<truck_name>/execute_robot_task

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

The `robot` field determines the target truck.

Waypoint files are stored under:

```text

operations/dump_truck/waypoints/

```

The physical Action Server uses the truck localization and control stack to execute the requested waypoint file.

A mock Action Server is also available for software-only integration testing.

---

# 3. Excavator Tasks

Excavators use the standard ROS 2 Action:

```text

control_msgs/action/FollowJointTrajectory

```

Each excavator runs inside its own ROS 2 namespace.

The standard Action interface is:

```text

/\<excavator_name>/upper_arm_controller/follow_joint_trajectory

```

For example:

```text

/excavator1/upper_arm_controller/follow_joint_trajectory

/excavator3/upper_arm_controller/follow_joint_trajectory

```

An excavator scenario step looks like:

```yaml

- id: excavator3_move

  type: excavator_trajectory

  robot: excavator3

  task_file: excavator3_excavation_cycle_test.yaml

  seconds_per_waypoint: 5.0

```

The `robot` field is used to resolve the target Action.

For example:

```yaml

robot: excavator3

```

resolves to:

```text

/excavator3/upper_arm_controller/follow_joint_trajectory

```

and:

```yaml

robot: excavator1

```

resolves to:

```text

/excavator1/upper_arm_controller/follow_joint_trajectory

```

An explicit `action_name` may be supplied in a scenario step when an override is required. Otherwise, the namespaced Action is derived automatically from `robot`.

Trajectory files are stored under:

```text

operations/excavator/trajectories/

```

Excavator trajectories may command all joints or only a subset of joints.

Joints not listed in the trajectory are not included in the `FollowJointTrajectory` goal.

This is useful for isolated joint testing and for operations that do not require every excavator joint.

---

# 4. Multi-Excavator Support

The Command Center supports independently namespaced excavators.

Conceptually:

```text

                         Scenario Manager

                                │

             ┌──────────────────┴──────────────────┐

             │                                     │

             ▼                                     ▼

        robot: excavator1                     robot: excavator3

             │                                     │

             ▼                                     ▼

/excavator1/upper_arm_controller/     /excavator3/upper_arm_controller/

   follow_joint_trajectory               follow_joint_trajectory

             │                                     │

             ▼                                     ▼

        Excavator 1                            Excavator 3

```

The robot name therefore serves as both:

- the logical robot identity used by the scenario

- the basis for resolving the excavator's ROS 2 Action namespace

This removes the previous single global excavator Action and allows multiple excavators to coexist on the same ROS 2 network.

---

# 5. Mixed-Robot Scenarios

A single scenario can coordinate dump trucks and excavators.

For example:

```yaml

scenario_name: truck1_excavator3_integration_test

steps:

  - id: excavator3_move

    type: excavator_trajectory

    robot: excavator3

    task_file: excavator3_excavation_cycle_test.yaml

    seconds_per_waypoint: 5.0

  - id: truck1_short_move

    type: task

    robot: truck1

    task_type: waypoint

    task_file: truck1_waypoints3.yaml

```

Because these steps are sequential, the execution flow is:

```text

Truck 1 Task

      │

      ▼

ExecuteRobotTask

      │

      ▼

SUCCESS

      │

      ▼

Excavator 3 Trajectory

      │

      ▼

/excavator3/upper_arm_controller/

follow_joint_trajectory

      │

      ▼

Action Result

```

The next sequential step begins only after the previous Action completes successfully.

A physical Truck 1 + Excavator 3 scenario has been used to verify this routing architecture.

---

# 6. Software-Only Integration Testing

The Command Center can be tested without physical robots.

A typical software-only configuration uses:

```text

Dump Truck  → Mock Action Server

Excavator   → SIM mode

```

This allows scenario orchestration to be tested independently of sensors, motors, Raspberry Pis, and physical calibration.

---

# 7. Build

From the repository root:

```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build \\

  --symlink-install \\

  --packages-select \\

    construction_site_interfaces \\

    dump_truck_action_server \\

    excavator_control \\

    construction_site_control

source install/setup.bash

```

A setuptools warning related to `pytest-repeat` may appear during the build.

If all packages finish successfully, this warning does not indicate a build failure.

---

# 8. Start an Excavator in SIM Mode

**Terminal 1**
```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 launch excavator_control \\

  excavator.launch.py \\

  mode:=sim \\

  robot_name:=excavator1

```

The expected node namespace is:

```text

/excavator1

```

The Action interface is:

```text

/excavator1/upper_arm_controller/follow_joint_trajectory

```

---

# 9. Start a Mock Dump Truck

**Terminal 2**
```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 run dump_truck_action_server \\

  mock_waypoint_action_server_node \\

  --ros-args \\

  -p truck_name:=truck1 \\

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

---

# 10. Run a Mixed-Robot Software Scenario

**Terminal 3**
```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 run construction_site_control \\

  scenario_manager_node \\

  --ros-args \\

  -p scenario:=truck1_excavator_cycle.yaml \\

  -p trucks:=truck1

```

The expected execution sequence is:

```text

SCENARIO START

      │

      ▼

Truck 1 Mock

      │

      ▼

SUCCESS

      │

      ▼

Excavator 1 SIM

      │

      ▼

EXCAVATOR SUCCESS

      │

      ▼

Next Step

      │

      ▼

SCENARIO COMPLETE

```

---

# 11. Direct Excavator Command Center Test

The Command Center contains a standalone excavator Action client.

For Excavator 3:

```bash

ros2 run construction_site_control \\

  excavator_task_client \\

  operations/excavator/trajectories/excavator3_excavation_cycle_test.yaml \\

  --robot excavator3 \\

  --seconds-per-waypoint 5.0

```

This bypasses the Scenario Manager and directly targets:

```text

/excavator3/upper_arm_controller/follow_joint_trajectory

```

The execution path is:

```text

Excavator Trajectory YAML

          │

          ▼

excavator_task_client

          │

          │ --robot excavator3

          ▼

/excavator3/upper_arm_controller/

follow_joint_trajectory

          │

          ▼

Excavator 3

```

An explicit Action name can also be supplied through the client's Action-name override when required.

---

# 12. Physical Robot Operation

The same high-level interfaces are used for simulation and physical operation.

For excavators:

```text

Command Center

      │

      │ FollowJointTrajectory

      ▼

/excavatorN/...

      │

      ▼

excavator_control

      │

      ├── SIM mode

      │

      └── PI mode

```

For dump trucks:

```text

Command Center

      │

      │ ExecuteRobotTask

      ▼

/truckN/execute_robot_task

      │

      ▼

dump_truck_action_server

      │

      ▼

dump_truck_control

      │

      ▼

Physical Dump Truck

```

The Command Center therefore remains independent of the low-level hardware implementation.

---

# 13. ROS 2 Network Architecture

Zenoh (`rmw_zenoh_cpp`) is the standard ROS 2 communication method for physical multi-machine operation.

The system uses ****one active Zenoh router****. The router can run on either:

- `ros-pc` — primary router host (`10.170.32.181`)

- `ros-backup-pc` — backup router host (`10.170.32.227`)

```text

                      ACTIVE ROUTER HOST

                   ros-pc OR ros-backup-pc

                              │

                         Zenoh Router

                              │

          ┌───────────────────┼───────────────────┐

          │                   │                   │

          ▼                   ▼                   ▼

      Dumptruck1          Excavator1          Excavator3

          Pi                  Pi                  Pi

```

The robot Raspberry Pis do not need to be configured as direct ROS peers of one another.

Each robot independently connects to the currently active router.

Network configuration is managed through:

```text

network/

├── devices.sh

├── setup_zenoh.sh

└── setup_network.sh

```

Normal operation uses:

```text

setup_zenoh.sh

```

The primary `ros-pc` router is the default. For example:

```bash

source network/setup_zenoh.sh client excavator3

```

is equivalent to:

```bash

source network/setup_zenoh.sh client excavator3 ros-pc

```

When `ros-backup-pc` is the active router, participating clients must select it explicitly:

```bash

source network/setup_zenoh.sh client excavator3 ros-backup-pc

```

Detailed network instructions are provided in:

```text

network/README.md

```

---

# 14. Standard Physical System Startup

The ROS PC has three normal system-level entry points:

```text

Excavator only  → excavator_system.launch.py

Dump truck only → dump_truck_system.launch.py

Integrated      → command_center.launch.py

```

The recommended physical system uses ****one active router host****.

## Primary Operation

```text

ros-pc

├── T1 → Zenoh Router

└── T2 → Command Center

Robot Raspberry Pis

└── T1 → Robot Hardware / Robot Server

```

## Backup Operation

```text

ros-backup-pc

├── T1 → Zenoh Router

└── T2 → Command Center

Robot Raspberry Pis

└── T1 → Robot Hardware / Robot Server

```

All participating robot computers must connect to the same active router.

---

## Primary ROS PC T1 — Zenoh Router

****Machine:**** `ros-pc`

****Keep this terminal running.****

```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh router ros-pc

ros2 run rmw_zenoh_cpp rmw_zenohd

```

The backward-compatible default is also valid:

```bash

source network/setup_zenoh.sh router

```

Only one router is normally required.

Do not start one router per robot.

---

## Primary-Router Robot Clients

When `ros-pc` is the active router, the default client configuration can be used.

### Dump Truck Raspberry Pi

Example for Truck 1:

```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

sudo pigpiod

ros2 launch dump_truck_bringup \

  dump_truck_pi.launch.py \

  truck_name:=truck1

```

The explicit network equivalent is:

```bash

source network/setup_zenoh.sh client dumptruck1 ros-pc

```

### Excavator Raspberry Pi

Example for Excavator 3:

```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client excavator3

sudo pigpiod

ros2 launch excavator_control \

  excavator.launch.py \

  mode:=pi \

  robot_name:=excavator3 \

  config:=$(ros2 pkg prefix excavator_control)/share/excavator_control/config/excavator3.yaml

```

The explicit network equivalent is:

```bash

source network/setup_zenoh.sh client excavator3 ros-pc

```

The Zenoh client profile configures the machine's network connection.

The launch argument:

```text

robot_name:=excavator3

```

configures the ROS 2 namespace.

These are separate configuration concepts and should normally use the corresponding robot identity.

> ****Excavator 3 Swing has been physically validated using overhead AprilTag feedback. Continue to use conservative motion and the configured physical safety limits.****

---

## Primary ROS PC T2 — Select the System Launch

****Keep this terminal running.****

Configure the ROS PC application terminal for the primary router:

```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client ros-pc

```

Use one of the following three system-level entry points.

### Excavator Only

```bash

ros2 launch construction_site_control \

  excavator_system.launch.py \

  excavators:=excavator3

```

This starts the shared overhead camera, AprilTag detector, and the selected excavator perception adapter. The excavator trajectory server runs on the excavator Raspberry Pi.

### Dump Truck Only

```bash

ros2 launch construction_site_control \

  dump_truck_system.launch.py \

  trucks:=truck1

```

For multiple dump trucks:

```bash

ros2 launch construction_site_control \

  dump_truck_system.launch.py \

  trucks:=truck1,truck3,truck4,truck5

```

This starts the shared overhead camera and AprilTag detector, the selected truck localization stacks, and the selected truck waypoint Action Servers.

### Integrated Dump Truck + Excavator

```bash

ros2 launch construction_site_control \

  command_center.launch.py \

  trucks:=truck1 \

  excavators:=excavator3 \

  start_scenario_manager:=false

```

The integrated launch owns the shared perception stack and starts the ROS-PC-side components required by the selected robots.

The physical excavator trajectory server remains on the excavator Raspberry Pi and is discovered through ROS 2 over Zenoh.

The lower-level launch files `overhead_camera.launch.py`, `excavator_perception.launch.py`, and `dump_truck_ros_pc.launch.py` are internal/debugging components and normally do not need to be launched directly.

---

## Backup ROS PC Operation

When `ros-backup-pc` is selected, `rmw_zenohd` must actually be started on the backup ROS PC.

### Backup ROS PC T1 — Zenoh Router

```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh router ros-backup-pc

ros2 run rmw_zenoh_cpp rmw_zenohd

```

****Keep this terminal running.****

### Backup ROS PC T2 — Select the System Launch

Configure the application terminal for the backup router:

```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client ros-backup-pc

```

Then use the same system-level entry points as on the primary ROS PC.

Excavator only:

```bash

ros2 launch construction_site_control \

  excavator_system.launch.py \

  excavators:=excavator3

```

Dump truck only:

```bash

ros2 launch construction_site_control \

  dump_truck_system.launch.py \

  trucks:=truck1

```

Integrated:

```bash

ros2 launch construction_site_control \

  command_center.launch.py \

  trucks:=truck1 \

  excavators:=excavator3 \

  start_scenario_manager:=false

```

### Backup-Router Robot Clients

Example for Truck 1:

```bash

source network/setup_zenoh.sh client dumptruck1 ros-backup-pc

```

Example for Excavator 3:

```bash

source network/setup_zenoh.sh client excavator3 ros-backup-pc

```

Every participating client must select `ros-backup-pc` while the backup router is active.

Selecting `ros-backup-pc` in `setup_zenoh.sh` does not remotely start a router on that machine. The router process must run on the physical backup ROS PC.

---

# 15. Verify Multi-Robot Communication

From a Zenoh-configured ROS PC terminal:

```bash

echo "===== NODES ====="

ros2 node list

echo

echo "===== ACTIONS ====="

ros2 action list

echo

echo "===== EXCAVATOR 3 ====="

ros2 action info \\

  /excavator3/upper_arm_controller/follow_joint_trajectory

echo

echo "===== EXCAVATOR 3 JOINT STATES ====="

ros2 topic info \\

  /excavator3/joint_states

```

For an active Excavator 3 server, the expected interfaces include:

```text

/excavator3/excavator_trajectory_server

/excavator3/upper_arm_controller/follow_joint_trajectory

/excavator3/joint_states

```

The Action should report one Action server.

---

# 16. Current Communication Status

Normal physical operation uses:

```text

RMW_IMPLEMENTATION=rmw_zenoh_cpp

```

Zenoh has been validated for:

- ROS 2 node and topic discovery

- dump truck communication

- dump truck `/wheel_states`

- dump truck `/cmd_vel`

- overhead camera communication

- AprilTag detection messages

- compressed camera image transport

- namespaced excavator `/joint_states`

- namespaced excavator `FollowJointTrajectory`

- excavator Action goal transport

- excavator Action feedback

- excavator Action results

- Command Center integration

- physical dump truck operation

- physical Truck 1 + Excavator 3 scenario routing

Zenoh is therefore the standard communication layer for the integrated system.

---

# 17. Running a Scenario Automatically

The Command Center can start the Scenario Manager automatically.

Before launching a physical scenario, configure the Command Center terminal for the currently active router.

## Primary ROS PC

For example, the Truck 1 + Excavator 3 integration scenario can be started with:

```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client ros-pc

ros2 launch construction_site_control command_center.launch.py \

  trucks:=truck1 \

  excavators:=excavator3 \

  start_scenario_manager:=true \

  scenario:=dtex_integration.yaml

```

## Backup ROS PC

When `ros-backup-pc` is the active router:

```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client ros-backup-pc

ros2 launch construction_site_control command_center.launch.py \

  trucks:=truck1 \

  excavators:=excavator3 \

  start_scenario_manager:=true \

  scenario:=dtex_integration.yaml

```

Scenario files are resolved from:

```text

operations/scenarios/

```

For new physical scenarios, it is generally preferable to first start the system with:

```text

start_scenario_manager:=false

```

and verify the intended robot interfaces before initiating physical motion.

---

# 18. Scenario Step Types

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

```yaml

- id: excavator3_move

  type: excavator_trajectory

  robot: excavator3

  task_file: excavator3_excavation_cycle_test.yaml

  seconds_per_waypoint: 5.0

```

Unless explicitly overridden, the Action is resolved from the robot name:

```text

/excavator3/upper_arm_controller/follow_joint_trajectory

```

---

## `wait`

Introduces a timed delay.

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

    - id: excavator3_move

      type: excavator_trajectory

      robot: excavator3

      task_file: excavator3_excavation_cycle_test.yaml

      seconds_per_waypoint: 5.0

```

The scenario continues after all parallel child steps complete successfully.

Different excavators can be represented by different `robot` names and therefore different Action namespaces.

---

## `condition`

Waits for a ROS topic value to satisfy a condition.

This allows scenario execution to depend on robot or system state.

Conceptually:

```text

Robot State Topic

      │

      ▼

Condition Evaluation

      │

      ▼

Scenario Execution

```

---

## `topic_publish`

Publishes directly to a supported ROS topic.

This can be used for:

- simple commands

- integration testing

- operations that do not require an Action abstraction

---

# 19. Action Success and Failure Behavior

The Scenario Manager waits for Action results before advancing sequential scenarios.

For excavator trajectories:

```text

Scenario Manager

      │

      ▼

FollowJointTrajectory Goal

      │

      ▼

Excavator Server

      │

      ├── SUCCESS

      │      │

      │      ▼

      │   Next Step

      │

      └── FAILURE

             │

             ▼

        Scenario Abort

```

This behavior has been physically tested with Excavator 3.

The namespaced Action successfully received the trajectory and returned continuous trajectory feedback through the Command Center.

When the physical excavator did not reach the final joint tolerance, the excavator server returned a trajectory failure and the Scenario Manager correctly aborted the scenario rather than reporting success.

Physical tracking or calibration failures should therefore be distinguished from Command Center routing failures.

---

# 20. Current Validation Status

## Verified

The following software and integration functionality has been tested:

```text

Excavator configuration loading

Excavator trajectory loading

Subset-joint trajectories

Trajectory limit validation

FollowJointTrajectory goal validation

Excavator SIM Action Server

Namespaced Excavator Actions

Namespaced Excavator joint-state topics

Excavator Action client

Command Center → Excavator SIM

Scenario Manager → Excavator SIM

Mock Dump Truck Action Server

Mixed Dump Truck + Excavator software scenario

Zenoh ROS 2 communication

Namespaced FollowJointTrajectory over Zenoh

Dump Truck ROS communication over Zenoh

Overhead Camera and AprilTag communication over Zenoh

Physical Dump Truck end-to-end operation over Zenoh

Physical Truck 1 + Excavator 3 Command Center routing

Excavator Action feedback propagation

Excavator Action failure propagation

Scenario abort following robot Action failure

```

Excavator 3 has completed physical coordinated trajectories using closed-loop Swing, Boom, Arm, and Bucket control.

Swing position is supplied by the overhead AprilTag system through `/excavator3/swing_joint_state`. The validated integration scenario runs the Excavator 3 excavation cycle and then executes the Truck 1 waypoint and dumping task.

---

## Remaining Excavator Hardware Work

Remaining Excavator 3 work includes:

- continued joint tracking and tolerance tuning

- continued coordinated multi-joint trajectory validation

- validation under changing payload and soil resistance

- final physical motion and safety validation

Swing sensing, Command Center routing, Zenoh communication, multi-joint excavation, and the Truck 1 integration sequence have been physically validated. These remaining items concern continued hardware tuning and operational robustness.

---

# 21. Recommended Development Workflow

Use the system in progressively higher-risk stages:

```text

1\. Unit Tests

      │

      ▼

2\. Excavator SIM

      │

      ▼

3\. Mock Truck + Excavator SIM

      │

      ▼

4\. Single Physical Robot

      │

      ▼

5\. Multi-Robot Physical Integration

      │

      ▼

6\. Integrated Construction Scenario

```

For new excavator trajectories:

1\. Inspect the trajectory YAML.

2\. Confirm the target excavator.

3\. Validate the trajectory against that machine's configuration.

4\. Confirm the intended joints.

5\. Confirm the physical workspace is clear.

6\. Begin with conservative physical motion.

For Excavator 3, Swing has been physically validated with overhead AprilTag feedback. Continue to use conservative trajectories and the configured physical safety limits.

---

# 22. Current Architecture

The current architecture supports:

```text

                         Operational YAML

                               │

                               ▼

                         Scenario Manager

                               │

             ┌─────────────────┴─────────────────┐

             │                                   │

             ▼                                   ▼

      ExecuteRobotTask                  FollowJointTrajectory

             │                                   │

             ▼                                   ▼

     /truckN/execute_robot_task        /excavatorN/upper_arm_controller/

                                          follow_joint_trajectory

             │                                   │

             ▼                                   ▼

         Dump Truck                           Excavator

             │                                   │

      Mock / Physical                     SIM / Physical

             │                                   │

             └─────────────────┬─────────────────┘

                               ▼

                          Zenoh Network

```

This architecture allows scenario logic to identify robots by name while robot-specific ROS 2 interfaces remain isolated by namespace.

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

> The Command Center coordinates robot tasks, while robot-specific packages remain responsible for executing physical motion.