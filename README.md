# CIC-ConRobotics-2026
ROS 2-based construction robotics platform for ****AE 573: Robotics and Automation in Construction, Fall 2026**** at Penn State.

This repository contains the software, configuration files, operational data, and lab materials used to control and coordinate physical construction robot models used in the course.

The platform currently supports:

- Multiple autonomous dump truck models

- Multiple independently namespaced robotic excavators

- Overhead camera-based AprilTag localization

- ROS 2-based robot control

- ROS 2 Actions for task execution

- Multi-robot coordination through a Command Center

- Waypoint-, trajectory-, and scenario-based construction operations

- Multi-machine ROS 2 communication using Zenoh

The repository is organized by ****system responsibility**** rather than ROS communication type. This makes it easier to understand where robot hardware, control logic, perception, operational data, and multi-robot coordination belong.

---
# 1. System Overview
The course platform represents a small-scale robotic construction site.

At a high level:

```text

                           OPERATIONS

                Waypoints / Trajectories / Scenarios

                               │

                               ▼

                        COMMAND CENTER

                               │

                          ROS 2 Actions

                               │

              ┌────────────────┴────────────────┐

              │                                 │

              ▼                                 ▼

         DUMP TRUCKS                       EXCAVATORS

              │                                 │

         Robot Control                      Joint Control

              │                                 │

              └────────────────┬────────────────┘

                               │

                             Zenoh

                               │

                               ▼

                       PHYSICAL ROBOTS



                    OVERHEAD PERCEPTION

                            │

                       AprilTags

                            │

                            ▼

                       LOCALIZATION

                            │

                            └──────► Robot Control

```

ROS 2 provides the software interfaces connecting perception, robot control, and higher-level task coordination.

Zenoh (`rmw_zenoh_cpp`) provides the standard ROS 2 communication layer between the active router host, ROS application terminals, and robot computers.

---
# 2. Repository Structure
```text

CIC-ConRobotics-2026/

│

├── robots/

│   ├── dump_truck/

│   │   ├── dump_truck_hardware/

│   │   ├── dump_truck_control/

│   │   └── dump_truck_bringup/

│   │

│   └── excavator/

│       └── excavator_control/

│

├── common/

│   └── construction_site_interfaces/

│

├── perception/

│   └── construction_robot_perception/

│

├── command_center/

│   ├── dump_truck_action_server/

│   └── construction_site_control/

│

├── operations/

│   ├── dump_truck/

│   │   └── waypoints/

│   ├── excavator/

│   │   └── trajectories/

│   └── scenarios/

│

├── network/

├── labs/

├── docs/

└── tools/

```

The general organization is:

```text

robots/

    Robot-specific hardware and control

common/

    Shared ROS interfaces

perception/

    Shared sensing and localization

command_center/

    High-level task and multi-robot coordination

operations/

    Waypoints, trajectories, and scenarios

network/

    ROS 2 multi-machine communication

labs/

    Student-facing lab activities

docs/

    Supporting technical documentation

tools/

    Utility and diagnostic scripts

```

---
# 3. Robots
```text

robots/

```

This directory contains ROS 2 packages associated with individual robot platforms.

Robot-specific hardware interfaces, control algorithms, machine configuration, and launch files belong here.

---
## 3.1 Dump Trucks
```text

robots/dump_truck/

├── dump_truck_hardware/

├── dump_truck_control/

└── dump_truck_bringup/

```

The dump truck platform is divided into three ROS 2 packages.

### `dump_truck_hardware`
Low-level interface to the physical dump truck.

This package contains nodes responsible for interacting directly with the Raspberry Pi and physical actuators.

Examples include:

- Motor control

- Wheel commands

- Dump bucket control

- Hardware-level robot behavior

Key nodes include:

```text

motor_drive_node.py

bucket_action_node.py

```

This package normally runs on the ****Raspberry Pi installed on the dump truck****.

### `dump_truck_control`
Robot-level motion and localization logic.

This package contains nodes for:

- Odometry

- AprilTag/odometry integration

- Waypoint navigation

- Robot motion control

Key nodes include:

```text

odometry_node.py

tag_odom_fusion_node.py

waypoint_controller_node.py

```

These components normally run on the ****ROS PC****.

Operational waypoint files are stored separately under:

```text

operations/dump_truck/waypoints/

```

### `dump_truck_bringup`
Launch files and robot-specific configuration.

This package provides launch configurations for starting the dump truck system on both the Raspberry Pi and ROS PC.

Hardware configurations are stored under:

```text

robots/dump_truck/dump_truck_bringup/config/hardware/

```

Current truck configurations include:

```text

truck1.yaml

truck3.yaml

truck4.yaml

truck5.yaml

```

Localization configuration is stored under:

```text

robots/dump_truck/dump_truck_bringup/config/localization/

```

---
## 3.2 Excavators
```text

robots/excavator/

└── excavator_control/

```

The excavator platform is integrated through the `excavator_control` package.

The package supports:

- Joint-state monitoring

- Joint trajectory execution

- Machine-specific configuration

- Sensor calibration

- Trajectory validation

- ROS 2 Action-based trajectory control

- Physical Raspberry Pi operation

- Simulation and software integration testing

- Command Center integration

- Multiple independently namespaced excavators

Excavators use the standard ROS 2:

```text

control_msgs/action/FollowJointTrajectory

```

Action.

Each excavator operates within its own ROS 2 namespace.

For example:

```text

/excavator1/upper_arm_controller/follow_joint_trajectory

/excavator1/joint_states

/excavator3/upper_arm_controller/follow_joint_trajectory

/excavator3/joint_states

```

This allows multiple excavators to coexist on the same ROS 2 network.

---
### Excavator Software
The primary package is:

```text

robots/excavator/excavator_control/

```

Its structure includes:

```text

excavator_control/

├── excavator_control/

│   ├── config_loader.py

│   ├── trajectory_loader.py

│   ├── validate_config.py

│   ├── validate_trajectory.py

│   ├── goal_validation.py

│   ├── excavator_trajectory_server.py

│   └── excavator_trajectory_client.py

│

├── config/

│   ├── excavator_template.yaml

│   ├── excavator1.yaml

│   └── excavator3.yaml

│

├── launch/

│   └── excavator.launch.py

│

└── test/

```

The trajectory server provides the same ROS 2 interface in both execution environments:

```text

                  FollowJointTrajectory

                           │

                           ▼

               Excavator Trajectory Server

                           │

                 ┌─────────┴─────────┐

                 │                   │

                 ▼                   ▼

             PI Mode              SIM Mode

                 │                   │

                 ▼                   ▼

         Physical Hardware      Joint Commands

```

The launch argument:

```text

robot_name:=excavatorN

```

determines the ROS 2 namespace.

For example:

```text

robot_name:=excavator3

```

creates interfaces under:

```text

/excavator3/

```

---
### Excavator Machine Configuration
Machine-specific configuration is stored under:

```text

robots/excavator/excavator_control/config/

```

For example:

```text

excavator1.yaml

excavator3.yaml

```

Machine configuration may contain:

- Joint limits

- Sensor calibration

- GPIO configuration

- Motor direction

- Motor control parameters

- Closed-loop control parameters

- Home configuration

Machine configuration is intentionally separated from operational trajectory data.

---
### Excavator Trajectories
Operational trajectory files are stored under:

```text

operations/excavator/trajectories/

```

Trajectory files describe ****what motion the excavator should perform****, while `excavator_control` defines ****how the machine executes that motion****.

The excavator joint model includes:

```text

swing

boom

arm

bucket

```

Trajectories may command all joints or only a subset.

Joints not listed in a trajectory are not included in the resulting `FollowJointTrajectory` request.

Trajectory validation can detect problems such as:

- Unknown joints

- Duplicate joints

- Invalid waypoint definitions

- Missing joint positions

- Joint targets outside configured limits

> **Excavator 3 swing status:** Swing has been physically validated using external AprilTag feedback published as a `JointState`. Physical commands are restricted to ±95 degrees, with ±105 degrees reserved as the observed hard safety range.

Detailed excavator instructions are provided in:

```text

robots/excavator/README.md

docs/operations/excavator3_truck1_integration.md

```

---
# 4. Common ROS Interfaces
```text

common/

└── construction_site_interfaces/

```

Shared ROS 2 interfaces used across the platform are stored here.

The package currently includes:

```text

construction_site_interfaces/

├── action/

│   └── ExecuteRobotTask.action

│

└── msg/

    └── RobotStatus.msg

```

Dump trucks use:

```text

construction_site_interfaces/action/ExecuteRobotTask

```

for high-level task execution.

Excavators use the standard:

```text

control_msgs/action/FollowJointTrajectory

```

interface and therefore do not require a custom excavator Action definition.

---
# 5. Perception and Localization
```text

perception/

└── construction_robot_perception/

```

Shared perception components belong here.

The current system uses an overhead camera and AprilTags to estimate robot positions within the model construction site.

The package includes configuration for:

- Overhead USB camera

- AprilTag detection

- Multi-truck tag definitions

Current configuration files include:

```text

config/

├── tags_multi_truck.yaml

├── usb_cam_obsbot.yaml

└── legacy_obsbot/

```

Additional AprilTag calibration documentation is available under:

```text

docs/perception/apriltag/

```

Utility scripts related to perception are stored under:

```text

tools/perception/

```

---
# 6. Command Center
```text

command_center/

├── dump_truck_action_server/

└── construction_site_control/

```

The Command Center provides higher-level coordination of robot tasks.

Instead of directly controlling motors, GPIO, or low-level velocity commands, it works with higher-level requests such as:

```text

Execute this dump truck waypoint task.

```

or:

```text

Execute this excavator trajectory.

```

or:

```text

Run this multi-robot construction scenario.

```

---
## 6.1 Dump Truck Actions
Each dump truck exposes:

```text

/<truck_name>/execute_robot_task

```

For example:

```text

/truck1/execute_robot_task

```

The dump truck Action Server resolves waypoint task files from:

```text

operations/dump_truck/waypoints/

```

---
## 6.2 Excavator Actions
Each excavator exposes a namespaced Action:

```text

/<excavator_name>/upper_arm_controller/follow_joint_trajectory

```

For example:

```text

/excavator3/upper_arm_controller/follow_joint_trajectory

```

In a scenario:

```yaml

- id: excavator3_move

  type: excavator_trajectory

  robot: excavator3

  task_file: excavator3_excavation_cycle_test.yaml

  seconds_per_waypoint: 5.0

```

the Scenario Manager resolves:

```yaml

robot: excavator3

```

to:

```text

/excavator3/upper_arm_controller/follow_joint_trajectory

```

This allows multiple excavators to be addressed independently.

---
## 6.3 Construction Scenarios
Scenario definitions are stored under:

```text

operations/scenarios/

```

The Scenario Manager currently supports:

```text

task

excavator_trajectory

wait

parallel

condition

topic_publish

```

Scenarios can therefore represent:

- Sequential robot execution

- Parallel robot execution

- Dump truck waypoint tasks

- Excavator trajectories

- Waiting

- Robot or topic conditions

- Direct supported topic commands

- Mixed robot operations

The goal is to separate ****what the construction operation should do**** from ****how each robot performs its task****.

Detailed Command Center documentation is provided in:

```text

command_center/README.md

```

---
# 7. Operations
```text

operations/

├── dump_truck/

│   └── waypoints/

├── excavator/

│   └── trajectories/

└── scenarios/

```

Operational data is intentionally separated from ROS 2 source packages.

The ROS packages define ****robot capabilities and system behavior****.

The `operations/` directory defines ****what the robots should do during a particular operation****.

---
## 7.1 Dump Truck Waypoints
```text

operations/dump_truck/waypoints/

```

Waypoint YAML files define navigation tasks for individual dump trucks.

Multiple waypoint files may exist for a truck to represent different routes or tasks.

---
## 7.2 Excavator Trajectories
```text

operations/excavator/trajectories/

```

Trajectory YAML files define excavator joint motions.

These files may command all supported joints or a subset of joints.

Machine-specific calibration remains under:

```text

robots/excavator/excavator_control/config/

```

---
## 7.3 Construction Scenarios
```text

operations/scenarios/

```

Scenario YAML files define higher-level construction operations involving one or more robots.

A scenario may specify:

- Which robot performs a task

- Which task or trajectory file should be executed

- Sequential operations

- Parallel operations

- Conditions

- Waiting behavior

This allows construction operations to be modified without rewriting robot-control code.

---
# 8. Network Configuration

```text
network/
├── devices.sh
├── setup_zenoh.sh
└── setup_network.sh
```

Normal physical multi-machine operation uses:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

through:

```text
network/setup_zenoh.sh
```

The system uses **one active Zenoh router**. The router can run on either:

- `ros-pc` — primary router host (`10.170.32.181`)
- `ros-backup-pc` — backup router host (`10.170.32.227`)

The standard architecture is:

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

Only one Zenoh router should normally be active at a time.

Device addresses are managed centrally through:

```text
network/devices.sh
```

The primary ROS PC is the default router. Normal operation can therefore use:

```bash
source network/setup_zenoh.sh router
source network/setup_zenoh.sh client ros-pc
source network/setup_zenoh.sh client dumptruck1
source network/setup_zenoh.sh client excavator3
```

The explicit equivalent is:

```bash
source network/setup_zenoh.sh router ros-pc
source network/setup_zenoh.sh client ros-pc
source network/setup_zenoh.sh client dumptruck1 ros-pc
source network/setup_zenoh.sh client excavator3 ros-pc
```

For backup-router operation, start the router **on the backup ROS PC** and configure every participating client for the same router:

```bash
# Run on ros-backup-pc
source network/setup_zenoh.sh router ros-backup-pc
ros2 run rmw_zenoh_cpp rmw_zenohd
```

```bash
# ROS applications running on ros-backup-pc
source network/setup_zenoh.sh client ros-backup-pc
```

```bash
# Examples on robot computers
source network/setup_zenoh.sh client dumptruck1 ros-backup-pc
source network/setup_zenoh.sh client excavator3 ros-backup-pc
```

Selecting `ros-backup-pc` in the setup script does not remotely start a router on that machine. `rmw_zenohd` must be started on the physical computer selected as the router host.

The previous DDS helper remains in the repository for specialized troubleshooting and development history, but normal physical operation uses Zenoh.

Detailed network instructions are provided in:

```text
network/README.md
```

---

# 9. Labs
```text

labs/

```

Course lab materials are stored here.

Students should follow the instructions for the specific lab rather than attempting to launch the entire repository at once.

---
# 10. Documentation
```text

docs/

```

Supporting technical documentation is stored here.

This includes material useful for operating, calibrating, or understanding the system that does not belong inside a ROS package.

---
# 11. Tools
```text

tools/

```

Utility and diagnostic scripts that do not belong to a specific ROS package are stored here.

For example:

```text

tools/perception/read_tag_tf.py

```

can be used for AprilTag-related diagnostics.

---
# 12. Software Environment
The Fall 2026 course environment is based on:

```text

Ubuntu 24.04

ROS 2 Jazzy

Python 3

```

Normal physical multi-machine communication uses:

```text

rmw_zenoh_cpp

```

---
# 13. Clone the Repository
Clone the repository:

```bash

cd \~

mkdir -p ws_conrobotics

cd ws_conrobotics

git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git

cd CIC-ConRobotics-2026

```

During active course development, the `dev` branch may be used:

```bash

git checkout dev

```

Pull the latest changes with:

```bash

git pull

```

---
# 14. Build the ROS 2 Workspace
From the repository root:

```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash

```

The workspace includes ROS 2 packages such as:

```text

construction_robot_perception

construction_site_control

construction_site_interfaces

dump_truck_action_server

dump_truck_bringup

dump_truck_control

dump_truck_hardware

excavator_control

```

Verify package discovery with:

```bash

colcon list

```

---
# 15. Rebuilding After Changes
For normal Python development:

```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash

```

If packages are moved or the workspace structure changes substantially, perform a clean build:

```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

rm -rf build install log

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash

```

Do not perform a clean build routinely unless it is necessary.

---
# 16. Standard Physical System Startup

Normal physical operation uses **one active router host**.

The primary configuration is:

```text
ros-pc
├── T1 → Zenoh Router
└── T2 → Perception / Localization / Command Center

Robot Raspberry Pis
└── T1 → Robot Hardware / Robot Server
```

If the system is intentionally operated from the backup computer:

```text
ros-backup-pc
├── T1 → Zenoh Router
└── T2 → Perception / Localization / Command Center

Robot Raspberry Pis
└── T1 → Robot Hardware / Robot Server
```

All participating physical robots must connect to the same active Zenoh router.

---

## 16.1 Primary ROS PC T1 — Zenoh Router

**Machine:** `ros-pc`

**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

The backward-compatible default:

```bash
source network/setup_zenoh.sh router
```

also selects `ros-pc`.

Only one router should normally be running. Do not start one router per robot.

---

## 16.2 Primary ROS PC T2 — Command Center

Example for operation with Truck 1:

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

The shorter default command is also valid:

```bash
source network/setup_zenoh.sh client ros-pc
```

For multiple dump trucks:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1,truck3,truck4,truck5 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

The overhead camera and AprilTag detector are shared across the site.

The excavator Action Server runs on the excavator Raspberry Pi and is discovered by the Command Center through ROS 2.

---

## 16.3 Primary-Router Robot Clients

When `ros-pc` is the active router, robot clients may use the default router selection.

### Dump Truck Raspberry Pi

Example for Dump Truck 1:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

The explicit equivalent is:

```bash
source network/setup_zenoh.sh client dumptruck1 ros-pc
```

Other dump trucks use the corresponding Zenoh device profile and launch file.

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

The Zenoh device profile configures network communication.

The launch argument:

```text
robot_name:=excavator3
```

sets the ROS 2 namespace.

The resulting interfaces include:

```text
/excavator3/excavator_trajectory_server
/excavator3/upper_arm_controller/follow_joint_trajectory
/excavator3/joint_states
```

> **Excavator 3 Swing is not currently validated and should not be commanded.**

---

## 16.4 Backup ROS PC Operation

When `ros-backup-pc` is used, the router must actually be started on the backup ROS PC.

### Backup ROS PC T1 — Zenoh Router

**Machine:** `ros-backup-pc`

**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-backup-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Backup ROS PC T2 — Command Center

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-backup-pc

ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

### Dump Truck Raspberry Pi

Example for Dump Truck 1:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1 ros-backup-pc

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

### Excavator Raspberry Pi

Example for Excavator 3:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client excavator3 ros-backup-pc

sudo pigpiod

ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi \
  robot_name:=excavator3 \
  config:=$(ros2 pkg prefix excavator_control)/share/excavator_control/config/excavator3.yaml
```

Every participating remote client must select `ros-backup-pc` while the backup router is active.

---

## 16.5 Multi-Robot Terminal Layout

A multi-robot system follows the same architecture regardless of which router-capable computer is selected:

```text
                       ACTIVE ROUTER HOST
                    ros-pc OR ros-backup-pc
                              │
                    ┌─────────┴─────────┐
                    │                   │
               Zenoh Router        Command Center
                    │                   │
          ┌─────────┼─────────┐         │
          │         │         │         │
          ▼         ▼         ▼         │
       Truck 1   Truck 3  Excavator 3 ◄─┘
          Pi        Pi         Pi
          │         │          │
          ▼         ▼          ▼
       Hardware  Hardware   Hardware
```

The important principle is:

```text
ONE ACTIVE ROUTER HOST
          +
    ONE Zenoh Router
          +
   ONE Command Center
          +
 N Physical Robot Clients
```

Adding another robot does not require another Zenoh router.

---

## 16.6 Run an Integrated Scenario

Before launching an integrated scenario, configure the application terminal for the currently active router.

### Primary ROS PC

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
  start_scenario_manager:=true \
  scenario:=dtex_integration.yaml
```

### Backup ROS PC

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-backup-pc

ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=true \
  scenario:=dtex_integration.yaml
```

The scenario contains robot-specific tasks such as:

```yaml
- id: truck1_short_move
  type: task
  robot: truck1
  task_type: waypoint
  task_file: truck1_waypoints3.yaml

- id: excavator3_move
  type: excavator_trajectory
  robot: excavator3
  task_file: excavator3_excavation_cycle_test.yaml
  seconds_per_waypoint: 5.0
```

The Scenario Manager resolves:

```text
robot: truck1
```

to the Truck 1 task interface and:

```text
robot: excavator3
```

to:

```text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

This physical Truck 1 + Excavator 3 routing architecture has been tested through the Command Center.

---

# 17. Operational Communication Flows
## 17.1 Dump Truck
```text

Overhead Camera

      │

      ▼

AprilTag Detection

      │

      ▼

Localization

      │

      ▼

/truckN/fused_odom

      │

      ▼

Waypoint Action Server

      │

      ▼

Waypoint Controller

      │

      ▼

/truckN/cmd_vel

      │

    Zenoh

      │

      ▼

Dump Truck Raspberry Pi

      │

      ▼

Motor Control

      │

      ▼

Physical Dump Truck

```

---
## 17.2 Excavator
```text

Scenario / Trajectory

      │

      ▼

Scenario Manager / Client

      │

      ▼

/excavatorN/upper_arm_controller/

follow_joint_trajectory

      │

    Zenoh

      │

      ▼

Excavator Raspberry Pi

      │

      ▼

Trajectory Server

      │

      ▼

Joint Control

      │

      ▼

Physical Excavator

```

---
# 18. Working With Operational YAML Files
Operational YAML files are stored outside ROS packages.

For dump truck waypoint tasks:

```text

operations/dump_truck/waypoints/

```

For excavator trajectories:

```text

operations/excavator/trajectories/

```

For multi-robot scenarios:

```text

operations/scenarios/

```

This keeps operational planning data separate from reusable ROS software and machine configuration.

---
# 19. Development Workflow
Development should generally follow:

```text

1\. Pull latest repository

        ↓

2\. Make changes

        ↓

3\. Build

        ↓

4\. Test

        ↓

5\. Review

        ↓

6\. Commit

        ↓

7\. Push

```

Before making changes:

```bash

git status

git pull

```

After making changes:

```bash

git status

git diff

```

Then commit:

```bash

git add .

git commit -m "Describe the change"

git push

```

Do not commit generated ROS 2 workspace directories:

```text

build/

install/

log/

```

---
# 20. Branches
The repository uses Git branches to separate stable course material from active development.

### `main`
Intended for stable, student-facing material.

### `dev`
Used for active development, integration, and testing before changes are promoted to `main`.

Students should use the branch specified by the instructor for each lab or activity.

---
# 21. Design Philosophy
The repository is organized around layers of responsibility.

```text

OPERATIONS

What should happen?

        │

        ▼

COMMAND CENTER

Which robot should perform which task?

        │

        ▼

ROBOT CONTROL

How should the robot move?

        │

        ▼

HARDWARE

How do we command the physical actuators?

```

Perception provides information about the physical environment and robot state across these layers.

```text

                 OPERATIONS

                     │

                     ▼

              COMMAND CENTER

                     │

                     ▼

PERCEPTION ───► ROBOT CONTROL

                     │

                     ▼

                  HARDWARE

```

This separation allows the same Command Center to coordinate different robot types and multiple instances of the same robot type.

For example:

```text

Truck 1

Truck 3

Truck 4

Truck 5

Excavator 1

Excavator 3

```

can coexist within the same system architecture while retaining robot-specific control and ROS 2 interfaces.

---
# 22. For Students
You do ****not**** need to understand every package in this repository before using the robots.

When working on a lab, focus on the part of the system relevant to that activity.

A useful mental model is:

```text

Need to change the physical robot?

    → robots/

Need localization or camera information?

    → perception/

Need shared ROS messages or Actions?

    → common/

Need to coordinate robot tasks?

    → command_center/

Need to change a route, trajectory, or scenario?

    → operations/

Need network configuration?

    → network/

Need lab instructions?

    → labs/

Need supporting documentation?

    → docs/

```

When in doubt, start with the README or instructions for the specific robot, system, or lab.

---
# 23. Current Project Status
This repository is under active development for ****Fall 2026****.

## Dump Trucks
The dump truck platform currently supports:

- Raspberry Pi-based hardware control

- Motor and bucket control

- Wheel-state feedback

- Odometry

- Overhead AprilTag-based localization

- Fused localization

- Waypoint navigation

- ROS 2 Action-based task execution

- Multiple dump truck configurations

- Multi-robot scenario coordination

---
## Excavators
The excavator platform currently supports:

- Machine-specific configuration

- Joint sensor calibration

- Joint-state feedback

- Joint trajectory execution

- Namespaced ROS 2 `FollowJointTrajectory` Actions

- Full-joint and subset-joint trajectory definitions

- Trajectory and configuration validation

- Physical Raspberry Pi execution

- Simulation/software integration

- Multiple excavator namespaces

- Command Center integration

- Mixed scenarios with dump trucks

Excavator 3 has been physically tested using coordinated closed-loop Swing, Boom, Arm, and Bucket control.

Swing feedback is generated from overhead AprilTag tracking and published through the packaged multi-excavator swing position adapter. The validated workflow completed a seven-waypoint excavation cycle followed by Truck 1 waypoint travel and dumping.

Final joint tuning and continued physical safety validation remain ongoing.

---
## Perception and Localization
The shared perception system currently supports:

- Overhead USB camera input

- AprilTag detection

- Multiple robot tag definitions

- Robot localization using AprilTag observations

- Integration with dump truck localization

---
## Network Communication

Normal multi-machine operation uses:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

with one active Zenoh router.

The router can run on either:

```text
ros-pc
```

or:

```text
ros-backup-pc
```

The primary `ros-pc` router remains the default configuration. The backup router can be selected explicitly through `network/setup_zenoh.sh`.

All participating clients must be configured for the same active router.

Zenoh has been used for:

- ROS 2 node and topic discovery
- Dump truck communication
- Camera and AprilTag communication
- Namespaced excavator joint-state communication
- Namespaced excavator Actions
- Command Center multi-robot integration

---

## Command Center
The Command Center currently supports:

- Individual robot tasks

- Dump truck waypoint tasks

- Excavator trajectory tasks

- Multiple independently addressed excavators

- Sequential execution

- Parallel execution

- Wait operations

- Topic-based conditions

- Robot-state conditions

- Direct supported topic commands

- Mixed dump truck and excavator scenarios

- Action feedback and result handling

- Scenario failure propagation

A physical Truck 1 + Excavator 3 scenario has been used to validate namespaced robot routing through the Command Center.

---
# 24. Course and Research Context
This platform is developed through the ****Computer Integrated Construction (CIC) Research Program at Penn State**** as part of educational and research activities in construction robotics.

The system is designed to provide students with hands-on experience integrating:

- Physical robots

- Sensors

- ROS 2

- Localization

- Robot control

- Task planning

- Multi-robot coordination

Rather than treating these topics independently, the model construction site provides a common physical environment in which students can see how individual robotics concepts connect to a complete robotic system.

---
# 25. Repository
****CIC-ConRobotics-2026****

Penn State  

Computer Integrated Construction (CIC) Research Program  

AE 573 — Robotics and Automation in Construction  

Fall 2026