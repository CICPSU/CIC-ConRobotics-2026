# CIC-ConRobotics-2026

ROS 2-based construction robotics platform for **AE 573: Robotics and Automation in Construction, Fall 2026** at Penn State.

This repository contains the software, configuration files, operational data, and lab materials used to control and coordinate physical construction robot models used in the course.

The platform currently supports:

- Multiple autonomous dump truck models
- A robotic excavator model
- Overhead camera-based AprilTag localization
- ROS 2-based robot control
- ROS 2 Actions for task execution
- Multi-robot coordination through a Command Center
- Waypoint- and scenario-based construction operations

The repository is organized by **system responsibility** rather than ROS communication type. This makes it easier to understand where robot hardware, control logic, perception, operational data, and multi-robot coordination belong.

---

# 1. System Overview

The course platform represents a small-scale robotic construction site.

At a high level:

```text
                 ┌─────────────────────────┐
                 │     Command Center      │
                 │                         │
                 │  Tasks and Scenarios    │
                 └────────────┬────────────┘
                              │
                         ROS 2 Actions
                              │
               ┌──────────────┴──────────────┐
               │                             │
        ┌──────▼──────┐               ┌──────▼──────┐
        │ Dump Trucks │               │  Excavator  │
        └──────┬──────┘               └──────┬──────┘
               │                             │
        Robot Control                 Robot Control
               │                             │
        Physical Robot                Physical Robot
               │
               │
        ┌──────▼──────┐
        │ Perception  │
        │ AprilTags   │
        │ Localization│
        └─────────────┘
```

ROS 2 provides the communication infrastructure connecting perception, robot control, and higher-level task coordination.

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
│
├── labs/
│
├── docs/
│
└── tools/
```

The major directories are described below.

---

# 3. Robots

```text
robots/
```

This directory contains ROS 2 packages associated with individual robot platforms.

Robot-specific hardware interfaces, control algorithms, and launch configurations belong here.

---

## 3.1 Dump Truck

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

This package normally runs on the **Raspberry Pi installed on the dump truck**.

---

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

This package normally runs on a **ROS PC** rather than directly on the Raspberry Pi.

Operational waypoint files are intentionally stored separately from the ROS package under:

```text
operations/dump_truck/waypoints/
```

---

### `dump_truck_bringup`

Launch files and robot-specific configuration.

This package provides launch configurations for starting the dump truck system on both:

- Raspberry Pi
- ROS PC

Hardware configurations are stored under:

```text
robots/dump_truck/dump_truck_bringup/config/hardware/
```

Current truck configuration files include:

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
## 3.2 Excavator

```text
robots/excavator/
└── excavator_control/
```

The excavator is integrated into the ROS 2 construction robotics platform through the `excavator_control` package.

The package provides the software required to operate the excavator in both physical and simulated environments.

The excavator control system includes:

- Joint-state monitoring
- Joint trajectory execution
- Machine-specific configuration
- Sensor calibration
- Trajectory validation
- ROS 2 Action-based trajectory control
- Physical Raspberry Pi operation
- Simulation and software integration testing
- Command Center integration

The excavator uses the standard ROS 2:

```text
control_msgs/action/FollowJointTrajectory
```

Action interface for trajectory execution.

This allows higher-level components such as the Command Center to request excavator motions without directly controlling motors or GPIO hardware.

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
│   └── excavator1.yaml
│
├── launch/
│   └── excavator.launch.py
│
└── test/
```

The trajectory server provides a common ROS 2 interface while supporting different execution environments.

Conceptually:

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

This allows the same higher-level trajectory interface to be used for physical robot operation and software integration testing.

---

### Machine Configuration

Machine-specific excavator configuration is stored under:

```text
robots/excavator/excavator_control/config/
```

For example:

```text
excavator1.yaml
```

contains configuration associated with the physical excavator, including:

- Joint limits
- Sensor calibration
- GPIO configuration
- Motor control parameters
- Control parameters
- Home configuration

A template configuration is also provided:

```text
excavator_template.yaml
```

Machine configuration is intentionally separated from operational trajectory data.

---

### Excavator Trajectories

Operational trajectory files are stored under:

```text
operations/excavator/trajectories/
```

rather than inside the ROS package.

Example structure:

```text
operations/excavator/trajectories/
├── excavator_trajectory_template.yaml
└── boom_small_test.yaml
```

Trajectory files describe **what motion the excavator should perform**, while the `excavator_control` package defines **how the excavator executes that motion**.

Trajectories may control all excavator joints or a subset of joints.

The current joint model includes:

```text
swing
boom
arm
bucket
```

For example, a trajectory may command only the boom while leaving the other joints outside that trajectory request.

Trajectory files can be validated before execution to detect problems such as:

- Unknown joints
- Duplicate joints
- Invalid waypoint definitions
- Missing joint positions
- Joint targets outside configured machine limits

This provides a safety and configuration-checking layer before trajectory execution.

---

### Command Center Integration

The excavator can participate in higher-level construction scenarios through the Command Center.

The communication structure is:

```text
Construction Scenario
        │
        ▼
Scenario Manager
        │
        ▼
Excavator Task Client
        │
        ▼
FollowJointTrajectory Action
        │
        ▼
Excavator Trajectory Server
        │
        ▼
Excavator
```

This allows dump truck tasks and excavator trajectories to be coordinated within the same construction-site scenario architecture.

Robot software therefore remains under:

```text
robots/excavator/
```

while operational trajectories remain under:

```text
operations/excavator/trajectories/
```

and multi-robot construction scenarios remain under:

```text
operations/scenarios/
```

This separation follows the same repository design philosophy used for the dump truck platform.

---

# 4. Common ROS Interfaces

```text
common/
└── construction_site_interfaces/
```

Shared ROS 2 interfaces used across the construction robotics platform are stored here.

The `construction_site_interfaces` package currently contains custom ROS 2 message and action definitions.

```text
construction_site_interfaces/
├── action/
│   └── ExecuteRobotTask.action
│
└── msg/
    └── RobotStatus.msg
```

These interfaces allow different robot systems and the Command Center to communicate using common task and status definitions.

Because other packages depend on these interfaces, this package is built before packages that use the custom messages and actions.

---

# 5. Perception and Localization

```text
perception/
└── construction_robot_perception/
```

Shared perception components belong here.

The current system uses an overhead camera and AprilTags to estimate the positions of robots within the model construction site.

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

Historical OBSBOT camera configuration files are retained under:

```text
config/legacy_obsbot/
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

Instead of directly controlling motors or publishing low-level velocity commands, the Command Center works with higher-level instructions such as:

```text
Execute this waypoint task.
```

or:

```text
Run this construction-site scenario.
```

---

## 6.1 Dump Truck Action Server

```text
command_center/dump_truck_action_server/
```

The dump truck action server connects high-level task requests with the dump truck control system.

The waypoint action server receives task requests and resolves the corresponding waypoint file from:

```text
operations/dump_truck/waypoints/
```

For example:

```text
truck1_waypoints.yaml
truck3_waypoints.yaml
truck4_waypoints.yaml
truck5_waypoints.yaml
```

This allows the Command Center to request a task without embedding waypoint data directly in the action request.

---

## 6.2 Construction Site Control

```text
command_center/construction_site_control/
```

This package provides higher-level construction-site coordination.

It includes the Scenario Manager, which can execute multi-step and multi-robot scenarios.

Scenario definitions are stored under:

```text
operations/scenarios/
```

Example scenarios include:

```text
truck1_then_truck3.yaml
truck1_truck3_parallel.yaml
truck1_3_4_5.yaml
truck5_then_truck4.yaml
```

Scenarios can represent operations such as:

- Sequential robot execution
- Parallel robot execution
- Waiting for another robot
- Robot status conditions
- ROS topic-based conditions
- Multi-robot task coordination

The goal is to separate **what the construction operation should do** from **how each individual robot performs its task**.

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

This distinction is important.

The ROS packages define **robot capabilities and system behavior**.

The `operations/` directory defines **what we want the robots to do during a particular operation**.

---

## 7.1 Dump Truck Waypoints

```text
operations/dump_truck/waypoints/
```

Waypoint YAML files define navigation tasks for individual dump trucks.

Current files include waypoint sets for:

```text
Truck 1
Truck 3
Truck 4
Truck 5
```

Multiple waypoint files may exist for a single truck to represent different tasks or routes.

---

## 7.2 Excavator Trajectories

```text
operations/excavator/trajectories/
```

This directory is used for excavator task and trajectory definitions.

Excavator operational sequences should be stored here rather than inside the excavator ROS package.

---

## 7.3 Construction Scenarios

```text
operations/scenarios/
```

Scenario YAML files define higher-level construction operations involving one or more robots.

A scenario may specify:

- Which robot performs a task
- Which task file should be executed
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

The physical robots communicate with the ROS PC over a shared network.

Network configuration is treated as a shared system-level responsibility rather than belonging to a particular robot.

The current platform uses **Zenoh (`rmw_zenoh_cpp`) as the primary ROS 2 communication method** between the ROS PC and robot computers.

The network directory contains:

| File | Purpose |
|---|---|
| `devices.sh` | Central registry of device names and assigned IP addresses |
| `setup_zenoh.sh` | Primary ROS 2 network configuration using Zenoh |
| `setup_network.sh` | DDS-based network configuration retained as an alternative/fallback |

Device IP addresses are defined centrally in:

```text
network/devices.sh
```

Users should normally not enter robot or ROS PC IP addresses manually.

Instead, configure each terminal using a device name.

For example:

```bash
source network/setup_zenoh.sh client ros-pc
source network/setup_zenoh.sh client dumptruck1
source network/setup_zenoh.sh client excavator1
```

The Zenoh router runs on the ROS PC and provides the communication backbone between the ROS PC and the robot computers.

The standard network architecture is:

```text
                    ROS PC
                       │
                 Zenoh Router
                       │
          ┌────────────┴────────────┐
          │                         │
          ▼                         ▼
    Dump Truck Pi              Excavator Pi
          │                         │
          ▼                         ▼
   Robot Hardware             Robot Hardware
```

The detailed startup procedure is provided in Section 16.

---

# 9. Labs

```text
labs/
```

Course lab materials are stored here.

For example:

```text
labs/lab01_ros_network_setup/
```

Labs provide student-facing instructions for configuring and operating the robotics platform.

Students should follow the instructions provided for the specific lab rather than attempting to launch the entire repository at once.

---

# 10. Documentation

```text
docs/
```

Supporting technical documentation is stored here.

This includes materials that are useful for operating, calibrating, or understanding the system but are not themselves ROS packages.

For example:

```text
docs/perception/apriltag/
```

contains documentation related to AprilTag and camera calibration.

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

Before using the repository, ROS 2 should be installed and configured on the computer.

Source ROS 2 with:

```bash
source /opt/ros/jazzy/setup.bash
```

---

# 13. Clone the Repository

Clone the repository:

```bash
cd ~
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
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
```

After the build completes:

```bash
source install/setup.bash
```

The workspace currently contains the following ROS 2 packages:

```text
construction_robot_perception
construction_site_control
construction_site_interfaces
dump_truck_action_server
dump_truck_bringup
dump_truck_control
dump_truck_hardware
```

You can verify package discovery with:

```bash
colcon list
```

---

# 15. Rebuilding After Changes

For normal Python development:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Because the workspace uses:

```text
--symlink-install
```

changes to Python source files can often be tested without repeatedly copying files into the install directory.

If packages are moved or the workspace structure changes substantially, perform a clean build:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

rm -rf build install log

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---
# 16. Standard System Startup

The recommended system configuration uses **Zenoh (`rmw_zenoh_cpp`)** for ROS 2 communication between the ROS PC and robot computers.

For normal operation, the system can be started using:

```text
ROS PC
├── Terminal 1 → Zenoh Router
└── Terminal 2 → Perception + Localization + Command Center

Robot Computer
└── Terminal 1 → Robot Hardware
```

This keeps the number of required terminals small while maintaining a clear separation between the communication infrastructure, ROS PC processes, and robot hardware.

Before starting the system, make sure the repository has been built and the latest workspace is available on each computer.

---

## 16.1 ROS PC Terminal 1 — Zenoh Router

**Machine:** ROS PC  
**Terminal:** T1  
**Keep this terminal running.**

Open a terminal on the ROS PC:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router

ros2 run rmw_zenoh_cpp rmw_zenohd
```

This terminal runs the Zenoh router used by the ROS PC and robot computers.

Do not close this terminal while operating the robotic system.

---

## 16.2 ROS PC Terminal 2 — Perception, Localization, and Command Center

**Machine:** ROS PC  
**Terminal:** T2  
**Keep this terminal running.**

For operation with Dump Truck 1:

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

This launch starts the ROS PC components required for Dump Truck 1 operation:

```text
Command Center
│
├── Overhead Camera
├── AprilTag Detection
├── Dump Truck Localization
└── Waypoint Action Server
```

The Scenario Manager is disabled in this example so that individual robot operation can be tested independently.

For coordinated construction scenarios, the Scenario Manager can be enabled when required.

---

## 16.3 Dump Truck 1 Terminal 1 — Robot Hardware

**Machine:** Dumptruck1 Raspberry Pi  
**Terminal:** T1  
**Keep this terminal running.**

Open a terminal on Dump Truck 1:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

This starts the hardware-side ROS 2 nodes for Dump Truck 1.

These nodes communicate with the ROS PC through the Zenoh router.

---

## 16.4 Standard Terminal Layout

For Dump Truck 1 operation, the complete terminal layout is:

```text
ROS PC
│
├── T1 — Zenoh Router
│
└── T2 — Command Center
         │
         ├── Overhead Camera
         ├── AprilTag Detection
         ├── Localization
         └── Waypoint Action Server


                    Zenoh
                      │
                      ▼


Dumptruck1 Raspberry Pi
│
└── T1 — Dump Truck Hardware
```

Therefore, normal Dump Truck 1 operation requires:

```text
ROS PC       → 2 terminals
Dumptruck1   → 1 terminal
```

Additional dump trucks follow the same architecture.

Each Raspberry Pi runs its own robot hardware nodes while the ROS PC provides shared perception, localization, and higher-level task coordination.

---
## 16.5 Operating Multiple Dump Trucks

The same Zenoh router is shared by all robot computers.

When multiple dump trucks are used, **do not start a separate Zenoh router for each robot**.

The system architecture becomes:

```text
                         ROS PC
                            │
                  ┌─────────┴─────────┐
                  │                   │
           Zenoh Router         Command Center
                  │                   │
                  │        ┌──────────┼──────────┐
                  │        │          │          │
                  │    Perception  Localization  Actions
                  │
        ┌─────────┼─────────┬─────────┐
        │         │         │         │
        ▼         ▼         ▼         ▼
     Truck 1   Truck 3   Truck 4   Truck 5
       Pi        Pi        Pi        Pi
```

All robot computers connect to the same Zenoh router running on the ROS PC.

---

### ROS PC Terminal 1 — Zenoh Router

**Machine:** ROS PC  
**Terminal:** T1  
**Keep this terminal running.**

Only one Zenoh router is required.

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router

ros2 run rmw_zenoh_cpp rmw_zenohd
```

---

### ROS PC Terminal 2 — Multi-Robot Command Center

**Machine:** ROS PC  
**Terminal:** T2  
**Keep this terminal running.**

For example, to operate Dump Trucks 1, 3, 4, and 5:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc

ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1,truck3,truck4,truck5 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

The Command Center uses the `trucks` argument to determine which dump truck systems should be started on the ROS PC.

For example:

```bash
trucks:=truck1
```

starts ROS PC components for Truck 1 only.

```bash
trucks:=truck1,truck3
```

starts ROS PC components for Trucks 1 and 3.

```bash
trucks:=truck1,truck3,truck4,truck5
```

starts ROS PC components for Trucks 1, 3, 4, and 5.

The overhead camera and AprilTag detector are shared across the construction site and therefore do not need to be started separately for each truck.

---

### Robot Terminals

Each physical dump truck runs its own hardware-side launch file on its Raspberry Pi.

#### Dump Truck 1

**Machine:** Dumptruck1 Raspberry Pi  
**Terminal:** T1  
**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

#### Dump Truck 3

**Machine:** Dumptruck3 Raspberry Pi  
**Terminal:** T1  
**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck3

ros2 launch dump_truck_bringup truck3_pi.launch.py
```

#### Dump Truck 4

**Machine:** Dumptruck4 Raspberry Pi  
**Terminal:** T1  
**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck4

ros2 launch dump_truck_bringup truck4_pi.launch.py
```

#### Dump Truck 5

**Machine:** Dumptruck5 Raspberry Pi  
**Terminal:** T1  
**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck5

ros2 launch dump_truck_bringup truck5_pi.launch.py
```

Each robot computer connects independently to the same Zenoh router.

The robot computers do not need to know the IP addresses of the other robots.

---

## 16.6 Multi-Robot Terminal Layout

For operation with three dump trucks, the recommended terminal layout is:

```text
ROS PC
│
├── T1 — Zenoh Router
│
└── T2 — Command Center
         │
         ├── Overhead Camera
         ├── AprilTag Detection
         ├── Truck 1 Localization + Action Server
         ├── Truck 3 Localization + Action Server
         ├── Truck 4 Localization + Action Server
         └── Truck 5 Localization + Action Server


                         Zenoh
                           │
          ┌────────────────┼────────────────┐
          │                │                │
          │                │                │
          ▼                ▼                ▼
     Dumptruck1        Dumptruck3       Dumptruck4       
     Raspberry Pi      Raspberry Pi     Raspberry Pi      
          │                │                │              
       T1 │             T1 │             T1 │              
          ▼                ▼                ▼             
      Hardware          Hardware         Hardware          
       Nodes             Nodes            Nodes            
```

The important principle is:

```text
ONE ROS PC
    │
    ├── ONE Zenoh Router
    │
    └── ONE Command Center
              │
              ├── Truck 1
              ├── Truck 3
              └── Truck 4

ONE hardware launch per physical robot
```

Adding another robot does **not** require another Zenoh router or another ROS PC terminal.

It only requires:

1. Adding the robot to the Command Center `trucks` argument.
2. Starting the corresponding hardware launch file on that robot's Raspberry Pi.

---

## 16.7 Adding the Excavator

The excavator uses the same Zenoh communication architecture.

The excavator computer connects to the existing Zenoh router:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client excavator1
```

The system therefore follows the same general architecture:

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

The Zenoh router acts as the shared communication backbone for the construction robotics platform.

Robot-specific startup procedures are documented in the corresponding robot directories.

---

## 16.8 Network Setup Summary

The network setup script identifies computers using device names rather than requiring users to manually enter IP addresses.

ROS PC application terminals use:

```bash
source network/setup_zenoh.sh client ros-pc
```

Dump truck computers use:

```bash
source network/setup_zenoh.sh client dumptruck1
source network/setup_zenoh.sh client dumptruck2
source network/setup_zenoh.sh client dumptruck3
source network/setup_zenoh.sh client dumptruck4
source network/setup_zenoh.sh client dumptruck5
```

Excavator computers use:

```bash
source network/setup_zenoh.sh client excavator1
```

The Zenoh router terminal uses:

```bash
source network/setup_zenoh.sh router
```

Users should normally **not manually set**:

```text
RMW_IMPLEMENTATION
ROS_STATIC_PEERS
ZENOH_CONFIG_OVERRIDE
```

The network setup scripts configure the required communication environment.

---

## 16.9 System Communication Flow

For a dump truck, the primary operational flow is:

```text
Overhead Camera
      │
      ▼
AprilTag Detection
      │
      ▼
 /detections
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

This architecture separates:

- Shared perception
- Robot-specific localization and control
- High-level task execution
- Network communication
- Physical robot hardware

The same communication backbone can support multiple construction robots simultaneously.

---

# 17. Working With Operational YAML Files

Operational YAML files are stored outside the ROS packages.

This is intentional.

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

For example, a dump truck task may reference:

```text
truck1_waypoints.yaml
```

The dump truck action server resolves the task to:

```text
operations/dump_truck/waypoints/truck1_waypoints.yaml
```

Similarly, the Scenario Manager resolves scenario names from:

```text
operations/scenarios/
```

This keeps operational planning data separate from reusable ROS software.

---

# 18. Development Workflow

Development should generally follow this workflow:

```text
1. Pull the latest repository
        ↓
2. Make changes on the appropriate branch
        ↓
3. Build the ROS 2 workspace
        ↓
4. Test the relevant robot/system
        ↓
5. Review the changes
        ↓
6. Commit
        ↓
7. Push
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

Do not commit generated ROS 2 workspace directories such as:

```text
build/
install/
log/
```

---

# 19. Branches

The repository uses Git branches to separate stable course material from active development.

### `main`

Intended for stable, student-facing material.

### `dev`

Used for active development, integration, and testing before changes are promoted to `main`.

Students should use the branch specified by the instructor for each lab or activity.

---

# 20. Design Philosophy

The repository is organized around several layers of responsibility.

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

This separation allows the platform to grow beyond a single robot.

For example, the same Command Center can coordinate multiple dump trucks and, as the platform develops, additional construction robots such as the excavator.

---

# 21. For Students

You do **not** need to understand every package in this repository before using the robots.

When working on a lab, focus on the part of the system relevant to that activity.

A useful mental model is:

```text
Need to change the physical robot?
    → robots/

Need localization or camera information?
    → perception/

Need shared ROS messages or actions?
    → common/

Need to coordinate robot tasks?
    → command_center/

Need to change a route or construction operation?
    → operations/

Need network configuration?
    → network/

Need lab instructions?
    → labs/

Need supporting documentation?
    → docs/
```

When in doubt, start with the README or instructions for the specific lab.

---

# 22. Project Status

This repository is under active development for **Fall 2026**.

The platform currently integrates multiple construction robot systems, shared perception, ROS 2 Actions, and higher-level multi-robot coordination within a common architecture.

---

## Dump Truck System

The dump truck platform currently supports:

- Raspberry Pi-based hardware control
- Motor and bucket control
- Wheel-state feedback
- Odometry
- Overhead AprilTag-based localization
- Fused robot localization
- Waypoint navigation
- ROS 2 Action-based task execution
- Multiple dump truck configurations
- Multi-robot scenario coordination

Operational waypoint definitions are stored separately from the reusable robot software under:

```text
operations/dump_truck/waypoints/
```

---

## Excavator System

The excavator platform currently supports:

- Machine-specific configuration
- Joint sensor calibration
- Joint-state feedback
- Joint trajectory execution
- ROS 2 `FollowJointTrajectory` Actions
- Full-joint and subset-joint trajectory definitions
- Trajectory and configuration validation
- Physical Raspberry Pi execution mode
- Simulation/software integration mode
- Command Center integration
- Scenario-based coordination with dump trucks

Operational excavator trajectories are stored under:

```text
operations/excavator/trajectories/
```

The excavator software architecture and ROS 2 communication interfaces are integrated into the repository.

Physical robot calibration and hardware behavior may continue to be refined independently from the higher-level ROS 2 architecture.

---

## Perception and Localization

The shared perception system currently supports:

- Overhead USB camera input
- AprilTag detection
- Multiple robot tag definitions
- Robot localization using AprilTag observations
- Integration of perception data with dump truck localization

The overhead perception system can provide shared observations for multiple robots operating within the model construction site.

---

## Network Communication

The platform uses **Zenoh (`rmw_zenoh_cpp`) as the primary ROS 2 communication method** between the ROS PC and robot computers.

The network architecture supports:

```text
One ROS PC
     │
     └── One Zenoh Router
              │
              ├── Dump Truck 1
              ├── Dump Truck 2
              ├── Dump Truck 3
              ├── Dump Truck 4
              ├── Dump Truck 5
              └── Excavator
```

Device addresses are managed centrally through:

```text
network/devices.sh
```

and normal Zenoh configuration is performed through:

```text
network/setup_zenoh.sh
```

The existing DDS-based network configuration is retained as an alternative/fallback.

---

## Command Center

The Command Center currently supports higher-level robot coordination using ROS 2 Actions and scenario definitions.

Supported scenario concepts include:

- Individual robot tasks
- Dump truck waypoint tasks
- Excavator trajectory tasks
- Sequential execution
- Parallel execution
- Wait operations
- Topic-based conditions
- Robot-state conditions
- Mixed dump truck and excavator scenarios

Scenario definitions are stored under:

```text
operations/scenarios/
```

This allows construction operations to be changed independently from the underlying robot-control software.

---

## Current Platform Architecture

At a high level, the current platform is:

```text
                         OPERATIONS
                             │
                 Waypoints / Trajectories
                       / Scenarios
                             │
                             ▼
                      COMMAND CENTER
                             │
                       ROS 2 Actions
                             │
              ┌──────────────┴──────────────┐
              │                             │
              ▼                             ▼
         DUMP TRUCKS                    EXCAVATOR
              │                             │
         Robot Control                Joint Control
              │                             │
              └──────────────┬──────────────┘
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

The platform is continuing to evolve as additional robot capabilities, course labs, operational scenarios, and documentation are developed for Fall 2026.

# 23. Course and Research Context

This platform is developed through the **Computer Integrated Construction (CIC) Research Program at Penn State** as part of educational and research activities in construction robotics.

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

# 24. Repository

CIC-ConRobotics-2026  
Penn State  
Computer Integrated Construction (CIC) Research Program  
AE 573 — Robotics and Automation in Construction  
Fall 2026
