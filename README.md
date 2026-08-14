# CIC-ConRobotics-2026

## Overview

This repository contains the software, simulation environments, and documentation developed for the **CIC ConRobotics course (AE 573), Fall 2026**.

The project focuses on the integration of construction robotics, ROS 2, robot simulation, localization, and multi-robot coordination using both physical scale-model construction robots and simulated construction equipment.

The primary software environment is:

- **ROS 2 Jazzy**
- **Ubuntu 24.04**
- **NVIDIA Isaac Sim 5.1.0**

The repository currently supports development and experimentation with:

- Scale-model dump trucks
- Scale-model excavators
- Full-scale excavator simulation
- AprilTag-based localization
- Odometry and localization fusion
- ROS 2 topic-based robot control
- ROS 2 action-based multi-robot coordination
- Multi-robot construction operation scenarios


---

# 1. System Architecture

The repository is organized around several layers of the construction robotics system.

Conceptually:

```text
                    Construction Robot System
                              │
              ┌───────────────┴───────────────┐
              │                               │
       Physical Robots                  Isaac Sim
              │                               │
              └───────────────┬───────────────┘
                              │
                            ROS 2
                              │
              ┌───────────────┴───────────────┐
              │                               │
      Topic-Based Control            Action-Based Control
              │                               │
      Low-Level Robot Control        Multi-Robot Coordination
      Hardware Testing               Task Sequencing
      Localization                   Synchronization
      Odometry                       Construction Scenarios
```

The **topic-based control layer** provides direct access to individual robot functions and remains useful for development, testing, calibration, localization, and diagnostics.

The **action-based command center** provides a higher-level coordination layer for executing construction tasks involving multiple robots.


---

# 2. Repository Structure

The repository currently contains the following major components:

```text
CIC-ConRobotics-2026/
│
├── AprilTag_Localization/
│
├── docs/
│   ├── ROS2/
│   └── isaac_sim/
│
├── dumptruck_ros2_python_based/
│
├── model_excavator_IsaacSim/
│
├── network/
│
├── ros2_action_based_command_center/
│
├── ros2_topic_based_control/
│
├── runtime_logs/
│
├── zx200_digging_stack_IsaacSim/
│
├── .gitattributes
├── .gitignore
└── README.md
```

> The repository structure may continue to evolve during development.


---

# 3. Main Components

## 3.1 ROS 2 Topic-Based Robot Control

Directory:

```text
ros2_topic_based_control/
```

This directory contains the primary low-level ROS 2 control stack for the physical construction robots.

It includes functionality for:

- dump truck hardware control
- motor control
- bucket control
- wheel encoder processing
- odometry
- AprilTag localization integration
- fused odometry
- waypoint control
- individual robot testing
- multi-robot testing
- hardware diagnostics

The current physical dump truck fleet includes:

```text
Truck 1
Truck 3
Truck 4
Truck 5
```

Hardware-specific differences between trucks are handled through per-truck YAML configuration files.

These parameters include, for example:

- AprilTag IDs
- bucket servo positions
- localization parameters
- odometry parameters
- encoder mapping
- left/right encoder swapping

This allows the robots to share the same ROS 2 node implementations while compensating for differences in physical hardware configuration.

For detailed setup, configuration, launch procedures, diagnostics, and testing:

**[ROS 2 Topic-Based Control README](ros2_topic_based_control/README.md)**


---

## 3.2 ROS 2 Action-Based Command Center

Directory:

```text
ros2_action_based_command_center/
```

The Action-Based Command Center provides the higher-level coordination layer for the construction robots.

Instead of directly publishing individual velocity commands, the command center coordinates robot tasks through ROS 2 actions.

Current capabilities include:

- robot task execution
- multi-step task sequences
- multi-robot coordination
- synchronized operations
- parallel task execution
- waypoint-based robot movement
- construction operation scenarios

The command center is designed to sit above the topic-based control layer:

```text
Action-Based Command Center
          │
          ▼
ROS 2 Actions / Task Logic
          │
          ▼
Topic-Based Robot Control
          │
          ▼
Physical Robot Hardware
```

The topic-based control stack therefore remains the primary low-level interface, while the command center provides task-level coordination.

For architecture, launch instructions, available actions, scenario definitions, and examples:

**[ROS 2 Action-Based Command Center README](ros2_action_based_command_center/README.md)**


---

## 3.3 AprilTag Localization

Directory:

```text
AprilTag_Localization/
```

AprilTag localization is used to provide external position and orientation information for the physical robots.

An overhead camera observes AprilTags mounted on the robots.

The localization system is used together with wheel odometry to provide robot pose information in a global coordinate frame.

Conceptually:

```text
Overhead Camera
      │
      ▼
AprilTag Detection
      │
      ▼
Robot Global Pose
      │
      ├─────────────┐
      │             │
      ▼             ▼
Tag Pose      Wheel Odometry
      │             │
      └──────┬──────┘
             ▼
        Fused Odometry
             │
             ▼
     Waypoint / Task Control
```

Robot-specific AprilTag IDs and localization parameters are configured separately for each truck.


---

## 3.4 Model Excavator Simulation

Directory:

```text
model_excavator_IsaacSim/
```

This directory contains the Isaac Sim environment and ROS 2 control components for the scale-model excavator.



---

## 3.5 Full-Scale Excavator Simulation

Directory:

```text
zx200_digging_stack_IsaacSim/
```

This directory contains simulation and ROS 2 components associated with the full-scale **Hitachi ZX200 excavator**.


---

## 3.6 Dump Truck Python Prototype

Directory:

```text
dumptruck_ros2_python_based/
```

This directory contains earlier Python-based ROS 2 development for the model dump trucks.

Some components in this directory represent earlier prototypes used during development of the current ROS 2 control architecture.

For current multi-truck operation, refer primarily to:

```text
ros2_topic_based_control/
```

and:

```text
ros2_action_based_command_center/
```


---

## 3.7 Network Configuration

Directory:

```text
network/
```

This directory contains network-related configuration and utilities used for ROS 2 communication between:

- ROS computers
- Raspberry Pi controllers
- physical robots
- other networked devices

Network configuration is particularly important when operating multiple robots simultaneously.


---

# 4. Physical Dump Truck Architecture

The current dump truck system uses a distributed ROS 2 architecture.

Each physical truck contains a Raspberry Pi responsible for hardware-level operation.

Conceptually:

```text
ROS Computer
     │
     │ ROS 2
     ▼
Raspberry Pi
     │
     ├── Motor Control
     ├── Bucket Control
     ├── Encoder Reading
     └── Wheel State Publishing
```

The ROS computer performs higher-level processing such as:

```text
Wheel States
     │
     ▼
Odometry
     │
     ├───────────────┐
     │               │
     ▼               ▼
Wheel Odom      AprilTag Pose
     │               │
     └───────┬───────┘
             ▼
        Fused Odometry
             │
             ▼
       Robot Controller
             │
             ▼
           cmd_vel
             │
             ▼
      Physical Truck
```

The current multi-truck configuration supports:

```text
Truck 1
Truck 3
Truck 4
Truck 5
```

Each robot uses its own ROS 2 namespace:

```text
/truck1/
/truck3/
/truck4/
/truck5/
```

This allows multiple robots to operate simultaneously while using common node implementations.


---

# 5. Multi-Robot Control

The repository supports coordinated operation of multiple construction robots.

For the dump truck fleet, a typical architecture is:

```text
                  Command Center
                       │
       ┌───────────────┼───────────────┐
       │               │               │
       ▼               ▼               ▼
    Truck 1         Truck 3         Truck 4       
       │               │               │          
       ▼               ▼               ▼          
   Controller      Controller      Controller     
       │               │               │               
       ▼               ▼               ▼               
    cmd_vel          cmd_vel          cmd_vel          
       │               │               │               
       ▼               ▼               ▼               
 Physical Truck   Physical Truck   Physical Truck  
```

Robot-specific hardware differences are handled through configuration rather than separate source-code implementations.

This allows the same control architecture to be deployed across multiple physical robots.


---

# 6. Branch Policy

The repository uses the following branch structure:

- `main` — stable branch intended primarily for students and course exercises
- `dev` — active development and integration branch used by the development team

Development changes should generally be tested on `dev` before being merged into `main`.


---

# 7. Clone the Repository

## 7.1 Go to Your Home Directory

```bash
cd ~
```

## 7.2 Create a Workspace Directory

You can choose the workspace name.

For example:

```bash
mkdir -p ~/ws_conrobotics
cd ~/ws_conrobotics
```

## 7.3 Clone the Repository

```bash
git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git
```

After cloning:

```text
~/ws_conrobotics/
└── CIC-ConRobotics-2026/
```

Move into the repository:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
```

### Important

Each user or development group should clone the repository inside their own home directory.

Do **not** clone the repository into shared system directories such as:

```text
/opt
/usr
```

unless specifically required by the system configuration.


---
# 8. Build the ROS 2 System

Several components in this repository are implemented as ROS 2 packages and must be built before they can be used.

For first-time setup, the recommended approach is to build the main ROS 2 packages together from the root of this repository.

> **For new ROS 2 users:**  
> Building and running are separate steps.
>
> The basic workflow is:
>
> ```text
> Clone repository
>       ↓
> Build ROS 2 packages
>       ↓
> Source the workspace
>       ↓
> Run the system
> ```
>
> You normally need to perform the full build only during the initial setup or after major updates to the repository.

---

## 8.1 Initial Full Build

First, source ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Move to the repository:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
```

Then build the main ROS 2 packages used by the current system:

```bash
colcon build \
  --symlink-install \
  --packages-select \
  construction_robot_perception \
  dump_truck_hardware \
  dump_truck_control \
  dump_truck_bringup \
  construction_site_interfaces \
  dump_truck_action_server \
  construction_site_control
```

This build includes the packages used for:

- overhead camera and AprilTag perception
- dump truck hardware interfaces
- motor and encoder handling
- wheel odometry
- AprilTag-based localization and odometry fusion
- dump truck bringup
- shared ROS 2 interfaces
- dump truck Action Servers
- multi-robot Command Center control

Wait until the build finishes.

A successful build should complete without any package reporting:

```text
Failed
```

---

## 8.2 Source the Built Workspace

After the build completes, source the workspace:

```bash
source install/setup.bash
```

This step is important.

`colcon build` creates the ROS 2 packages inside the workspace, but the current terminal must still be told where to find them.

You can verify that the main packages are available with:

```bash
ros2 pkg list | grep -E \
'construction_robot_perception|dump_truck|construction_site'
```

If the expected packages appear, the ROS 2 workspace is ready.

---

## 8.3 Complete First-Time Build Command

For first-time users, the entire procedure can be copied and executed from start to finish:

```bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Move to the repository
cd ~/ws_conrobotics/CIC-ConRobotics-2026

# Build the main ROS 2 packages
colcon build \
  --symlink-install \
  --packages-select \
  construction_robot_perception \
  dump_truck_hardware \
  dump_truck_control \
  dump_truck_bringup \
  construction_site_interfaces \
  dump_truck_action_server \
  construction_site_control

# Source the newly built workspace
source install/setup.bash

# Verify that the packages are available
ros2 pkg list | grep -E \
'construction_robot_perception|dump_truck|construction_site'
```

For a normal first-time setup, use this complete build rather than performing multiple separate builds from the subsystem READMEs.

---

## 8.4 Opening a New Terminal

You do **not** need to rebuild the ROS 2 packages every time you open a new terminal.

However, every new terminal must source both ROS 2 and the built workspace.

Run:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source install/setup.bash
```

After this, the terminal is ready to use the ROS 2 packages in this repository.

> **Remember:**  
>
> ```text
> New terminal
>      ↓
> source ROS 2
>      ↓
> source this workspace
>      ↓
> Ready to run
> ```
>
> Opening a new terminal does **not** require another build.

---

## 8.5 When Is Another Build Required?

You should rebuild when you modify files that are part of a ROS 2 package, such as:

- Python ROS 2 nodes
- launch files
- ROS 2 package configuration
- package definitions
- ROS 2 interfaces
- other files installed by the package

You may also need to rebuild after pulling significant updates from GitHub.

You normally do **not** need to rebuild simply because:

- you opened a new terminal
- you restarted a Raspberry Pi
- you restarted the ROS computer
- you stopped and restarted a ROS 2 node

For normal operation, sourcing the existing workspace is sufficient.

---

## 8.6 Development and Package-Specific Builds

The full build above is intended primarily for initial setup.

During development, rebuilding every package after every small change is unnecessary.

The subsystem documentation therefore provides smaller, package-specific build commands when appropriate.

For example, if you are working only on the dump truck control stack, you may rebuild only the affected dump truck packages rather than rebuilding the entire system.

Follow the relevant subsystem README for these development workflows.

### Topic-Based Robot Control

See:

**[ROS 2 Topic-Based Control](ros2_topic_based_control/README.md)**

This documentation covers:

- individual robot control
- Raspberry Pi setup
- per-truck hardware configuration
- motor control
- encoder configuration
- odometry
- AprilTag localization
- fused odometry
- waypoint control
- hardware testing and diagnostics
- multi-truck low-level testing

### Action-Based Multi-Robot Control

See:

**[ROS 2 Action-Based Command Center](ros2_action_based_command_center/README.md)**

This documentation covers:

- ROS 2 Action Servers
- robot task execution
- task sequencing
- synchronized operations
- multi-robot coordination
- construction operation scenarios

---

## 8.7 Recommended Order for New Users

If you are using this repository for the first time, follow this order:

```text
1. Clone the repository
        ↓
2. Perform the Initial Full Build
        ↓
3. Source the workspace
        ↓
4. Follow the Topic-Based Control README
        ↓
5. Verify individual robot operation
        ↓
6. Verify odometry and localization
        ↓
7. Follow the Action-Based Command Center README
        ↓
8. Run coordinated multi-robot operations
```

Do not start with the Command Center before verifying that the individual robots can be controlled correctly.

The topic-based control layer serves as the primary low-level testing and diagnostic interface for the robots.


---

# 9. Recommended Workflow

For development and debugging of individual robots:

```text
Hardware
   ↓
Wheel States
   ↓
Odometry
   ↓
Localization
   ↓
Fused Odometry
   ↓
cmd_vel Test
   ↓
Waypoint Control
```

Use:

**[ros2_topic_based_control](ros2_topic_based_control/README.md)**

for these low-level tests.

Once individual robot operation has been validated, use:

**[ros2_action_based_command_center](ros2_action_based_command_center/README.md)**

for multi-robot task execution and coordinated construction scenarios.

This separation makes it possible to diagnose individual hardware and localization problems without involving the higher-level command center.


---

# 10. Development Philosophy

This repository is intended as both a **research/development platform** and an **educational robotics environment**.

The software architecture therefore emphasizes:

- modular ROS 2 nodes
- reusable control logic
- hardware-specific YAML configuration
- simulation-to-physical robot workflows
- direct access to low-level robot interfaces
- higher-level multi-robot coordination
- transparent and reproducible robot experiments

The goal is to allow students and researchers to move between:

```text
(Simulation) optional
    ↕
ROS 2 Control
    ↕
Physical Construction Robots
```

while maintaining a common control architecture.


---

# 11. Documentation

Additional documentation is available throughout the repository.

### ROS 2

```text
docs/ROS2/
```

### Isaac Sim

```text
docs/isaac_sim/
```

### Topic-Based Robot Control

**[ros2_topic_based_control/README.md](ros2_topic_based_control/README.md)**

### Action-Based Multi-Robot Control

**[ros2_action_based_command_center/README.md](ros2_action_based_command_center/README.md)**


---

# 12. Current Development Status

The repository is under active development for the **Fall 2026 CIC ConRobotics course**.

Current major capabilities include:

- ROS 2 control of physical scale-model construction robots
- Four-truck ROS 2 operation
- Per-truck hardware configuration
- Wheel encoder-based odometry
- AprilTag-based global localization
- Fused robot odometry
- Waypoint-based navigation
- ROS 2 action-based robot tasks
- Multi-robot task sequencing
- Coordinated construction operation scenarios
- Isaac Sim integration for construction robot simulation

Additional components and documentation will continue to be added as the project develops.