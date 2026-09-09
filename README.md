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
```

This directory is reserved for the ROS 2 excavator control system.

The excavator architecture is being integrated into the same repository structure used by the dump truck system.

Excavator-specific code belongs under:

```text
robots/excavator/
```

while operational trajectory files belong under:

```text
operations/excavator/trajectories/
```

This separation keeps robot software independent from task-specific operational data.

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
```

The physical robots communicate with ROS PCs over a shared network.

This directory contains common network configuration utilities used by the course platform.

Current scripts include:

```text
devices.sh
setup_network.sh
```

Network configuration is treated as a shared system-level concern rather than belonging to a particular robot.

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

# 16. Typical ROS 2 Terminal Setup

Every new terminal used for the course should source ROS 2 and the workspace.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_conrobotics/CIC-ConRobotics-2026/install/setup.bash
```

Depending on the lab and network configuration, additional ROS networking environment variables may also be required.

Follow the instructions provided for the specific lab or robot.

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

The dump truck platform currently provides the most complete ROS 2 implementation, including:

- Hardware control
- Odometry
- Localization
- Waypoint navigation
- ROS 2 Action-based task execution
- Multi-robot scenario coordination

The excavator ROS 2 system is being integrated into the same architecture.

Additional robot capabilities, labs, documentation, and construction-site scenarios will continue to be added during development.

---

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
