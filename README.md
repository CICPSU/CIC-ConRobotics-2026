# CIC-ConRobotics-2026

Welcome to ROS 2-based construction robotics platform for **SITE (Systems Integration and Tehcnology Education) Robotics Arena in AE 573: Robotics and Automation in Construction, Fall 2026** at Penn State.

This repository contains the software, configuration, operational data, and course materials used to operate and coordinate physical construction robot models.

The platform includes:

- Multiple autonomous dump truck models
- Multiple independently namespaced robotic excavators
- Overhead camera and AprilTag-based perception/localization
- ROS 2 Action-based robot control
- Waypoint, trajectory, and multi-robot scenario execution
- Multi-machine ROS 2 communication using Zenoh

---

# 1. Start Here

For normal robot operation, use:

```text
command_center/README.md
```

That README is the **primary operational manual** for:

- starting the physical system
- launching the Command Center
- operating dump trucks
- operating excavators
- running trajectories and waypoint tasks
- creating and running multi-robot scenarios
- adding and validating new robots
- system-level troubleshooting and development

For network, SSH, Raspberry Pi setup, and Zenoh configuration, use:

```text
network/README.md
```

For a course lab, follow the instructions under:

```text
labs/
```

Do not try to operate the complete system from this root README.

---

# 2. System Overview

The platform represents a small-scale robotic construction site.

```text
                   OPERATIONS
          Waypoints / Trajectories / Scenarios
                         │
                         ▼
                  COMMAND CENTER
                         │
                    ROS 2 Actions
                         │
              ┌──────────┴──────────┐
              │                     │
              ▼                     ▼
         DUMP TRUCKS            EXCAVATORS
              │                     │
              └──────────┬──────────┘
                         │
                       Zenoh
                         │
                         ▼
                  PHYSICAL ROBOTS

               SHARED PERCEPTION
          Overhead Camera / AprilTags
                         │
                         ▼
                    LOCALIZATION
                         │
                         └──────► Robot Control
```

ROS 2 provides the software interfaces between perception, robot control, and higher-level coordination.

Zenoh (`rmw_zenoh_cpp`) is the standard ROS 2 communication layer for the physical multi-machine system.

---

# 3. Repository Structure

```text
CIC-ConRobotics-2026/
├── robots/
│   ├── dump_truck/
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
├── labs/
├── docs/
└── tools/
```

The repository is organized by **system responsibility**:

| Directory | Responsibility |
|---|---|
| `robots/` | Robot-specific hardware, control, configuration, and bringup |
| `common/` | Shared ROS 2 interfaces |
| `perception/` | Shared sensing and localization |
| `command_center/` | High-level robot task and multi-robot coordination |
| `operations/` | Waypoints, excavator trajectories, and scenarios |
| `network/` | Raspberry Pi setup, SSH, device registry, and Zenoh |
| `labs/` | Student-facing lab activities |
| `docs/` | Supporting technical documentation |
| `tools/` | Utility and diagnostic scripts |

---

# 4. Software Environment

The Fall 2026 platform uses:

```text
Ubuntu 24.04
ROS 2 Jazzy
Python 3
rmw_zenoh_cpp
```

Normal physical multi-machine communication uses **one active Zenoh router**.

Detailed network configuration belongs in:

```text
network/README.md
```

---

# 5. Clone the Repository

```bash
cd ~

mkdir -p ws_conrobotics
cd ws_conrobotics

git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git
cd CIC-ConRobotics-2026
```

## Branches

### `main`

Stable, course-ready material.

### `dev`

Active development, integration, and testing before changes are promoted to `main`.

Use the branch specified for the current course activity or development task.

To switch branches:

```bash
git checkout main
```

or:

```bash
git checkout dev
```

Then update the local repository:

```bash
git pull
```

> Branch-specific operating instructions belong **only in this root README**. Internal READMEs should remain branch-agnostic.

---

# 6. Build the ROS 2 Workspace

From the repository root:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Verify package discovery if needed:

```bash
colcon list
```

Do not commit generated workspace directories:

```text
build/
install/
log/
```

A clean build is not normally required. If the workspace structure changes substantially or a normal rebuild cannot resolve a build problem:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

rm -rf build install log

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

# 7. Operating the Robots

After cloning, building, and completing the required network setup, continue with:

```text
command_center/README.md
```

The normal operational architecture is:

```text
ONE active Zenoh router
        +
ONE Command Center
        +
N physical robot clients
```

The Command Center README contains the current commands for:

- ROS PC startup
- physical robot startup
- dump truck waypoint tasks
- excavator trajectories
- multi-robot scenarios
- individual robot validation
- adding new robots
- development and troubleshooting

Do **not** use older subsystem launch instructions elsewhere in the repository as the default operational workflow.

---

# 8. Key Documentation

## System Operation

```text
command_center/README.md
```

Primary manual for operating and developing the integrated construction robotics system.

## Network / Raspberry Pi / Zenoh

```text
network/README.md
```

Use for:

- preparing a new Raspberry Pi
- IoT network device registration
- SSH
- device addresses
- Zenoh router/client configuration
- ROS 2 multi-machine communication troubleshooting

## AprilTag Calibration

```text
docs/perception/apriltag/
```

Use for camera/AprilTag calibration and supporting perception procedures.

## Course Labs

```text
labs/
```

Students should follow the README or instructions for the specific lab.

---

# 9. Development Workflow

A typical development cycle is:

```text
Pull
  ↓
Modify
  ↓
Build
  ↓
Test
  ↓
Review
  ↓
Commit
  ↓
Push
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

Robot-specific development, validation, and instructions for adding new physical robots are documented in:

```text
command_center/README.md
```

---

# 10. Design Principle

The repository separates **what the construction operation should do** from **how each robot performs it**.

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
How are the physical actuators commanded?
```

Perception provides information about the physical environment and robot state across these layers.

Operational data therefore belongs under:

```text
operations/
```

while reusable robot software and machine-specific configuration remain under:

```text
robots/
```

---

# 11. For Students

You do **not** need to understand every package in this repository before using the robots.

Start with the instructions for your lab or activity.

A useful directory map is:

```text
Operate robots / run scenarios?
    → command_center/README.md

Set up a Pi / SSH / Zenoh?
    → network/README.md

Change robot hardware or machine configuration?
    → robots/

Change localization or perception?
    → perception/

Change a waypoint, trajectory, or scenario?
    → operations/

Complete a course lab?
    → labs/

Need supporting calibration documentation?
    → docs/
```

When in doubt, start with:

```text
command_center/README.md
```

---

# 12. Course and Research Context

This platform is developed through the **Computer Integrated Construction (CIC) Research Program at Penn State** for educational and research activities in construction robotics.

The model construction site provides hands-on experience integrating:

- physical robots
- sensors
- ROS 2
- localization
- robot control
- task planning
- multi-robot coordination

---

**CIC-ConRobotics-2026**  
Penn State  
Computer Integrated Construction (CIC) Research Program  
AE 573 — Robotics and Automation in Construction  
Fall 2026