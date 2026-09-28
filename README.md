# CIC-ConRobotics-2026

> **Development Branch**
>
> This is the `dev` version of the CIC-ConRobotics-2026 repository.
> Active development, hardware testing, integration, and validation are performed on this branch.
>
> **Students should use the `main` branch only.**

Welcome to the ROS 2-based construction robotics platform for **SITE (Systems Integration and Technology Education) Robotics Arena in AE 573: Robotics and Automation in Construction, Fall 2026** at Penn State.

This repository contains the software, configuration, operational data, and course materials used to operate and coordinate physical construction robot models.

The platform includes:

- Multiple autonomous dump truck models
- Multiple independently namespaced robotic excavators
- Overhead camera and AprilTag-based perception/localization
- ROS 2 Action-based robot control
- Waypoint, trajectory, and multi-robot scenario execution
- Multi-machine ROS 2 communication using Zenoh

---

# 1. Branch Policy

This repository uses two primary branches with different purposes.

| Branch | Purpose | Primary Users |
|---|---|---|
| `main` | Stable, course-ready release | AE 573 students |
| `dev` | Development, testing, integration, and validation | Development team |

## `main`

`main` is the stable version used for course activities.

Students should interact **only with `main`** unless specifically instructed otherwise.

Routine development should not be performed directly on `main`.

## `dev`

`dev` is the active development branch.

Use `dev` for:

- robot software development
- hardware configuration and calibration
- trajectory and waypoint development
- Command Center development
- perception and AprilTag integration
- network configuration development
- lab development
- documentation updates
- system integration and testing

Changes should be tested on `dev` before being released to `main`.

---

# 2. Start Here

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

For course labs, follow the instructions under:

```text
labs/
```

Do not try to operate the complete system from this root README.

---

# 3. System Overview

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
              ┌─────────┴─────────┐
              │                   │
              ▼                   ▼
         DUMP TRUCKS          EXCAVATORS
              │                   │
              └─────────┬─────────┘
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

# 4. Repository Structure

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

---

# 5. Development Workflow

All development is performed on `dev`.

```bash
git checkout dev
git pull origin dev

# Make and test changes

git status
git diff

git add .
git diff --cached --check
git commit -m "Describe the change"
git push origin dev
```

Do not make routine development changes directly on `main`.

---

# 6. Release to `main`

Release to `main` only after the changes have been tested on `dev`.

```bash
git checkout main
git pull origin main

git merge dev --no-commit --no-ff

# Keep the student-facing main README
git checkout HEAD -- README.md

git commit -m "Merge dev into main for AE 573 course release"
git push origin main

git checkout dev
```

Students interact only with `main`.

---

# 7. README Policy

The root `README.md` intentionally differs between branches:

- `dev` — development and branch-management information
- `main` — stable, student-facing information

All subdirectory READMEs should normally be identical between branches and branch-independent.

Do not add `dev`-specific instructions to:

```text
command_center/README.md
network/README.md
labs/*/README.md
```

---

# 8. Branch Rule

> **Develop and test on `dev`. Release validated changes to `main`. Students use `main`.**

**CIC-ConRobotics-2026**  
Penn State  
Computer Integrated Construction (CIC) Research Program  
AE 573 — Robotics and Automation in Construction