# CIC-ConRobotics-2026

## Description
This repository is used for CIC ConRobotics course AE 573, Fall 2026. ROS 2 Jazzy + Isaac Sim 5.1.0 course project.

The project integrates:
- ROS 2 (Jazzy)
- NVIDIA Isaac Sim 5.1.0


## Branch policy
- `main`: stable branch for students (exercises)
- `dev`: development/integration branch for the dev team

## Repository Structure

```
CIC-ConRobotics-2026/
├── docs/  # Documentation and launch instructions
│   ├── ROS2/
│   │   └── launch.md
│   └── isaac_sim/
│       └── launch.md
├── zx200_digging_stack/  # ROS2 and Isaac Sim packages for excavator control
├── isaac_sim/  # Isaac-Sim related assets (.usd)    
├── .gitattributes
├── .gitignore
└── README.md
```


## Clone
### 1. Go to your home directory
```bash
cd ~
```
### 2. Create a workspace directory
You can name this however you like. Below we use ws_conrobotics as an example.
```bash
mkdir -p ws_conrobotics
cd ws_conrobotics
```
### 3. Clone the repository
```bash
git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git
```
After cloning, your directory structure should look like this:
~/ws_conrobotics/
└── CIC-ConRobotics-2026/

### 4. Move into the project directory
```bash
cd CIC-ConRobotics-2026
```
You are now ready to continue with the next steps.

### Note
	•	Each user/group should work in their own home directory. Do not clone this repository into a shared system directory (e.g., /opt, /usr, etc.).


## Project status (important)

This directory (`zx200_digging_stack`) is **NOT a ROS 2 package** at the moment.

- There is **no `package.xml`**
- There is **no `setup.py`**
- You do **NOT** run `colcon build` for this stack
- Scripts are executed directly with `python3`

### Why it is not a ROS 2 package (yet)

The current goal is to:
- Rapidly iterate on excavator motion logic
- Share a working stack easily with new team members
- Run the same control logic in:
  - Isaac Sim (simulation)
  - Physical scale models (Raspberry Pi + motor drivers)

To achieve this, we keep the stack as:
- Plain Python scripts
- ROS 2 used only as a communication layer (`rclpy`, topics, actions)
- No build step (Colcon Build) required

This allows:
- `git clone` → `python3 run_*` → motion
- Faster debugging and experimentation
- Lower onboarding cost for new contributors

### When this *will* become a ROS 2 package

This stack is expected to be converted into a proper ROS 2 package in a later phase, when:
- Interfaces are stable
- Motion primitives are finalized

At that point, we will:
- Add `package.xml` and `setup.py`
- Support `colcon build`
- Split modules into reusable ROS 2 nodes

Until then, **treat this as a prototyping stack**, not a packaged ROS product.
