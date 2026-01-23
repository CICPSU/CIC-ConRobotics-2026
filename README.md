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

