# ROS 2 Network Configuration

This directory contains the shared network configuration for the CIC Construction Robotics platform.

The standard communication architecture uses:

```text
rmw_zenoh_cpp
```

with **one active Zenoh router** for the complete construction robotics system.

```text
                    ACTIVE ROUTER HOST
                 ros-pc OR ros-backup-pc
                           │
                       Zenoh Router
                           │
          ┌────────────────┼────────────────┐
          │                │                │
          ▼                ▼                ▼
      Dump Truck       Excavator        ROS PC Apps
          Pi               Pi
```

All ROS application computers and physical robot computers must connect to the same active router.

> This README covers network configuration, SSH access, Zenoh setup, and communication troubleshooting.  
> For robot startup, trajectories, waypoint tasks, and scenario execution, see `command_center/README.md`.

---

# 1. Network Files

```text
network/
├── README.md
├── devices.sh
├── setup_pi.sh
├── setup_zenoh.sh
└── setup_network.sh
```

| File | Purpose |
|---|---|
| `devices.sh` | Central registry of device names and assigned IP addresses |
| `setup_pi.sh` | Base setup for a new Raspberry Pi running Ubuntu 24.04 |
| `setup_zenoh.sh` | Standard ROS 2 network configuration using Zenoh |
| `setup_network.sh` | Legacy DDS configuration retained only for specialized troubleshooting |

Normal physical operation uses:

```text
network/devices.sh
network/setup_zenoh.sh
```

For a **new Raspberry Pi**, first use:

```text
network/setup_pi.sh
```

`setup_network.sh` is **not** part of the normal Fall 2026 workflow.

---

# 2. New Raspberry Pi Setup

Use `network/setup_pi.sh` when preparing a new Raspberry Pi running Ubuntu 24.04.

The script performs the base software setup required by the current platform, including:

- system updates and development tools
- ROS 2 Jazzy
- `rmw_zenoh_cpp`
- ROS development tools
- SSH
- pigpio
- I2C support
- Raspberry Pi / ADS1115 Python dependencies used for excavator calibration

The script intentionally does **not** configure a Zenoh router, select a robot identity, or install robot-specific repository configuration.

## 2.1 Run the Setup Script

First connect the new Raspberry Pi to the Internet.

Copy:

```text
network/setup_pi.sh
```

to the Raspberry Pi.

From the directory containing the script:

```bash
chmod +x setup_pi.sh
./setup_pi.sh
```

The script does not modify `~/.bashrc`. ROS and Zenoh environments are configured explicitly in operational terminals.

When setup completes, reboot:

```bash
sudo reboot
```

After reboot, continue with device registration below.

---

# 3. Device Registration

Before a new Raspberry Pi can be used as a physical robot on the course network, complete the network registration process.

## 3.1 Register a New Device

The general process is:

1. Obtain the Raspberry Pi's **MAC address**.
2. Register the device on the **IoT network**.
3. Obtain the **fixed IP address assigned by the IoT network**.
4. On the computer that will operate the robot, allow firewall access to the assigned fixed IP address as required. This requires higher authentification from IT personnel.
5. Add the new device profile and assigned IP address to:

```text
network/devices.sh
```

6. Verify network access and SSH before configuring Zenoh.

Device addresses are maintained centrally in:

```text
network/devices.sh
```

This file is the **single source of truth for network addresses**.

It contains profiles for:

- ROS PCs
- Dump truck Raspberry Pis
- Excavator Raspberry Pis

Do **not** duplicate device IP addresses in:

- this README
- launch files
- operational YAML files
- robot-control code
- normal startup instructions

If an address changes, update:

```text
network/devices.sh
```

rather than changing multiple files throughout the repository.

---

# 4. SSH Access to Robot Computers

Physical robot Raspberry Pis are normally operated remotely through SSH.

The Raspberry Pi and the operating computer must be connected to the required network environment. If the current connection requires the Penn State VPN, connect to the VPN before attempting SSH.

Use:

```text
network/devices.sh
```

to identify the target robot and its current IP address.

The general SSH command is:

```bash
ssh <username>@<robot-ip>
```

## 4.1 SSH Key Setup

Passwordless SSH is recommended for repeated robot operation.

On Linux, macOS, or Git Bash:

```bash
ssh-keygen
ssh-copy-id <username>@<robot-ip>
```

If an SSH key already exists, do not create another key unless needed.

On Windows, Git Bash can be used when `ssh-copy-id` is not available in the standard PowerShell environment.

After installing the key, verify:

```bash
ssh <username>@<robot-ip>
```

before continuing with Zenoh or robot startup.

## 4.2 If SSH Does Not Connect

Check, in this order:

1. The Raspberry Pi is powered on.
2. The Raspberry Pi and operating computer are connected to the required network.
3. The Penn State VPN is connected when required.
4. The target address matches `network/devices.sh`.
5. The SSH service is running on the Raspberry Pi.
6. The robot is reachable over the network.

If local access to the Raspberry Pi is available:

```bash
sudo systemctl status ssh
```

and, if necessary:

```bash
sudo systemctl enable --now ssh
```

---

# 6. Standard Zenoh Architecture


The physical platform uses:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

and one active:

```text
rmw_zenohd
```

router.

The router may run on:

```text
ros-pc
```

or:

```text
ros-backup-pc
```

Under normal operation, use `ros-pc`.

Only **one intended Zenoh router should normally be active at a time**.

```text
                 ONE ACTIVE ROUTER HOST
                          │
                      rmw_zenohd
                          │
            ┌─────────────┼─────────────┐
            │             │             │
            ▼             ▼             ▼
         Truck Pi     Excavator Pi   ROS PC Apps
```

Robot computers do not need to be configured as direct ROS peers of one another. They connect to the selected Zenoh router.

---

# 6. Configure Zenoh

Normal network configuration is performed with:

```text
network/setup_zenoh.sh
```

The script must be **sourced** because it configures the current terminal environment.

Correct:

```bash
source network/setup_zenoh.sh ...
```

Do not use:

```bash
./network/setup_zenoh.sh ...
```

The general commands are:

```bash
source network/setup_zenoh.sh router [router-device]
```

and:

```bash
source network/setup_zenoh.sh client <device> [router-device]
```

Supported router profiles are:

```text
ros-pc
ros-backup-pc
```

If `[router-device]` is omitted, the default is:

```text
ros-pc
```

Therefore:

```bash
source network/setup_zenoh.sh router
```

is equivalent to:

```bash
source network/setup_zenoh.sh router ros-pc
```

and:

```bash
source network/setup_zenoh.sh client excavator3
```

is equivalent to:

```bash
source network/setup_zenoh.sh client excavator3 ros-pc
```

---

# 7. Start the Zenoh Router

Only one Zenoh router should normally be running.

## 6.1 Primary Router — ROS PC

**Machine:** `ros-pc`

Keep this terminal running.

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

## 6.2 Backup Router — Backup ROS PC

**Machine:** `ros-backup-pc`

Keep this terminal running.

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-backup-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

When `ros-backup-pc` is the active router, **every remote client must also be configured to use `ros-backup-pc`**.

---

# 8. Configure ROS PC Application Terminals

A ROS application terminal must connect to the currently active router.

## Primary ROS PC

When `ros-pc` is the active router:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc
```

## Backup ROS PC

When `ros-backup-pc` is the active router:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-backup-pc
```

A ROS application running on the active router host connects to its local router through:

```text
tcp/127.0.0.1:7447
```

Network setup ends here. For normal system launch and scenario execution, continue with:

```text
command_center/README.md
```

---

# 9. Configure Physical Robot Clients

Each robot Raspberry Pi must connect to the same active Zenoh router.

General form:

```bash
source network/setup_zenoh.sh client <robot-device> [router-device]
```

The first device identifies **which computer is being configured**.

The optional second device identifies **which router it should use**.

For example:

```bash
source network/setup_zenoh.sh client excavator3 ros-backup-pc
```

means:

```text
excavator3
    └── robot computer being configured

ros-backup-pc
    └── active router
```

## 8.1 Primary Router

When `ros-pc` is active:

```bash
source network/setup_zenoh.sh client dumptruck1
```

```bash
source network/setup_zenoh.sh client excavator3
```

Explicitly specifying the default router is also valid:

```bash
source network/setup_zenoh.sh client excavator3 ros-pc
```

## 8.2 Backup Router

When `ros-backup-pc` is active:

```bash
source network/setup_zenoh.sh client dumptruck1 ros-backup-pc
```

```bash
source network/setup_zenoh.sh client excavator3 ros-backup-pc
```

All active clients must use the same router.

Additional robots follow the same pattern using their device profile from `network/devices.sh`.

---

# 10. Device Profile vs. ROS Robot Name

The Zenoh device profile and ROS robot name are related, but they are not the same configuration.

The **Zenoh device profile** determines:

```text
How does this computer connect to the network?
```

The **ROS robot name / namespace** determines:

```text
Which ROS interfaces belong to this robot?
```

For an excavator, these names normally match.

Example:

```text
Zenoh device profile: excavator3
ROS robot name:       excavator3
ROS namespace:        /excavator3/...
```

For dump trucks, they intentionally differ:

```text
Zenoh device profile: dumptruck1
ROS robot name:       truck1
ROS namespace:        /truck1/...
```

For example:

```bash
source network/setup_zenoh.sh client dumptruck1
```

configures the network connection of the Raspberry Pi.

The robot's ROS launch separately creates interfaces under:

```text
/truck1/...
```

The network profile does not automatically create a ROS namespace.

---

# 11. Environment Variables

`setup_zenoh.sh` configures the ROS 2 communication environment automatically.

Important variables include:

```text
ROS_DOMAIN_ID
RMW_IMPLEMENTATION
ZENOH_CONFIG_OVERRIDE

CIC_ZENOH_ROLE
CIC_ZENOH_DEVICE
CIC_ZENOH_ROUTER
CIC_ZENOH_ROUTER_IP
```

For the current platform:

```text
ROS_DOMAIN_ID=10
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Users should normally **not manually export Zenoh configuration variables**.

Use:

```bash
source network/setup_zenoh.sh ...
```

instead.

---

# 12. Verify Zenoh Configuration

After sourcing `setup_zenoh.sh`, inspect the environment:

```bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
echo $ZENOH_CONFIG_OVERRIDE
echo $CIC_ZENOH_ROLE
echo $CIC_ZENOH_DEVICE
echo $CIC_ZENOH_ROUTER
echo $CIC_ZENOH_ROUTER_IP
```

Expected middleware:

```text
rmw_zenoh_cpp
```

Expected ROS domain:

```text
10
```

A remote client using the primary router should point to:

```text
the primary-router endpoint configured from `network/devices.sh`
```

A remote client using the backup router should point to:

```text
the backup-router endpoint configured from `network/devices.sh`
```

A ROS application on the active router host should point to:

```text
tcp/127.0.0.1:7447
```

---

# 13. Verify ROS 2 Communication

Once the router and clients are running, verify ROS discovery.

```bash
ros2 node list
```

```bash
ros2 topic list
```

```bash
ros2 action list
```

Seeing an interface confirms discovery, but not necessarily that useful data is being transferred.

Check actual data when needed.

Examples:

```bash
ros2 topic hz /truck1/wheel_states
```

```bash
ros2 topic echo /excavator3/joint_states --once
```

```bash
ros2 action info \
  /excavator3/upper_arm_controller/follow_joint_trajectory
```

For perception traffic:

```bash
ros2 topic echo /detections
```

or:

```bash
ros2 topic hz /image_raw/compressed
```

Run frequency tests for several seconds rather than relying on a single sample.

---

# 14. ROS 2 Message Types Must Exist on Both Computers

Zenoh transports ROS 2 messages between machines, but each participating computer still needs the interface definitions required to interpret those messages.

For example, a computer receiving AprilTag detections must have the corresponding message definitions installed.

Computers interacting with Command Center interfaces may require:

```text
construction_site_interfaces
```

If ROS 2 reports:

```text
The message type '...' is invalid
```

verify that the required ROS 2 package is installed, built, and sourced on that computer.

This is different from a Zenoh discovery problem.

---

# 15. Troubleshooting

If ROS 2 nodes cannot communicate across machines, check the system in this order:

1. Confirm that the ROS PC and robot computers are connected to the required network.
2. Confirm whether `ros-pc` or `ros-backup-pc` is the intended active router.
3. Confirm that exactly one intended Zenoh router is running.
4. Confirm that every client was configured for that same router.
5. Confirm `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
6. Confirm all machines use `ROS_DOMAIN_ID=10`.
7. Inspect `CIC_ZENOH_ROUTER` and `CIC_ZENOH_ROUTER_IP`.
8. Inspect `ZENOH_CONFIG_OVERRIDE`.
9. Check `ros2 node list`, `ros2 topic list`, and `ros2 action list`.
10. Check actual data with `ros2 topic echo` or `ros2 topic hz`.
11. Verify that required ROS 2 message packages are installed and sourced.

Useful checks:

```bash
echo $CIC_ZENOH_ROUTER
echo $CIC_ZENOH_ROUTER_IP
echo $ZENOH_CONFIG_OVERRIDE
```

If the terminal has previously been configured with another middleware or router, open a fresh terminal and source the ROS and Zenoh environment again.

The ROS 2 daemon can also be restarted:

```bash
ros2 daemon stop
```

Then source:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client <device> [router-device]
```

For address changes, modify:

```text
network/devices.sh
```

rather than hard-coding the new address elsewhere.

---

# 16. Legacy DDS Configuration

The repository retains:

```text
network/setup_network.sh
```

for legacy DDS configuration and specialized troubleshooting.

It is **not the standard communication path for physical robot operation**.

Normal operation uses:

```text
rmw_zenoh_cpp
```

through:

```bash
source network/setup_zenoh.sh ...
```

Do **not** switch a physical robot to Fast DDS or CycloneDDS as a normal troubleshooting step.

Do not source the Zenoh and legacy DDS setup scripts in the same operational terminal.

---

# 17. Network Design Rules

The network layer is intentionally separated from robot software.

```text
Robot Software
      │
      ▼
    ROS 2
      │
      ▼
Network Configuration
      │
      ▼
    Zenoh
      │
      ▼
Physical Network
```

The operational rules are:

```text
ONE active router host
        +
ONE Zenoh router
        +
N Zenoh clients
```

Select the router first. Configure every participating computer for that router. Then run normal ROS 2 commands.

Robot operation itself belongs in:

```text
command_center/README.md
```