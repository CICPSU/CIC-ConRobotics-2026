# ROS 2 Network Configuration

This directory contains the shared network configuration for the CIC Construction Robotics platform.

The platform uses **Zenoh (`rmw_zenoh_cpp`) as the primary ROS 2 communication method** between the ROS PC and robot computers.

The network configuration is designed around a simple architecture:

```text
                    ROS PC
                       │
                 Zenoh Router
                       │
       ┌───────────────┼───────────────┐
       │               │               │
       ▼               ▼               ▼
  Dump Truck Pi   Dump Truck Pi   Excavator Pi
```

Only one Zenoh router is required for the construction robotics system.

All ROS PC applications and robot computers connect to this router.

---

# 1. Directory Structure

```text
network/
├── README.md
├── devices.sh
├── setup_zenoh.sh
└── setup_network.sh
```

The files have the following responsibilities:

| File | Purpose |
|---|---|
| `devices.sh` | Central registry of device names and assigned IP addresses |
| `setup_zenoh.sh` | Primary ROS 2 network configuration using Zenoh |
| `setup_network.sh` | DDS-based network configuration retained as an alternative/fallback |

---

# 2. Device Registry

Device IP addresses are maintained centrally in:

```text
network/devices.sh
```

The file defines addresses for:

- ROS PCs
- Dump truck Raspberry Pis
- Excavator Raspberry Pis

For example:

```bash
ROS_PC="..."

DUMPTRUCK_01="..."
DUMPTRUCK_02="..."

EXCAVATOR_01="..."
EXCAVATOR_02="..."
```

The purpose of this file is to provide a **single source of truth for network addresses**.

Do not hard-code device IP addresses into launch files, operational YAML files, or normal startup instructions.

When a device address changes, update:

```text
network/devices.sh
```

rather than modifying multiple files throughout the repository.

---

# 3. Primary Communication Method: Zenoh

The primary ROS 2 communication method for the platform is:

```text
rmw_zenoh_cpp
```

Zenoh is used to provide communication between the ROS PC and the robot computers.

The standard topology is:

```text
                         ROS PC
                            │
                       rmw_zenohd
                            │
                      Zenoh Router
                            │
          ┌─────────────────┼─────────────────┐
          │                 │                 │
          ▼                 ▼                 ▼
      Dump Truck        Dump Truck        Excavator
          Pi                Pi                Pi
```

The ROS PC hosts the Zenoh router.

Robot computers connect to the ROS PC as Zenoh clients.

ROS nodes running on the ROS PC also connect to the local Zenoh router as clients.

---

# 4. Standard Network Setup

Network configuration is performed using:

```text
network/setup_zenoh.sh
```

The script must be **sourced** rather than executed because it configures environment variables in the current terminal.

Correct:

```bash
source network/setup_zenoh.sh ...
```

Do not use:

```bash
./network/setup_zenoh.sh ...
```

---

# 5. ROS PC — Start the Zenoh Router

**Machine:** ROS PC  
**Terminal:** T1  
**Keep this terminal running.**

From the repository root:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router

ros2 run rmw_zenoh_cpp rmw_zenohd
```

Only **one Zenoh router** should normally be running for the construction robotics system.

The router terminal must remain running while the robots are being operated.

---

# 6. ROS PC — Configure Application Terminals

ROS nodes running on the ROS PC should connect to the local Zenoh router.

In each ROS PC application terminal:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc
```

After this setup, ROS applications can be launched normally.

For example:

```bash
ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

---

# 7. Robot Computers — Configure Zenoh Clients

Each physical robot computer connects to the same Zenoh router running on the ROS PC.

The user specifies the robot by name.

## Dump Truck 1

```bash
source network/setup_zenoh.sh client dumptruck1
```

## Dump Truck 2

```bash
source network/setup_zenoh.sh client dumptruck2
```

## Dump Truck 3

```bash
source network/setup_zenoh.sh client dumptruck3
```

## Dump Truck 4

```bash
source network/setup_zenoh.sh client dumptruck4
```

## Dump Truck 5

```bash
source network/setup_zenoh.sh client dumptruck5
```

## Excavator 1

```bash
source network/setup_zenoh.sh client excavator1
```

Additional configured excavators follow the same pattern.

The device name identifies the computer being configured.

It does **not** mean that the robot connects directly to that device's own IP address.

Remote robot clients connect to the Zenoh router running on the ROS PC.

---

# 8. Multiple Robots

Multiple robots share the same Zenoh router.

For example:

```text
                         ROS PC
                            │
                       Zenoh Router
                            │
          ┌─────────┬───────┼───────┬─────────┐
          │         │       │       │         │
          ▼         ▼       ▼       ▼         ▼
       Truck 1   Truck 2 Truck 3 Truck 4   Truck 5
          │         │       │       │         │
          Pi        Pi      Pi      Pi        Pi
```

Each robot computer independently runs:

```bash
source network/setup_zenoh.sh client <device>
```

For example, on four different Raspberry Pis:

```bash
# Dumptruck1 Raspberry Pi
source network/setup_zenoh.sh client dumptruck1
```

```bash
# Dumptruck3 Raspberry Pi
source network/setup_zenoh.sh client dumptruck3
```

```bash
# Dumptruck4 Raspberry Pi
source network/setup_zenoh.sh client dumptruck4
```

```bash
# Dumptruck5 Raspberry Pi
source network/setup_zenoh.sh client dumptruck5
```

All four clients connect to the same ROS PC router.

The robots do not need to be configured as direct peers of one another.

The general architecture is therefore:

```text
1 ROS PC
    │
    ├── 1 Zenoh Router
    │
    └── ROS PC Applications
             │
             ├── Perception
             ├── Localization
             └── Command Center

1 Zenoh client configuration per robot computer
```

---

# 9. Environment Variables

`setup_zenoh.sh` configures the ROS 2 communication environment automatically.

The primary variables include:

```text
ROS_DOMAIN_ID
RMW_IMPLEMENTATION
ZENOH_CONFIG_OVERRIDE
```

For the current course system:

```text
ROS_DOMAIN_ID=10
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Remote robot computers are configured to connect to the Zenoh router defined by the ROS PC address in:

```text
network/devices.sh
```

Users should normally **not manually export these variables**.

Use:

```bash
source network/setup_zenoh.sh ...
```

instead.

---

# 10. Switching From DDS to Zenoh

A terminal may contain environment variables from a previous DDS configuration.

`setup_zenoh.sh` removes DDS-specific settings that should not remain active when using Zenoh.

These include:

```text
ROS_STATIC_PEERS
ROS_AUTOMATIC_DISCOVERY_RANGE
ROS_AUTOMATIC_DISCOVERY
```

This reduces configuration conflicts when switching between communication backends.

When changing ROS middleware implementations, opening a fresh terminal is still recommended when practical.

If ROS 2 CLI discovery behaves unexpectedly after changing middleware, restart the ROS 2 daemon:

```bash
ros2 daemon stop
```

The daemon will restart automatically when required.

---

# 11. Verify Zenoh Configuration

After sourcing the Zenoh setup script, inspect the environment with:

```bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
echo $ZENOH_CONFIG_OVERRIDE
```

For a robot client, the expected middleware is:

```text
rmw_zenoh_cpp
```

and the ROS domain should be:

```text
10
```

The Zenoh configuration should indicate client mode and a connection endpoint to the ROS PC router.

---

# 12. Verify ROS 2 Communication

After the router and clients are running, verify ROS 2 discovery with:

```bash
ros2 node list
```

and:

```bash
ros2 topic list
```

For a dump truck system, expected topics may include:

```text
/detections
/truck1/cmd_vel
/truck1/wheel_states
/truck1/odom
/truck1/fused_odom
```

The exact topic list depends on which components are currently running.

---

# 13. Verify Topic Data

Seeing a topic name confirms discovery, but it does not necessarily confirm that useful data is being transferred.

Check the actual message stream.

For example:

```bash
ros2 topic echo /detections
```

For frequency-based checks:

```bash
ros2 topic hz /truck1/wheel_states
```

For image transport, compressed image topics may be useful when communicating across machines:

```bash
ros2 topic hz /image_raw/compressed
```

Run frequency tests for several seconds rather than relying on a single sample.

---

# 14. ROS 2 Message Types Must Exist on Both Computers

Zenoh transports ROS 2 messages between machines, but each computer still needs the ROS 2 interface definitions required to deserialize the messages it uses.

For example, a computer receiving:

```text
/detections
```

must have the corresponding AprilTag message definitions installed.

If ROS 2 reports an error similar to:

```text
The message type '...' is invalid
```

first verify that the required ROS 2 message package is installed on that computer.

This is different from a network discovery problem.

---

# 15. Standard Multi-Robot Startup Pattern

A typical multi-robot session uses the following terminal arrangement.

## ROS PC T1

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router

ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Keep running.**

## ROS PC T2

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

**Keep running.**

## Dumptruck1 T1

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

**Keep running.**

Other robot computers follow the same pattern using their own device and launch names.

---

# 16. DDS Fallback

The repository retains:

```text
network/setup_network.sh
```

for DDS-based network configuration.

This is maintained as an alternative/fallback communication method.

The DDS setup uses device addresses from the same:

```text
network/devices.sh
```

registry.

Example:

```bash
source network/setup_network.sh dumptruck_01 dumptruck_03
```

The Zenoh and DDS setup scripts should not normally be sourced together in the same operational terminal.

For normal Fall 2026 system operation, use:

```text
setup_zenoh.sh
```

unless a lab or troubleshooting procedure explicitly specifies otherwise.

---

# 17. Troubleshooting

If ROS 2 nodes cannot communicate across machines, check the system in the following order:

1. Confirm that the ROS PC and robot computer are connected to the required network.
2. Confirm that the Zenoh router is still running on the ROS PC.
3. Confirm that the correct `setup_zenoh.sh` client profile was sourced.
4. Confirm that `RMW_IMPLEMENTATION` is `rmw_zenoh_cpp`.
5. Confirm that all machines use the same `ROS_DOMAIN_ID`.
6. Confirm that the required ROS 2 message packages exist on each computer.
7. Check `ros2 node list` and `ros2 topic list`.
8. Check actual topic data using `ros2 topic echo` or `ros2 topic hz`.
9. If middleware was recently changed, open a fresh terminal or run:

```bash
ros2 daemon stop
```

Then repeat the network setup.

For network configuration changes, update:

```text
network/devices.sh
```

rather than hard-coding new IP addresses into launch commands.

---

# 18. Design Principle

The network layer is intentionally separated from the robot software.

Robot code should not need to know whether another robot is located at a particular IP address.

Instead:

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

This allows the robot software, perception system, and Command Center to remain largely independent from changes to the underlying physical network.

The intended operational rule is:

```text
Configure the network once.

Then run normal ROS 2 commands.
```