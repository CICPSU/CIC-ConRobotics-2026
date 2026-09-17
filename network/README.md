# ROS 2 Network Configuration

This directory contains the shared network configuration for the CIC Construction Robotics platform.

The standard ROS 2 communication architecture for the platform uses:

```text
rmw_zenoh_cpp
```

between the ROS PC and robot computers.

The network is organized around one shared Zenoh router:

```text
                              ROS PC
                                 │
                           Zenoh Router
                                 │
          ┌──────────────────────┼──────────────────────┐
          │                      │                      │
          ▼                      ▼                      ▼
      Dump Truck            Excavator 1            Excavator 3
          Pi                    Pi                     Pi
```

Only one Zenoh router is required for the construction robotics system.

All ROS PC applications and physical robot computers communicate through this shared architecture.

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
| `setup_zenoh.sh` | Standard ROS 2 network configuration using Zenoh |
| `setup_network.sh` | Legacy DDS configuration retained for specialized troubleshooting |

Normal physical operation uses:

```text
setup_zenoh.sh
```

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

The purpose of this file is to provide a **single source of truth for network addresses**.

Do not hard-code device IP addresses into:

- launch files
- operational YAML files
- robot-control code
- normal startup instructions

When a device address changes, update:

```text
network/devices.sh
```

rather than modifying multiple files throughout the repository.

---

# 3. Standard Communication Architecture: Zenoh

The standard ROS 2 middleware for the physical platform is:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

The ROS PC hosts one Zenoh router:

```text
rmw_zenohd
```

Robot computers connect to this router as Zenoh clients.

ROS 2 applications running on the ROS PC also connect to the local router as clients.

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

The robot computers do not need to be configured as direct ROS peers of one another.

The intended topology is:

```text
1 ROS PC
    │
    ├── 1 Zenoh Router
    │
    ├── ROS PC Applications
    │
    └── N Robot Clients
```

---

# 4. Network Setup Script

Normal network configuration is performed using:

```text
network/setup_zenoh.sh
```

The script must be **sourced**, because it configures environment variables in the current terminal.

Correct:

```bash
source network/setup_zenoh.sh ...
```

Do not use:

```bash
./network/setup_zenoh.sh ...
```

The general forms are:

```bash
source network/setup_zenoh.sh router
```

for the ROS PC router terminal, and:

```bash
source network/setup_zenoh.sh client <device>
```

for ROS PC application terminals and physical robot computers.

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

Do not start a separate router for each robot.

The router terminal must remain running while the physical system is being operated.

---

# 6. ROS PC — Configure Application Terminals

ROS 2 nodes running on the ROS PC connect to the local Zenoh router.

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

The device name identifies which computer is being configured.

Examples:

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

## Excavator 3

```bash
source network/setup_zenoh.sh client excavator3
```

Additional configured robots follow the same pattern.

The device name selects the appropriate network configuration.

It does **not** mean that the robot connects directly to its own IP address.

Remote robot clients connect to the Zenoh router running on the ROS PC.

---

# 8. Zenoh Device Profile vs. ROS 2 Namespace

The Zenoh device profile and ROS 2 namespace are related to robot identity, but they serve different purposes.

For example, on Excavator 3:

```bash
source network/setup_zenoh.sh client excavator3
```

configures the **network connection** for the Excavator 3 computer.

When the excavator server is launched with:

```text
robot_name:=excavator3
```

the launch argument configures the **ROS 2 namespace**.

The resulting ROS interfaces include:

```text
/excavator3/excavator_trajectory_server
/excavator3/upper_arm_controller/follow_joint_trajectory
/excavator3/joint_states
```

Conceptually:

```text
Zenoh Device Profile
        │
        ▼
How does this computer connect?
        │
        ▼
       Zenoh


ROS robot_name
        │
        ▼
What ROS namespace does this robot use?
        │
        ▼
/excavator3/...
```

The network profile does not automatically create the ROS namespace.

For normal operation, the corresponding robot identity should be used consistently for both.

---

# 9. Multiple Robots

Multiple robots share the same Zenoh router.

For example:

```text
                              ROS PC
                                 │
                           Zenoh Router
                                 │
       ┌──────────┬──────────┬───┴────┬──────────┐
       │          │          │        │          │
       ▼          ▼          ▼        ▼          ▼
    Truck 1    Truck 3    Truck 4  Truck 5   Excavator 3
       │          │          │        │          │
       Pi         Pi         Pi       Pi         Pi
```

Each robot computer independently runs:

```bash
source network/setup_zenoh.sh client <device>
```

For example:

```bash
# Dumptruck1 Raspberry Pi
source network/setup_zenoh.sh client dumptruck1
```

```bash
# Dumptruck3 Raspberry Pi
source network/setup_zenoh.sh client dumptruck3
```

```bash
# Excavator3 Raspberry Pi
source network/setup_zenoh.sh client excavator3
```

All clients connect to the same ROS PC router.

The robots do not need to be configured as direct peers of one another.

---

# 10. Environment Variables

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

# 11. Verify Zenoh Configuration

After sourcing the Zenoh setup script, inspect the environment with:

```bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
echo $ZENOH_CONFIG_OVERRIDE
```

Expected middleware:

```text
rmw_zenoh_cpp
```

Expected ROS domain:

```text
10
```

For a remote robot client, the Zenoh configuration should indicate client mode and an endpoint for the ROS PC router.

For a ROS PC application terminal, the client connects to the router running locally on the ROS PC.

---

# 12. Verify ROS 2 Communication

After the router and clients are running, check ROS 2 discovery:

```bash
ros2 node list
```

Check topics:

```bash
ros2 topic list
```

Check Actions:

```bash
ros2 action list
```

The exact interfaces depend on which components are currently running.

For example, a Truck 1 system may expose:

```text
/truck1/cmd_vel
/truck1/wheel_states
/truck1/odom
/truck1/fused_odom
/truck1/execute_robot_task
```

An active Excavator 3 server should expose interfaces including:

```text
/excavator3/excavator_trajectory_server
/excavator3/joint_states
/excavator3/upper_arm_controller/follow_joint_trajectory
```

---

# 13. Verify Topic and Action Data

Seeing a topic or Action name confirms ROS 2 discovery, but it does not by itself confirm that useful data is being transferred.

Check actual messages when appropriate.

For example:

```bash
ros2 topic echo /detections
```

Dump Truck 1 wheel-state frequency:

```bash
ros2 topic hz /truck1/wheel_states
```

Excavator 3 joint feedback:

```bash
ros2 topic echo /excavator3/joint_states --once
```

Excavator 3 Action:

```bash
ros2 action info \
  /excavator3/upper_arm_controller/follow_joint_trajectory
```

For an active Excavator 3 trajectory server, the Action should report one Action server.

For image transport, compressed image topics may be useful when communicating across machines:

```bash
ros2 topic hz /image_raw/compressed
```

Run frequency tests for several seconds rather than relying on a single sample.

---

# 14. ROS 2 Message Types Must Exist on Both Computers

Zenoh transports ROS 2 messages between machines, but each computer still needs the ROS 2 interface definitions required to interpret the messages it uses.

For example, a computer receiving:

```text
/detections
```

must have the corresponding AprilTag message definitions installed.

Similarly, computers interacting with the Command Center may require:

```text
construction_site_interfaces
```

and computers interacting with excavator trajectories require the standard ROS 2 control message definitions.

If ROS 2 reports an error similar to:

```text
The message type '...' is invalid
```

first verify that the required ROS 2 message package is installed and sourced on that computer.

This is different from a network discovery problem.

---

# 15. Standard Multi-Robot Startup Pattern

A typical physical multi-robot session uses the following terminal arrangement.

## ROS PC T1 — Router

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router

ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Keep running.**

---

## ROS PC T2 — Applications / Command Center

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

---

## Dumptruck1 Raspberry Pi

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

**Keep running.**

---

## Excavator3 Raspberry Pi

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

**Keep running.**

Other robot computers follow the same pattern using their corresponding device profile and launch configuration.

---

# 16. One Router, Multiple Robot Types

Dump trucks and excavators use the same communication backbone.

For example:

```text
                              ROS PC
                                 │
                ┌────────────────┴────────────────┐
                │                                 │
          Zenoh Router                      ROS Applications
                │                                 │
       ┌────────┼─────────┐                       │
       │        │         │                       │
       ▼        ▼         ▼                       │
    Truck 1  Truck 3  Excavator 3 ◄───────────────┘
       Pi       Pi         Pi
```

No robot-specific router is required.

Adding another physical robot normally requires:

1. A device entry in `network/devices.sh`
2. A valid Zenoh client profile
3. The robot's ROS 2 packages and dependencies
4. Robot-specific launch/configuration

It does not require another Zenoh router.

---

# 17. Troubleshooting

If ROS 2 nodes cannot communicate across machines, check the system in this order:

1. Confirm that the ROS PC and robot computer are connected to the required network.
2. Confirm that the Zenoh router is still running on the ROS PC.
3. Confirm that the correct `setup_zenoh.sh` client profile was sourced.
4. Confirm that `RMW_IMPLEMENTATION` is `rmw_zenoh_cpp`.
5. Confirm that all machines use the same `ROS_DOMAIN_ID`.
6. Confirm that the required ROS 2 message packages exist and are sourced.
7. Check `ros2 node list`, `ros2 topic list`, and `ros2 action list`.
8. Check actual topic data using `ros2 topic echo` or `ros2 topic hz`.
9. For Action-based robots, inspect the expected Action with `ros2 action info`.

For Excavator 3, for example:

```bash
ros2 action info \
  /excavator3/upper_arm_controller/follow_joint_trajectory
```

If middleware configuration was previously changed in the same terminal, opening a fresh terminal is recommended.

The ROS 2 daemon can also be restarted with:

```bash
ros2 daemon stop
```

Then source the ROS environment and `setup_zenoh.sh` again.

For network address changes, update:

```text
network/devices.sh
```

rather than hard-coding IP addresses into launch commands.

---

# 18. Legacy DDS Configuration

The repository retains:

```text
network/setup_network.sh
```

for legacy DDS-based configuration and specialized troubleshooting.

It uses the same device registry:

```text
network/devices.sh
```

This is **not the standard communication path for normal physical robot operation**.

Normal Fall 2026 operation uses:

```text
rmw_zenoh_cpp
```

through:

```bash
source network/setup_zenoh.sh ...
```

Do not switch a physical robot to DDS as a normal troubleshooting step.

The Zenoh and legacy DDS setup scripts should not be sourced together in the same operational terminal.

---

# 19. Design Principle

The network layer is intentionally separated from robot software.

Robot code should not need to know the physical IP address of another robot.

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

This allows robot software, perception, and the Command Center to remain largely independent from changes to the physical network.

The intended operational rule is:

```text
Configure the network once.

Then run normal ROS 2 commands.
```

At the system level:

```text
ONE ROS PC
    +
ONE Zenoh Router
    +
ONE Command Center
    +
N Robot Clients
```