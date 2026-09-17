# ROS 2 Network Configuration

This directory contains the shared network configuration for the CIC Construction Robotics platform.

The standard ROS 2 communication architecture for the platform uses:

```text
rmw_zenoh_cpp
```

between the ROS application computers and robot computers.

The network is organized around **one active Zenoh router**.

The router can run on either:

- the primary ROS PC: `ros-pc`
- the backup ROS PC: `ros-backup-pc`

```text
                      ACTIVE ROUTER HOST
                   ros-pc OR ros-backup-pc
                              │
                         Zenoh Router
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
          ▼                   ▼                   ▼
      Dump Truck          Excavator 1         Excavator 3
          Pi                  Pi                  Pi
```

Only one Zenoh router is normally required for the construction robotics system.

All ROS applications and physical robot computers communicate through the currently selected router.

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

- ROS computers
- Dump truck Raspberry Pis
- Excavator Raspberry Pis

For the current router-capable computers:

| Device profile | Role | IP address |
|---|---|---|
| `ros-pc` | Primary ROS PC / primary router host | `10.170.32.181` |
| `ros-backup-pc` | Backup ROS PC / backup router host | `10.170.32.227` |

The corresponding entries in `devices.sh` are:

```text
ROS_PC
ROS_Laptop_Backup
```

The purpose of `devices.sh` is to provide a **single source of truth for network addresses**.

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

The system uses **one active Zenoh router**:

```text
rmw_zenohd
```

The router can run on either of the following computers:

| Router profile | Role | IP address |
|---|---|---|
| `ros-pc` | Primary router host | `10.170.32.181` |
| `ros-backup-pc` | Backup router host | `10.170.32.227` |

Under normal operation, `ros-pc` is the router.

If the primary ROS PC is unavailable or the system needs to be operated from the backup computer, `ros-backup-pc` can instead host the router.

Only **one router should normally be active at a time**.

```text
                    ACTIVE ROUTER HOST
                 ros-pc OR ros-backup-pc
                           │
                      rmw_zenohd
                           │
                      Zenoh Router
                           │
          ┌────────────────┼────────────────┐
          │                │                │
          ▼                ▼                ▼
      Dump Truck       Dump Truck       Excavator
          Pi               Pi               Pi
```

Robot computers connect to the selected router as Zenoh clients.

ROS 2 applications running on the router host also connect to that router as clients through localhost.

The robot computers do not need to be configured as direct ROS peers of one another.

The intended topology is:

```text
1 Active Router Host
        │
        ├── 1 Zenoh Router
        │
        ├── ROS 2 Applications
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
source network/setup_zenoh.sh router [router-device]
```

and:

```bash
source network/setup_zenoh.sh client <device> [router-device]
```

The available router profiles are:

```text
ros-pc
ros-backup-pc
```

If the router device is omitted, the script defaults to:

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

Similarly:

```bash
source network/setup_zenoh.sh client excavator3
```

is equivalent to:

```bash
source network/setup_zenoh.sh client excavator3 ros-pc
```

This preserves the standard `ros-pc` workflow while allowing the entire system to be redirected to the backup router when required.

---

# 5. Start the Zenoh Router

Only **one Zenoh router** should normally be running for the construction robotics system.

Do not start a separate router for each robot.

## Primary Router — ROS PC

**Machine:** ROS PC (`ros-pc`)

**Keep this terminal running.**

From the repository root:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

The shorter backward-compatible command:

```bash
source network/setup_zenoh.sh router
```

also selects `ros-pc`.

## Backup Router — Backup ROS PC

**Machine:** Backup ROS PC (`ros-backup-pc`)

**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-backup-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

When the backup router is used, remote clients must also be configured to connect to:

```text
ros-backup-pc
```

The router terminal must remain running while the physical system is being operated.

---

# 6. Configure ROS Application Terminals

ROS 2 application terminals must connect to the currently active Zenoh router.

## Applications on the Primary ROS PC

When `ros-pc` is the router:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc ros-pc
```

Because the application and router are running on the same machine, the client connects through:

```text
tcp/127.0.0.1:7447
```

The shorter backward-compatible command is:

```bash
source network/setup_zenoh.sh client ros-pc
```

## Applications on the Backup ROS PC

When `ros-backup-pc` is the router:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-backup-pc ros-backup-pc
```

The backup computer also connects to its local router through:

```text
tcp/127.0.0.1:7447
```

After the network environment is configured, ROS applications such as the Command Center can be launched normally.

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

Each physical robot computer connects to the **currently active Zenoh router**.

The command format is:

```bash
source network/setup_zenoh.sh client <robot-device> [router-device]
```

The first device name identifies the computer being configured.

The optional second device name identifies the router that the computer should connect to.

Conceptually:

```text
client excavator3 ros-backup-pc
       │              │
       │              └── Which router should this client use?
       │
       └── Which computer is being configured?
```

## Normal Operation — Primary ROS PC Router

When `ros-pc` is the router:

```bash
source network/setup_zenoh.sh client dumptruck1
source network/setup_zenoh.sh client dumptruck2
source network/setup_zenoh.sh client dumptruck3
source network/setup_zenoh.sh client dumptruck4
source network/setup_zenoh.sh client dumptruck5

source network/setup_zenoh.sh client excavator1
source network/setup_zenoh.sh client excavator3
```

Because `ros-pc` is the default router, explicitly specifying it is optional.

For example, these are equivalent:

```bash
source network/setup_zenoh.sh client excavator3
```

```bash
source network/setup_zenoh.sh client excavator3 ros-pc
```

Remote clients in this configuration connect to:

```text
tcp/10.170.32.181:7447
```

## Backup Operation — Backup ROS PC Router

When `ros-backup-pc` is the router, the backup router must be specified for each remote client:

```bash
source network/setup_zenoh.sh client dumptruck1 ros-backup-pc
source network/setup_zenoh.sh client dumptruck2 ros-backup-pc
source network/setup_zenoh.sh client dumptruck3 ros-backup-pc
source network/setup_zenoh.sh client dumptruck4 ros-backup-pc
source network/setup_zenoh.sh client dumptruck5 ros-backup-pc

source network/setup_zenoh.sh client excavator1 ros-backup-pc
source network/setup_zenoh.sh client excavator3 ros-backup-pc
```

Remote clients in this configuration connect to:

```text
tcp/10.170.32.227:7447
```

Additional configured robots follow the same pattern.

---

# 8. Zenoh Device Profile vs. ROS 2 Namespace

The Zenoh device profile and ROS 2 namespace are related to robot identity, but they serve different purposes.

For example, on Excavator 3:

```bash
source network/setup_zenoh.sh client excavator3
```

configures the **network connection** for the Excavator 3 computer using the default `ros-pc` router.

Alternatively:

```bash
source network/setup_zenoh.sh client excavator3 ros-backup-pc
```

configures the same Excavator 3 computer to use the backup router.

Neither command creates the ROS namespace.

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
Zenoh Device Profile + Router Selection
                  │
                  ▼
       How does this computer connect?
                  │
                  ▼
                Zenoh


            ROS robot_name
                  │
                  ▼
      What ROS namespace does it use?
                  │
                  ▼
          /excavator3/...
```

The network profile does not automatically create the ROS namespace.

For normal operation, the corresponding robot identity should be used consistently.

---

# 9. Multiple Robots

Multiple robots share the same **active** Zenoh router.

For example:

```text
                       ACTIVE ROUTER HOST
                    ros-pc OR ros-backup-pc
                              │
                         Zenoh Router
                              │
       ┌──────────┬───────────┼──────────┬──────────┐
       │          │           │          │          │
       ▼          ▼           ▼          ▼          ▼
    Truck 1    Truck 3     Truck 4    Truck 5   Excavator 3
       │          │           │          │          │
       Pi         Pi          Pi         Pi         Pi
```

When the primary router is active, each robot can use the default configuration:

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

When the backup router is active, each client must identify it:

```bash
# Dumptruck1 Raspberry Pi
source network/setup_zenoh.sh client dumptruck1 ros-backup-pc
```

```bash
# Dumptruck3 Raspberry Pi
source network/setup_zenoh.sh client dumptruck3 ros-backup-pc
```

```bash
# Excavator3 Raspberry Pi
source network/setup_zenoh.sh client excavator3 ros-backup-pc
```

All active clients must connect to the same active router.

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

The script also exports helper variables describing the selected configuration:

```text
CIC_ZENOH_ROLE
CIC_ZENOH_DEVICE
CIC_ZENOH_ROUTER
CIC_ZENOH_ROUTER_IP
```

For the current course system:

```text
ROS_DOMAIN_ID=10
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

For a remote client using the primary router:

```text
ZENOH_CONFIG_OVERRIDE=mode="client";connect/endpoints=["tcp/10.170.32.181:7447"]
```

For a remote client using the backup router:

```text
ZENOH_CONFIG_OVERRIDE=mode="client";connect/endpoints=["tcp/10.170.32.227:7447"]
```

For a ROS application running on the active router host:

```text
ZENOH_CONFIG_OVERRIDE=mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]
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

For a remote client using the primary router, the endpoint should contain:

```text
tcp/10.170.32.181:7447
```

For a remote client using the backup router, the endpoint should contain:

```text
tcp/10.170.32.227:7447
```

For a client running on the selected router host, the endpoint should contain:

```text
tcp/127.0.0.1:7447
```

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

A typical physical multi-robot session uses one active router host, ROS application terminals, and multiple robot clients.

## Option A — Primary ROS PC as Router

### ROS PC T1 — Router

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Keep running.**

### ROS PC T2 — Applications / Command Center

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc ros-pc

ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1,truck3,truck4,truck5 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

**Keep running.**

### Dumptruck1 Raspberry Pi

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

**Keep running.**

### Excavator3 Raspberry Pi

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

## Option B — Backup ROS PC as Router

### Backup ROS PC T1 — Router

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-backup-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Keep running.**

### Backup ROS PC T2 — Applications / Command Center

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-backup-pc ros-backup-pc

ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1,truck3,truck4,truck5 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

**Keep running.**

### Dumptruck1 Raspberry Pi

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1 ros-backup-pc

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

**Keep running.**

### Excavator3 Raspberry Pi

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client excavator3 ros-backup-pc

sudo pigpiod

ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi \
  robot_name:=excavator3 \
  config:=$(ros2 pkg prefix excavator_control)/share/excavator_control/config/excavator3.yaml
```

**Keep running.**

Other robot computers follow the same pattern using their corresponding device profile and the currently active router.

---

# 16. One Router, Multiple Robot Types

Dump trucks and excavators use the same communication backbone.

```text
                     ACTIVE ROUTER HOST
                  ros-pc OR ros-backup-pc
                            │
              ┌─────────────┴─────────────┐
              │                           │
         Zenoh Router               ROS Applications
              │                           │
       ┌──────┼─────────┐                 │
       │      │         │                 │
       ▼      ▼         ▼                 │
    Truck 1 Truck 3 Excavator 3 ◄─────────┘
       Pi      Pi        Pi
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

1. Confirm that the active router host and robot computers are connected to the required network.
2. Confirm that exactly one intended Zenoh router is running.
3. Confirm whether the active router is `ros-pc` or `ros-backup-pc`.
4. Confirm that every client was configured for that same router.
5. Confirm that `RMW_IMPLEMENTATION` is `rmw_zenoh_cpp`.
6. Confirm that all machines use the same `ROS_DOMAIN_ID`.
7. Inspect `CIC_ZENOH_ROUTER` and `CIC_ZENOH_ROUTER_IP`.
8. Inspect `ZENOH_CONFIG_OVERRIDE` and confirm that it points to the expected router.
9. Confirm that the required ROS 2 message packages exist and are sourced.
10. Check `ros2 node list`, `ros2 topic list`, and `ros2 action list`.
11. Check actual topic data using `ros2 topic echo` or `ros2 topic hz`.
12. For Action-based robots, inspect the expected Action with `ros2 action info`.

For example:

```bash
echo $CIC_ZENOH_ROUTER
echo $CIC_ZENOH_ROUTER_IP
echo $ZENOH_CONFIG_OVERRIDE
```

A remote client using the primary router should point to:

```text
tcp/10.170.32.181:7447
```

A remote client using the backup router should point to:

```text
tcp/10.170.32.227:7447
```

For Excavator 3:

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
Select one active router.

Configure all clients for that router.

Then run normal ROS 2 commands.
```

At the system level:

```text
ONE ACTIVE ROUTER HOST
          +
    ONE Zenoh Router
          +
   ONE Command Center
          +
    N Robot Clients
```

The primary configuration uses:

```text
ros-pc
```

The backup configuration uses:

```text
ros-backup-pc
```