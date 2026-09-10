# Establish ROS 2 Communication with Raspberry Pi

These instructions explain how to establish ROS 2 communication between the Linux ROS computer and Raspberry Pi-based construction robots on the PSU network.

The CIC ConRobotics system currently uses **Zenoh** as the primary communication method for ROS 2 communication between computers.

The basic network architecture is:

```text
                       ROS PC
                          │
                     Zenoh Router
                          │
          ┌───────────────┼───────────────┐
          │               │               │
          ▼               ▼               ▼
      Dumptruck1      Dumptruck3      Excavator1
      Raspberry Pi    Raspberry Pi    Raspberry Pi
```

The important rule is:

```text
1 Zenoh Router
      +
1 ROS PC
      +
N Robot Clients
```

Each robot connects to the Zenoh router running on the ROS PC.

Robots do **not** need to connect directly to each other.

---

# 1. Before You Start

Make sure:

- the ROS PC and Raspberry Pi are connected to the appropriate PSU network
- ROS 2 Jazzy is installed
- the project repository has been cloned
- the repository has been built
- Zenoh support is installed
- the Raspberry Pi IP address is registered in the project device list

The project repository is:

```text
CIC-ConRobotics-2026
```

The network configuration files are stored in:

```text
network/
├── devices.sh
├── setup_zenoh.sh
├── setup_network.sh
└── README.md
```

The purpose of each file is:

```text
devices.sh
    Device name and IP address registry

setup_zenoh.sh
    Primary ROS 2 network configuration

setup_network.sh
    DDS-based fallback configuration

README.md
    Detailed network documentation
```

---

# 2. Install Zenoh Support

Zenoh must be installed on each ROS machine that participates in the ROS 2 network.

This includes:

- the ROS PC
- each Raspberry Pi

Run:

```bash
sudo apt update

sudo apt install ros-jazzy-rmw-zenoh-cpp
```

This normally only needs to be done once on each computer.

---

# 3. Device Registration

All project device IP addresses are managed in:

```text
network/devices.sh
```

Example:

```bash
ROS_PC="10.170.32.181"

DUMPTRUCK_01="10.170.32.192"
DUMPTRUCK_02="10.170.32.193"
DUMPTRUCK_03="10.170.32.194"
DUMPTRUCK_04="10.170.32.45"
DUMPTRUCK_05="10.170.32.219"

EXCAVATOR_01="10.170.32.182"
EXCAVATOR_02="10.170.32.191"
EXCAVATOR_03="10.170.32.222"
EXCAVATOR_04="10.170.32.223"
```

Do not manually type robot IP addresses into every ROS terminal.

Instead, the network setup script uses the device registry.

If a new Raspberry Pi is added to the project, make sure its IP address is added to:

```text
network/devices.sh
```

before attempting multi-machine ROS communication.

---

# 4. Start the Zenoh Router

The Zenoh router normally runs on the main ROS PC.

## ROS PC — Terminal 1

**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router

ros2 run rmw_zenoh_cpp rmw_zenohd
```

This terminal becomes the communication router for the construction robotics system.

Do not close this terminal while the robots are communicating.

Normally, only **one Zenoh router** is needed.

---

# 5. Configure the ROS PC

Open another terminal on the ROS PC.

## ROS PC — Terminal 2

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc
```

The terminal is now configured to communicate through the local Zenoh router.

You can verify the configuration with:

```bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
echo $ZENOH_CONFIG_OVERRIDE
```

Expected values include:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp

ROS_DOMAIN_ID=10
```

The exact Zenoh endpoint is configured automatically by `setup_zenoh.sh`.

---

# 6. Configure a Raspberry Pi

Each Raspberry Pi uses the same setup script but selects its own device profile.

For example, on Dumptruck1:

## Dumptruck1 Raspberry Pi — Terminal 1

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1
```

For Dumptruck3:

```bash
source network/setup_zenoh.sh client dumptruck3
```

For Dumptruck4:

```bash
source network/setup_zenoh.sh client dumptruck4
```

For Dumptruck5:

```bash
source network/setup_zenoh.sh client dumptruck5
```

For Excavator1:

```bash
source network/setup_zenoh.sh client excavator1
```

For Excavator2:

```bash
source network/setup_zenoh.sh client excavator2
```

The script automatically configures the Raspberry Pi to connect to the Zenoh router on the ROS PC.

You do **not** need to manually specify:

```text
ROS_STATIC_PEERS
```

or manually enter the ROS PC IP address.

---

# 7. Connecting Multiple Robots

Multiple robots can communicate through the same Zenoh router.

For example:

```text
ROS PC
│
├── Terminal 1
│   └── Zenoh Router
│
└── Terminal 2
    └── ROS applications / Command Center


Dumptruck1 Raspberry Pi
└── Terminal 1
    └── source network/setup_zenoh.sh client dumptruck1


Dumptruck3 Raspberry Pi
└── Terminal 1
    └── source network/setup_zenoh.sh client dumptruck3


Excavator1 Raspberry Pi
└── Terminal 1
    └── source network/setup_zenoh.sh client excavator1
```

All three robots connect independently to the same ROS PC.

You do not need to add the other robots' IP addresses to each Raspberry Pi.

Conceptually:

```text
                    ROS PC
                       │
                  Zenoh Router
                       │
       ┌───────────────┼───────────────┐
       │               │               │
       ▼               ▼               ▼
    Truck 1         Truck 3        Excavator 1
```

Adding another robot simply adds another Zenoh client.

---

# 8. Verify ROS 2 Communication

After the router and clients are running, check ROS 2 discovery.

On the ROS PC:

```bash
ros2 node list
```

and:

```bash
ros2 topic list
```

You should see nodes and topics from the connected robot.

For example, when Dumptruck1 is running you may see topics such as:

```text
/truck1/cmd_vel
/truck1/wheel_states
```

To inspect a topic:

```bash
ros2 topic echo /truck1/wheel_states
```

To measure the message frequency:

```bash
ros2 topic hz /truck1/wheel_states
```

Allow `ros2 topic hz` to run for approximately 20–30 seconds before evaluating communication quality.

---

# 9. Example: Dump Truck Startup

A typical Dumptruck1 startup looks like this.

## ROS PC — Terminal 1

**Keep running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router

ros2 run rmw_zenoh_cpp rmw_zenohd
```

## Dumptruck1 Raspberry Pi — Terminal 1

**Keep running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

sudo pigpiod

ros2 launch dump_truck_bringup truck1_pi.launch.py
```

## ROS PC — Terminal 2

**Keep running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc

ros2 launch construction_site_control command_center.launch.py \
  trucks:=truck1 \
  start_camera:=true \
  start_apriltag:=true \
  start_localization:=true \
  start_action_servers:=true \
  start_scenario_manager:=false
```

The normal system therefore requires only:

```text
ROS PC T1
    Zenoh Router

ROS PC T2
    Command Center

Robot Pi T1
    Robot Hardware
```

---

# 10. ROS_DOMAIN_ID

The CIC ConRobotics system currently uses:

```text
ROS_DOMAIN_ID=10
```

This is automatically configured by:

```bash
source network/setup_zenoh.sh ...
```

Normally, students should not manually export a different domain ID.

All ROS 2 machines that need to communicate must use the same ROS domain.

---

# 11. Isaac Sim

Isaac Sim can also use ROS 2 communication.

However, the Isaac Sim launch environment is different from a normal ROS terminal and should not automatically be configured using old `ROS_STATIC_PEERS` instructions.

Do **not** use the previous configuration:

```text
ROS_STATIC_PEERS=...
ROS_AUTOMATIC_DISCOVERY=...
```

as the standard project procedure.

When using Isaac Sim, make sure that:

- the Isaac Sim ROS 2 bridge is configured for the intended ROS environment
- the ROS domain matches the rest of the project
- the middleware configuration is compatible with the current project setup

The current physical multi-machine system uses Zenoh.

Isaac Sim integration should be tested for the specific simulation workflow being used before assuming that the same launch configuration applies.

---

# 12. SSH

SSH stands for **Secure Shell**.

SSH allows you to remotely log in to a Raspberry Pi and execute commands without connecting a monitor, keyboard, or mouse directly to the Raspberry Pi.

This is the recommended way to operate the robot computers.

---

# 13. Enable SSH on Raspberry Pi

On the Raspberry Pi:

```bash
sudo apt update

sudo apt install openssh-server -y
```

Enable SSH automatically at startup:

```bash
sudo systemctl enable ssh
```

Start SSH:

```bash
sudo systemctl start ssh
```

Check the service:

```bash
sudo systemctl status ssh
```

You should see:

```text
active (running)
```

---

# 14. Connect to Raspberry Pi Using SSH

From your laptop or ROS PC:

```bash
ssh <username>@<raspberry_pi_ip>
```

For example, Dumptruck1:

```bash
ssh besure@10.170.32.192
```

Dumptruck3:

```bash
ssh besure@10.170.32.194
```

Excavator1:

```bash
ssh besure@10.170.32.182
```

The first time you connect, SSH may ask:

```text
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

Enter:

```text
yes
```

Then enter the Raspberry Pi password.

If you are connecting from outside the university network, make sure the University VPN is connected when required.

---

# 15. VS Code Remote SSH

VS Code provides an extension called:

```text
Remote - SSH
```

This allows you to:

- open Raspberry Pi files
- edit code remotely
- use the Raspberry Pi terminal
- run ROS commands
- use Git directly on the Raspberry Pi

without connecting a physical monitor to the Raspberry Pi.

Install the **Remote - SSH** extension in VS Code.

Then edit your SSH configuration file:

```text
~/.ssh/config
```

Example:

```text
Host dumptruck1
    HostName 10.170.32.192
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck2
    HostName 10.170.32.193
    User dumptruck_02
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck3
    HostName 10.170.32.194
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator1
    HostName 10.170.32.182
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator2
    HostName 10.170.32.191
    User besure
    IdentityFile ~/.ssh/id_ed25519
```

After saving the configuration, you can connect using:

```bash
ssh dumptruck1
```

instead of:

```bash
ssh besure@10.170.32.192
```

The same aliases can be selected from VS Code Remote SSH.

---

# 16. Create an SSH Key

Using an SSH key allows you to log in without typing the Raspberry Pi password every time.

Create the key on your laptop or desktop computer:

```bash
ssh-keygen -t ed25519
```

When asked where to save the key, pressing **Enter** normally accepts the default location:

```text
~/.ssh/id_ed25519
```

This creates:

```text
~/.ssh/id_ed25519
~/.ssh/id_ed25519.pub
```

The `.pub` file is the public key.

---

# 17. Copy the SSH Key to Raspberry Pi

On Linux, macOS, or Git Bash on Windows, use:

```bash
ssh-copy-id <username>@<raspberry_pi_ip>
```

For Dumptruck1:

```bash
ssh-copy-id besure@10.170.32.192
```

For Dumptruck2:

```bash
ssh-copy-id dumptruck_02@10.170.32.193
```

For Dumptruck3:

```bash
ssh-copy-id besure@10.170.32.194
```

For Excavator1:

```bash
ssh-copy-id besure@10.170.32.182
```

For Excavator2:

```bash
ssh-copy-id besure@10.170.32.191
```

After this, test:

```bash
ssh dumptruck1
```

You should normally be able to connect without entering the Raspberry Pi password.

---

# 18. Windows Users

Windows PowerShell normally includes the `ssh` command, but `ssh-copy-id` is not included by default.

The easiest option is to install and use **Git Bash**.

In Git Bash:

```bash
ssh-keygen -t ed25519
```

Then:

```bash
ssh-copy-id besure@10.170.32.192
```

The same SSH keys can still be used by VS Code Remote SSH.

---

# 19. Common Network Checks

## Check the Raspberry Pi IP address

On the Raspberry Pi:

```bash
hostname -I
```

---

## Check whether the ROS PC is reachable

From the Raspberry Pi:

```bash
ping 10.170.32.181
```

Stop with:

```text
Ctrl+C
```

---

## Check whether the Zenoh router port is reachable

If needed, first install Netcat:

```bash
sudo apt install netcat-openbsd
```

Then from the Raspberry Pi:

```bash
nc -vz 10.170.32.181 7447
```

If the Zenoh router is running and the network allows the connection, the connection should succeed.

---

## Check the selected ROS middleware

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_zenoh_cpp
```

---

## Check the ROS domain

```bash
echo $ROS_DOMAIN_ID
```

Expected:

```text
10
```

---

## Check the Zenoh configuration

```bash
echo $ZENOH_CONFIG_OVERRIDE
```

A robot client should show a connection endpoint for the ROS PC Zenoh router.

---

# 20. Common Problems

## No ROS topics appear

Check:

1. Is the Zenoh router running on the ROS PC?
2. Did you source ROS 2?
3. Did you source the repository workspace?
4. Did you run `setup_zenoh.sh`?
5. Are both machines using `ROS_DOMAIN_ID=10`?
6. Are both machines connected to the correct network?
7. Is the robot IP correctly registered in `network/devices.sh`?
8. Is the robot ROS node actually running?

Example:

```bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID

ros2 node list
ros2 topic list
```

---

## `RMW_IMPLEMENTATION` is empty or incorrect

Run the network setup again:

ROS PC:

```bash
source network/setup_zenoh.sh client ros-pc
```

Robot:

```bash
source network/setup_zenoh.sh client dumptruck1
```

---

## The robot cannot connect to the ROS PC

Check basic network communication:

```bash
ping 10.170.32.181
```

Then check the Zenoh router port:

```bash
nc -vz 10.170.32.181 7447
```

---

## SSH works but ROS does not

SSH and ROS 2 communication are different services.

Successful SSH communication proves that the machines can communicate over the network, but it does not prove that ROS 2 is configured correctly.

Check:

```bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
echo $ZENOH_CONFIG_OVERRIDE
```

and confirm that the Zenoh router is running.

---

# 21. DDS Fallback

The project still includes the older DDS network helper:

```text
network/setup_network.sh
```

This is retained for:

- troubleshooting
- compatibility testing
- fallback operation
- communication-layer experiments

It is **not** the standard startup method for the current integrated physical robot system.

Normal operation should use:

```bash
source network/setup_zenoh.sh ...
```

---

# 22. Summary

For normal ROS 2 communication:

```text
ROS PC
│
├── Start ONE Zenoh Router
│
└── Configure ROS applications as:
    client ros-pc


Robot Raspberry Pi
│
└── Configure as:
    client <robot_name>
```

Example:

```bash
# ROS PC — Router terminal
source network/setup_zenoh.sh router
ros2 run rmw_zenoh_cpp rmw_zenohd
```

```bash
# ROS PC — ROS application terminal
source network/setup_zenoh.sh client ros-pc
```

```bash
# Dumptruck1 Raspberry Pi
source network/setup_zenoh.sh client dumptruck1
```

The project network architecture is:

```text
1 Router + 1 Command Center + N Robots
```

Do not manually maintain `ROS_STATIC_PEERS` lists for normal operation.