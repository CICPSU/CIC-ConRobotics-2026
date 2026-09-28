# Lab 01 — ROS 2 Network Setup

In this lab, you will configure remote access to the course ROS computer, connect to a Raspberry Pi, and establish ROS 2 communication between two computers.

The course has two ROS PCs. **One active Zenoh router** serves the system, and the instructor will identify whether it runs on `ros-pc` or `ros-backup-pc`. In Step 10, you will select that router and make sure every ROS client uses it.

By the end of this lab, you will be able to:

- remotely access the ROS computer using SSH
- use VS Code Remote SSH
- clone and build the course repository
- connect to a Raspberry Pi
- configure the course Zenoh network
- run ROS 2 nodes across two computers
- verify ROS 2 communication using a talker and listener

The CIC ConRobotics system uses the following basic network architecture:

```text
Your Laptop
     │
     │ SSH
     │
     ├── Zenoh Router
     │
     └──────────────┐
                    │
                    ▼
               Raspberry Pi
```
or

```text
ROS PC
     │
     │ SSH
     ▼
     │
     ├── Zenoh Router
     │
     └──────────────┐
                    │
                    ▼
               Raspberry Pi
```
For ROS 2 communication, the important architecture is:

```text
1 Zenoh Router
      +
1 ROS PC
      +
N Robot Clients
```

---

# Part A — ROS Computer Setup

## Step 1 — Log In to the ROS Computer

For the initial setup, you must first log in to the ROS computer physically using your Penn State account.

> **Important:** You only need to complete this initial setup once.

### 1. Log in to Ubuntu

Log in to the ROS computer using your Penn State account.

Wait until the Ubuntu desktop has fully loaded.

### 2. Open a Terminal

Click **Show Apps** at the bottom-left corner of the desktop.

<img src="images/step01_show_apps.png" width="900">

Search for **Terminal** and click the **Terminal** application.

<img src="images/step01_terminal_search.png" width="900">

A Terminal window should open.

<img src="images/step01_terminal_open.png" width="900">

You should see a command prompt similar to:

```text
your_psu_id@computer-name:~$
```

Do not close this terminal. You will use this for the next steps.

### 3. Confirm your user account

Run:

```bash
whoami
```

Press **Enter**.

You should see your PSU ID:

```text
your_psu_id
```

---

# Part B — Remote Development Setup

## Step 2 — Create an SSH Key

You will now configure your laptop so that you can remotely access the ROS computer without entering your Penn State password every time.

> **Important:** From this step forward, use **your own laptop**, not the physical ROS computer.

---

### 1. Open a Terminal on YOUR Laptop

Open a terminal on your laptop.

### macOS

Open:

```text
Terminal
```

### Windows

Use:

```text
Git Bash
```

> **Important for Windows users:** Use **Git Bash** for the SSH setup steps in this lab.

Windows PowerShell includes the `ssh` command, but it does not normally include `ssh-copy-id`.

Git Bash provides a Linux-like shell and allows you to use the same SSH commands shown in this lab.

If Git Bash is not installed, install Git for Windows first.

---

### 2. Check for an Existing SSH Key

Before creating a new SSH key, check whether your laptop already has one.

Run:

```bash
ls ~/.ssh/id_ed25519.pub
```

This command works in:

```text
macOS Terminal
Git Bash on Windows
```

If you see a file path similar to:

```text
/Users/your_username/.ssh/id_ed25519.pub
```

or:

```text
/c/Users/your_username/.ssh/id_ed25519.pub
```

you already have an SSH key.

> **Do not create a new key.**

Continue to:

```text
Step 2.5 — Connect to the PSU VPN
```

If you see a message similar to:

```text
No such file or directory
```

continue to the next section.

---

### 3. Generate an SSH Key

Run:

```bash
ssh-keygen -t ed25519
```

Press **Enter**.

You should see a message similar to:

```text
Generating public/private ed25519 key pair.

Enter file in which to save the key (.../.ssh/id_ed25519):
```

Press **Enter** to use the default location.

You will then be asked for a passphrase.

For this course setup, press **Enter** without typing anything.

After completing the prompts, you should see:

```text
Your identification has been saved in ...

Your public key has been saved in ...
```

<img src="images/step03_ssh_keygen_complete.png" width="900">

### Checkpoint

Your SSH key has been successfully created.

---

## Step 2.5 — Connect to the PSU VPN

> **IMPORTANT:** Before attempting to connect to the ROS computer remotely, connect your laptop to the Penn State VPN.

Access https://www.it.psu.edu/software/ and download vpn.
Connect to the global protect.

### Checkpoint — PSU VPN

Before continuing, confirm that:

- [ ] Your laptop is connected to the Penn State VPN.
- [ ] The VPN connection is active.

> Keep the VPN connected while accessing the course ROS computers remotely.

---

## Step 3 — Copy Your SSH Key to the ROS Computer

Next, copy your SSH key to the ROS computer.

This allows you to connect without entering your Penn State password every time.

> **Important:** Keep the PSU VPN connected during this step.

---

### 1. Identify the ROS Computer

Use the ROS computer assigned to you for the lab.

Current course ROS computers include:

```text
ROS-PC-1    will be provided
ROS-PC-2    will be provided
```

> **Important:** Use the ROS computer assigned by the instructor.

---

### 2. Copy Your SSH Key

On your laptop, run:

```bash
ssh-copy-id 'YOUR_PSU_ID@AD.PSU.EDU'@ROS_COMPUTER_IP
```

Replace:

- `YOUR_PSU_ID` with your Penn State user ID
- `ROS_COMPUTER_IP` with the assigned ROS computer IP

Example:

```bash
ssh-copy-id 'abc123@AD.PSU.EDU'@10.170.xx.xxx
```

### Windows Users

Run this command from:

```text
Git Bash
```

Do **not** use PowerShell for the `ssh-copy-id` step.

Example:

```bash
ssh-copy-id 'abc123@AD.PSU.EDU'@xx.xxx.xx.xxx
```

---

### 3. Confirm the First Connection

If this is your first connection, you may see:

```text
The authenticity of host '10.170.xx.xxx (10.170.xx.xxx)' can't be established.

ED25519 key fingerprint is SHA256:...

Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

<img src="images/step04_ssh_first_connection.png" width="900">

Type:

```text
yes
```

and press **Enter**.

This message is normal.

---

### 4. Enter Your Penn State Password

You may be asked for your Penn State password.

Your password will not appear while you type.

You will not see:

```text
letters
dots
asterisks
```

This is normal.

<img src="images/step04_ssh_first_connection.png" width="900">

If the SSH key was copied successfully, you should see:

```text
Number of key(s) added: 1
```


---

### 5. Test the SSH Connection

Run:

```bash
ssh 'YOUR_PSU_ID@AD.PSU.EDU'@ROS_COMPUTER_IP
```

Example:

```bash
ssh 'abc123@AD.PSU.EDU'@10.170.xx.xxx
```

If configured correctly, you should connect without entering your Penn State password.

The prompt should look similar to:

```text
abc123@AD.PSU.EDU@E5-AE-ROS-PC:~$
```


### Checkpoint — Passwordless SSH Connection

Confirm that:

- [ ] The PSU VPN is connected.
- [ ] Your SSH key was copied to the ROS computer.
- [ ] You can connect from your laptop.
- [ ] You are not asked for your Penn State password.

To return to your laptop:

```bash
exit
```

---

## Step 4 — Configure VS Code Remote SSH

You will now configure **Visual Studio Code (VS Code)** to remotely access the ROS computer.

After completing this setup, you will be able to edit files and run commands directly on the ROS computer from your laptop.

> **Important:** Complete this step on **your own laptop**.

> Keep the PSU VPN connected while using the remote ROS computers.

---

### 1. Open VS Code

Open **Visual Studio Code**.
<img src="images/open_VS_Code.png" width="900">
---

### 2. Install Remote - SSH

Open the **Extensions** panel.
<img src="images/VS_Code_Extenion.png" width="900">
Search for:

```text
Remote - SSH
```

Install the extension from Microsoft.

If it is already installed, continue.

---

### 3. Open the SSH Configuration File

Open the VS Code **Command Palette**.

### macOS

```text
Command + Shift + P
```

### Windows

```text
Ctrl + Shift + P
```

Search for:

```text
Remote-SSH: Open SSH Configuration File...
```
<img src="images/Command_Palette.png" width="900">

Select your user SSH configuration file.

### macOS/Linux

```text
~/.ssh/config
```

### Windows

```text
C:\Users\YOUR_USERNAME\.ssh\config
```

---

### 4. Add the Course Computers

Add the following configuration:

```text
Host dumptruck1
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck3
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck4
    HostName 10.170.xx.xx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host dumptruck5
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator1
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator2
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator3
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator4
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator5
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator6
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host excavator7
    HostName 10.170.xx.xxx
    User besure
    IdentityFile ~/.ssh/id_ed25519

Host ROS-PC-1
    HostName 10.170.xx.xxx
    User YOUR_PSU_ID@AD.PSU.EDU
    IdentityFile ~/.ssh/id_ed25519

Host ROS-PC-2
    HostName 10.170.xx.xxx
    User YOUR_PSU_ID@AD.PSU.EDU
    IdentityFile ~/.ssh/id_ed25519
```

Replace:

```text
YOUR_PSU_ID
```

with your Penn State user ID.

> **Important:** Do not change `HostName` unless instructed to do so.

Save the file.

---

### 5. Connect to the ROS Computer

Open the Command Palette again.

Search for:

```text
Remote-SSH: Connect to Host...
```

Select:

```text
ROS-PC-1
```

or:

```text
ROS-PC-2
```

depending on your assignment.

A new VS Code window should open.

If VS Code asks for the operating system of the remote computer, select:

```text
Linux
```

---

### 6. Confirm the Remote Connection

Look at the bottom-left corner of VS Code.

You should see something similar to:

```text
SSH: ROS-PC-1
```

<img src="images/step05_ssh_login_success.png" width="900">

### Checkpoint — VS Code Remote Connection

Confirm that:

- [ ] Remote - SSH is installed.
- [ ] Your ROS computer appears in the host list.
- [ ] You can connect to the ROS computer.
- [ ] VS Code shows the active remote connection.

> Even though VS Code is displayed on your laptop, commands in this remote window are now running on the **ROS computer**.

---

# Part C — Course Repository Setup

## Step 5 — Clone the Course Repository

You will now download the course GitHub repository to the ROS computer.

> **Important:** Make sure your VS Code window is remotely connected to the ROS computer.

---

### 1. Open a Terminal in VS Code

Select:

```text
Terminal → New Terminal
```

The prompt should look similar to:

```text
your_psu_id@AD.PSU.EDU@E5-AE-ROS-PC:~$
```

---

### 2. Create the Course Workspace

Run:

```bash
mkdir -p ~/ws_conrobotics
cd ~/ws_conrobotics
```

---

### 3. Clone the Course Repository

Run:

```bash
git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git
```

Enter the repository:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
```

---

### 4. Select the Course Branch

For the current course development environment:

```bash
git checkout dev
```

Confirm:

```bash
git branch --show-current
```

Expected:

```text
dev
```

---

### 5. Check the Repository

Run:

```bash
ls
```

You should see directories such as:

```text
robots
common
perception
command_center
operations
network
labs
docs
tools
```

### Checkpoint — Repository Downloaded

Confirm that:

- [ ] VS Code is remotely connected to the ROS computer.
- [ ] The repository was successfully cloned.
- [ ] You are on the correct branch.
- [ ] You are inside `CIC-ConRobotics-2026`.
- [ ] You can see the repository contents.

---

## Step 6 — Build the ROS 2 Workspace

Next, build the ROS 2 packages used by the course.

### 1. Set Up ROS 2

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
```

The `source` command configures the current Terminal so it can find ROS 2.

---

### 2. Build the Workspace

Run:

```bash
colcon build --symlink-install
```

The build may take some time.

When complete, the build summary should show that the packages finished successfully.

---

### 3. Load the Course Workspace

Run:

```bash
source install/setup.bash
```

You must source both ROS 2 and the course workspace when opening a new Terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_conrobotics/CIC-ConRobotics-2026/install/setup.bash
```

### Checkpoint — ROS 2 Workspace Built

Confirm that:

- [ ] `colcon build --symlink-install` completed without errors.
- [ ] The `install` directory was created.
- [ ] You successfully ran `source install/setup.bash`.

---

# Part D — Connect to the Raspberry Pi

## Step 7 — Connect to Your Assigned Raspberry Pi

You will now connect to a Raspberry Pi used in the course robotics system.

> **Important:** Each student or group must use the Raspberry Pi assigned to them.

> Do **not** connect to another group's Raspberry Pi. Multiple students controlling the same robot can interfere with each other's work.

---

### 1. Confirm Your Assigned Raspberry Pi

Your instructor will provide your assigned Raspberry Pi.

Examples include:

```text
dumptruck1
dumptruck3
dumptruck4
dumptruck5
excavator1
excavator2
excavator3
excavator4
excavator5
excavator6
excavator7
```

---

### 2. Open a Second VS Code Window

Keep your ROS PC VS Code window open.

Open another VS Code window and use:

```text
Remote-SSH: Connect to Host...
```

Select your assigned Raspberry Pi.

For example:

```text
dumptruck1
```

---

### 3. Confirm the Raspberry Pi Connection

The Raspberry Pi terminal prompt should look similar to:

```text
besure@Dumptruck1:~$
```

You should now have two clearly separate VS Code windows:

```text
VS Code Window 1
    │
    └── ROS PC


VS Code Window 2
    │
    └── Raspberry Pi
```

Your laptop is connecting independently to both machines:

```text
                    Your Laptop
                    /         \
                   /           \
                  ▼             ▼
              ROS PC       Raspberry Pi
```

> **Important:** Always check the terminal prompt before running commands.

### Checkpoint — Raspberry Pi Connection

Confirm that:

- [ ] You are using the Raspberry Pi assigned to your group.
- [ ] You can connect using VS Code Remote SSH.
- [ ] The Raspberry Pi hostname appears in the terminal prompt.
- [ ] You can clearly distinguish the ROS PC and Raspberry Pi VS Code windows.

---

# Part E — Configure ROS 2 Communication


---

## Step 8 — Select the Correct Router

Before starting ROS 2 nodes, ask the instructor **which PC is hosting the active Zenoh router**.

| Router host | Router profile to use |
|---|---|
| Primary ROS PC | `ros-pc` |
| Backup ROS PC | `ros-backup-pc` |

Use the instructor's choice for every participating computer. The ROS PC running your applications and the PC hosting the router may be different.

### 1. Identify Your Computer and the Active Router

Open `network/devices.sh` in the course repository. This file contains the device profiles and their assigned IP addresses.

Match your assigned ROS PC's SSH address from Step 4 to its entry in `network/devices.sh`. The SSH aliases `ROS-PC-1` and `ROS-PC-2` are connection names; confirm which network profile each represents with the instructor.

Before continuing, record:

| Item | Your assignment |
|---|---|
| ROS PC application profile | `ros-pc` or `ros-backup-pc` |
| Raspberry Pi profile | Your assigned robot, such as `dumptruck1` |
| Active router profile | Instructor-selected `ros-pc` or `ros-backup-pc` |

Look up the selected router's IP in `network/devices.sh`; you will compare it with the configuration output below. Do not edit device addresses to select a router.

### 2. Understand the Two Profile Arguments

The client command has this form:

```text
source network/setup_zenoh.sh client <this-computer-profile> <active-router-profile>
```

For example:

```bash
source network/setup_zenoh.sh client dumptruck1 ros-backup-pc
```

This configures **Dumptruck1** to use the router on **the backup ROS PC**.

> **Important:** Always include the final router profile in this lab. The network README specifies `ros-pc` as the default when the router argument is omitted. Selecting a router profile configures the current terminal; it does not connect your SSH session to another computer or start a router there.

Run the setup command again in **every new ROS terminal**, using that terminal's computer profile and the same instructor-selected router.

### Checkpoint — Router Selected

- [ ] I know the network profile of my assigned ROS PC and Raspberry Pi.
- [ ] The instructor has identified the active router host.
- [ ] I have located that router's IP in `network/devices.sh`.

---

## Step 9 — Start or Confirm the Zenoh Router

**Machine: the instructor-selected router host.**

If the instructor or another group already runs the shared router, confirm it is running on the selected host and continue to Step 12. **Do not start a second router.**

If you are assigned to start it, open a VS Code Remote SSH terminal on the selected router host. Check the connection indicator and run:

```bash
hostname
```

Confirm this is the computer identified by the instructor, then prepare the terminal:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Run **only one** of these commands, matching the host you are connected to.

**Primary ROS PC hosts the router:**

```bash
source network/setup_zenoh.sh router ros-pc
```

**Backup ROS PC hosts the router:**

```bash
source network/setup_zenoh.sh router ros-backup-pc
```

Check the selected router and compare its IP with `network/devices.sh`:

```bash
echo "Router: $CIC_ZENOH_ROUTER"
echo "Router IP: $CIC_ZENOH_ROUTER_IP"
```

If these match the instructor's selection, start the router:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Keep this terminal running.** The application terminals and robot clients will use this router.

---

## Step 10 — Configure the ROS PC and Start the Talker

Open a **new terminal on your assigned application ROS PC**. Keep the router terminal running if it is on this computer.

Prepare this terminal:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 1. Select the Correct Client Command

Run **only the row matching both your current computer and the active router**:

| Computer running this terminal | Active router | Command |
|---|---|---|
| `ros-pc` | `ros-pc` | `source network/setup_zenoh.sh client ros-pc ros-pc` |
| `ros-pc` | `ros-backup-pc` | `source network/setup_zenoh.sh client ros-pc ros-backup-pc` |
| `ros-backup-pc` | `ros-pc` | `source network/setup_zenoh.sh client ros-backup-pc ros-pc` |
| `ros-backup-pc` | `ros-backup-pc` | `source network/setup_zenoh.sh client ros-backup-pc ros-backup-pc` |

### 2. Verify the Router Before Running the Talker

Run:

```bash
hostname
echo "Device: $CIC_ZENOH_DEVICE"
echo "Router: $CIC_ZENOH_ROUTER"
echo "Router IP: $CIC_ZENOH_ROUTER_IP"
echo "Middleware: $RMW_IMPLEMENTATION"
echo "ROS domain: $ROS_DOMAIN_ID"
echo "Zenoh configuration: $ZENOH_CONFIG_OVERRIDE"
```

Confirm:

- **Device** matches the ROS PC profile you are using.
- **Router** matches the instructor-selected router.
- **Router IP** matches that profile's address in `network/devices.sh`.
- **Middleware** is `rmw_zenoh_cpp` and **ROS domain** is `10`.
- In the Zenoh configuration, the connection endpoint is `tcp/127.0.0.1:7447` when this PC also hosts the active router. If the router is on the other PC, the endpoint must use that router's IP on port `7447`.

> If the selected router is wrong or the values are blank, stop here. Open a fresh terminal, source ROS and the workspace, and use the correct row above. If the values remain wrong, ask the instructor to check the setup script. Do not manually change the environment variables.

These checks confirm the terminal's configuration. The talker/listener test below checks actual message transfer.

### 3. Start the Talker

```bash
ros2 run demo_nodes_cpp talker
```

You should see:

```text
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
[INFO] [talker]: Publishing: 'Hello World: 3'
```

**Keep this terminal running.**

<img src="images/Listner-Talker.png" width="900">
---

## Step 11 — Configure the Raspberry Pi and Start the Listener

Go to your assigned Raspberry Pi VS Code window and open a terminal.

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 1. Select the Same Active Router

Use your assigned Pi profile: `dumptruck1`, `dumptruck3`, `dumptruck4`, `dumptruck5`, `excavator1`, `excavator2`, or `excavator3`.

The following examples use `dumptruck1`. **Replace `dumptruck1` with your assigned Pi profile** and run only the command for the instructor-selected router.

**If the active router is `ros-pc`:**

```bash
source network/setup_zenoh.sh client dumptruck1 ros-pc
```

**If the active router is `ros-backup-pc`:**

```bash
source network/setup_zenoh.sh client dumptruck1 ros-backup-pc
```

### 2. Compare the Pi Configuration with the ROS PC

```bash
hostname
echo "Device: $CIC_ZENOH_DEVICE"
echo "Router: $CIC_ZENOH_ROUTER"
echo "Router IP: $CIC_ZENOH_ROUTER_IP"
echo "Middleware: $RMW_IMPLEMENTATION"
echo "ROS domain: $ROS_DOMAIN_ID"
echo "Zenoh configuration: $ZENOH_CONFIG_OVERRIDE"
```

For Dumptruck1 using the backup router, the relevant output should be:

```text
Device: dumptruck1
Router: ros-backup-pc
Router IP: <ros-backup-pc address from network/devices.sh>
Middleware: rmw_zenoh_cpp
ROS domain: 10
```

The IP placeholder above represents the actual address shown by your terminal.

Compare the Pi's output with the ROS PC's output from Step 12:

| Check | Required result |
|---|---|
| Device | Each terminal shows its own assigned computer profile. |
| Router | Both show the same instructor-selected router profile. |
| Router IP | Both match the selected router's IP in `network/devices.sh`. |
| Middleware and ROS domain | Both show `rmw_zenoh_cpp` and `10`. |
| Pi connection endpoint | Uses the selected ROS PC's IP on port `7447`; it must not use `127.0.0.1`, which would refer to the Pi itself. |

> **Do not continue if the router names or router IPs differ.** Open a fresh Pi terminal, source ROS and the workspace, and select the correct router again. Checking only `RMW_IMPLEMENTATION` is not enough to identify the selected router.

<!-- Suggested image: images/step13_router_match.png. Show ROS PC and Pi terminals side by side; highlight the matching Router and Router IP values and their distinct Device values. -->

### 3. Start the Listener

```bash
ros2 run demo_nodes_cpp listener
```

If communication is working, you should see:

```text
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
[INFO] [listener]: I heard: [Hello World: 3]
```



---

## Step 12 — Understand What Just Happened

When the application ROS PC also hosts the selected router, your system looks like this:

```text
ROS PC
│
├── Terminal 1
│   │
│   └── Zenoh Router
│
└── Terminal 2
    │
    └── ROS 2 Talker
             │
             │
             ▼
        Zenoh Router
             │
             ▼
      Raspberry Pi
             │
             └── ROS 2 Listener
```
or
```text
Your Laptop
│
├── Terminal 1
│   │
│   └── Zenoh Router
│
└── Terminal 2
    │
    └── ROS 2 Talker
             │
             │
             ▼
        Zenoh Router
             │
             ▼
      Raspberry Pi
             │
             └── ROS 2 Listener
```

The talker and listener are running on **different computers**.

Zenoh transports the ROS 2 messages between them.

If the router runs on the other ROS PC, both the talker computer and the Raspberry Pi connect to that selected router instead. The device profiles differ, but their `CIC_ZENOH_ROUTER` and `CIC_ZENOH_ROUTER_IP` values must match.

The same architecture will later be used for:

```text
ROS PC
│
├── Camera
├── AprilTag
├── Localization
├── Command Center
└── Zenoh Router
        │
        ├── Dump Truck
        ├── Dump Truck
        └── Excavator
```

The important concept is:

```text
1 Router + N ROS 2 Clients
```

You do **not** manually configure a list of ROS peers.

---

# Part F — Verify ROS 2 Communication

## Step 13 — Inspect the ROS Network

While the talker and listener are running, open another Zenoh-configured ROS PC terminal.

Run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Run the **same complete client command you selected in Step 12**, including both the computer profile and the active router profile. Each new terminal needs its own configuration.

Check before continuing:

```bash
echo "Router: $CIC_ZENOH_ROUTER"
echo "Router IP: $CIC_ZENOH_ROUTER_IP"
```

These must match the router selected in Step 10. Do not use an abbreviated command that omits the router profile.

Check the nodes:

```bash
ros2 node list
```

You should see nodes corresponding to the talker and listener.

Check topics:

```bash
ros2 topic list
```

You should see:

```text
/chatter
```

Inspect the messages:

```bash
ros2 topic echo /chatter
```

You should see the same `Hello World` messages.

Press:

```text
Ctrl + C
```

to stop `ros2 topic echo`.

---

## Checkpoint — ROS 2 Communication

Confirm that:

- [ ] One intended Zenoh router is running on the instructor-selected ROS PC.
- [ ] The talker terminal was configured with its own PC profile and the selected router profile.
- [ ] The Raspberry Pi was configured using its correct robot profile.
- [ ] Both clients show the same instructor-selected `CIC_ZENOH_ROUTER`.
- [ ] Both clients show the same `CIC_ZENOH_ROUTER_IP`, matching `network/devices.sh`.
- [ ] Both clients report `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and `ROS_DOMAIN_ID=10`.
- [ ] The talker is running on the ROS computer.
- [ ] The listener is running on the Raspberry Pi.
- [ ] The Raspberry Pi receives multiple `Hello World` messages.

If all items are complete:

> **Congratulations! You have successfully established ROS 2 communication between two computers using Zenoh.**

---

# Part G — Lab Submission

## Step 14 — Submit Your Result

State your assigned ROS PC, Raspberry Pi, and the **instructor-selected active router**.

Submit **one combined screenshot** showing the ROS PC and Pi terminals side by side. If the text would be too small, submit two readable screenshots instead.

Your evidence must clearly show:

- the ROS PC and Raspberry Pi hostnames or VS Code Remote SSH connection indicators
- each terminal's `Device`, `Router`, `Router IP`, `Middleware`, and `ROS domain` output from Steps 12 and 13
- the **same selected router name and router IP on both computers**
- the talker publishing on the ROS PC and at least five `I heard: [Hello World: ...]` messages on the Pi

Capture the configuration output before it scrolls out of view. If necessary, include an additional screenshot of the checks.

> A screenshot showing only `rmw_zenoh_cpp` does not identify which PC's router was selected. Include the router name and IP as well as the received messages.

---

# Before You Leave

Stop running ROS 2 nodes using:

```text
Ctrl + C
```

Stop the talker.

Stop the listener.

Stop the Zenoh router only if you are responsible for it and the instructor confirms that no other group needs it. **Do not stop a shared router while other groups are using it.**

You can then close your VS Code Remote SSH connections.

---

# Lab Complete

You have now:

- configured SSH on the ROS computer
- created and installed an SSH key
- connected to the PSU VPN
- connected to the ROS computer remotely
- configured VS Code Remote SSH
- cloned the course GitHub repository
- built the ROS 2 workspace
- connected to a Raspberry Pi
- installed ROS 2 Zenoh support
- selected the instructor-designated router on `ros-pc` or `ros-backup-pc`
- started or confirmed the shared Zenoh router
- configured ROS 2 Zenoh clients and verified matching router names and IP addresses
- tested ROS 2 communication between two computers
- inspected ROS 2 nodes and topics across the network

You are now ready to use the course multi-machine robotics system in future labs.

The network architecture you will continue using is:

```text
1 Zenoh Router
      +
1 Command Center
      +
N Robots
```