# Lab 03 — ROS 2 Dump Truck Control

In this lab, your team will control a physical model dump truck using ROS 2.

You will:

1. Connect to your assigned dump truck
2. Identify the main parts of the system
3. Start the dump truck ROS 2 system
4. Use a YAML file to move forward and backward
5. Add a dump action
6. Observe the ROS 2 system using `rqt_graph`
7. Record a short driving and dumping sequence

---

# Part 1 — Prepare Your Dump Truck

## Step 1 — Connect to Your Assigned Truck

Using **VS Code Remote SSH**, connect as you did in Lab 01.

Keep two VS Code windows open:

- One connected to your assigned **ROS PC**
- One connected to your assigned **Dump Truck Pi**

This lab uses **Truck 1** as the example.

| Assigned truck | SSH / Zenoh profile | ROS robot name |
|---|---|---|
| Truck 1 | `dumptruck1` | `truck1` |
| Truck 3 | `dumptruck3` | `truck3` |
| Truck 4 | `dumptruck4` | `truck4` |
| Truck 5 | `dumptruck5` | `truck5` |

Replace `dumptruck1` and `truck1` with your assigned names throughout the lab.

> **Picture placeholder:** Two VS Code windows showing the ROS PC and assigned Dump Truck Pi connections.

---

## Step 2 — Prepare the Workspace

On the **ROS PC**, use the course repository from Labs 01 and 02.

The **repository** is the folder containing the course's robot code, configuration files, and labs. Before updating it, copy your previous YAML files and screenshots to a folder outside `CIC-ConRobotics-2026` so you can keep your work. Follow the instructor's directions for getting the current course version.

Then run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

These commands:

1. Open the course repository.
2. Load ROS 2 Jazzy into the terminal.
3. Build the course packages so ROS 2 can run them.
4. Load the built packages into the terminal.

Wait until the build finishes successfully. If the build fails, ask the instructor before continuing.

The instructor will confirm that the **Dump Truck Pi** also has the current software built.

### Checkpoint

- [ ] We are connected to the correct ROS PC and Dump Truck Pi.
- [ ] The ROS PC build completed successfully.
- [ ] The instructor confirmed that the truck is ready.

---

# Part 2 — Explore the Dump Truck System

## Step 3 — Identify the Main Components

Find each component before starting the truck.

| Component | What it does |
|---|---|
| ROS PC | Runs camera processing, position estimation, and waypoint control |
| Raspberry Pi | Controls the truck's motors and dump bed; reads the wheel encoders |
| Overhead camera and AprilTag | Help locate the truck in the test area |
| Wheel encoders | Measure wheel rotation |
| Left and right wheel motors | Turn the wheels to move the truck. Different wheel speeds turn the truck. |
| Dump bed (loading container) | The container at the back of the truck that carries material. It tilts upward to unload and then lowers. |

The system uses two sources of position information:

- **Wheel odometry:** Estimates movement from wheel rotation. Wheel slip can cause error.
- **AprilTag measurements:** Use the camera to locate the truck in the site coordinate system.

**Fused odometry** uses the initial AprilTag pose and wheel movement to estimate the truck's position. Depending on the configured settings, later AprilTag measurements also correct that estimate.

The camera and wheel encoders provide information used to estimate the truck's position. The ROS PC reads your YAML waypoints and sends driving and dumping commands to the Raspberry Pi. The Pi controls the wheel motors and the mechanism that tilts the loading container.

ROS 2 messages travel between the ROS PC and Pi through **Zenoh**, as in Lab 01.

> **Picture placeholder:** A labeled photo showing the Raspberry Pi, AprilTag, left/right wheel motors, and dump bed (loading container).

Before continuing, ask the instructor to show your group how to stop the truck. Only one group member should send movement commands at a time.

---

# Part 3 — Start the Dump Truck ROS 2 System

## Step 4 — Start the Zenoh Router

Use your assigned ROS PC as the Zenoh router, as in Lab 2. Run only **one router on that PC**. If it is already running, continue to Step 5.

Start Zenoh in **Terminal 1** and keep it running.

If you are using **ROS-PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh router ros-pc
ros2 run rmw_zenoh_cpp rmw_zenohd
```

If you are using **ROS-Backup-PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh router ros-backup-pc
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Use only the block for your assigned PC. The Pi and all client terminals must connect to that PC's router.

---

## Step 5 — Start Your Dump Truck

Switch to the VS Code window connected to your **Dump Truck Pi**.

Keep hands clear of the dump bed during startup.

If your group is using **ROS-PC as the Zenoh router**, run on the **Dump Truck Pi**:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client dumptruck1

sudo pigpiod

ros2 launch dump_truck_bringup \
  dump_truck_pi.launch.py \
  truck_name:=truck1
```

If your group is using **ROS-Backup-PC as the Zenoh router**, run on the **Dump Truck Pi**:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client dumptruck1 ros-backup-pc

sudo pigpiod

ros2 launch dump_truck_bringup \
  dump_truck_pi.launch.py \
  truck_name:=truck1
```

Use only the block for your assigned router.

Ask the instructor for the password if prompted. If `pigpiod` is already running, leave it running and continue with the launch command.

Keep this terminal running.

---

## Step 6 — Start the Command Center

The Command Center starts the camera, AprilTag detection, truck localization, and task server.

**One person starts the shared Command Center.** If it is already running with your truck included, do not start another copy.

Before starting, make sure the three fixed floor tags—**16, 17, and 18**—are visible to the overhead camera.

On **Terminal 2**, use the block for your assigned ROS PC.

If you are using **ROS-PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc

ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:=truck1 \
  start_scenario_manager:=false
```

If you are using **ROS-Backup-PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-backup-pc ros-backup-pc

ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:=truck1 \
  start_scenario_manager:=false
```

For this lab, select only your assigned truck. Selecting several trucks together will be used in a future lab.

Keep this terminal running. Wait for:

```text
SITE REGISTRATION LOCKED
```

This means the camera has been aligned with the site's coordinate system.

After registration is locked, temporary blocking of the **floor tags** is allowed. Keep the **truck's tag** visible for its position measurements.

Do not move the camera or fixed floor tags after registration. If either moves, stop operation and ask the instructor to update any relocated landmark coordinates and restart the full Command Center.

Your terminal layout should now be:

| Computer | Terminal | Purpose |
|---|---|---|
| ROS PC | 1 | Shared Zenoh router |
| ROS PC | 2 | Shared Command Center |
| Dump Truck Pi | 1 | Truck hardware |
| ROS PC | 3, opened next | Check position and send tasks |

Use the same terminal layout on ROS-Backup-PC.

> **Picture placeholder:** The running ROS PC and Pi terminals, including the registration message.

---

# Part 4 — Check the Truck's Position

## Step 7 — Check ROS 2 Communication

Open **Terminal 3** on your assigned ROS PC.

If you are using **ROS-PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
```

If you are using **ROS-Backup-PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-backup-pc ros-backup-pc
```

First, confirm site registration:

```bash
ros2 topic echo /truck1/tag_odom_fusion/site_registration --once --full-length
```

Look for `LOCKED` in the reported state.

List the available ROS 2 actions:

```bash
ros2 action list
```

Look for `/truck1/execute_robot_task` (using your assigned truck number). This is the interface used to send the waypoint task. Listing it does not move the truck.

Check the truck's estimated position:

```bash
ros2 topic echo /truck1/fused_odom --once
```

Look under `pose` → `pose` → `position` for `x` and `y`.

These values are the truck's estimated coordinates in **meters**. They are not the distance from where the truck started this lab.

Take a screenshot and save it as:

```text
groupX_position.png
```

### Checkpoint

- [ ] Site registration is locked.
- [ ] Our truck's `/execute_robot_task` action appears in the list.
- [ ] The truck's AprilTag is visible to the camera.
- [ ] `/truck1/fused_odom` returns a message.
- [ ] The estimated position agrees with the truck's location in the test area.

If the position does not look correct, ask the instructor before sending a task.

---

# Part 5 — Move Forward

## Step 8 — Create a One-Waypoint YAML File

In the **ROS PC** VS Code window, create:

```text
~/ws_conrobotics/CIC-ConRobotics-2026/operations/dump_truck/waypoints/GroupX_lab03.yaml
```

Replace `GroupX` with your group number. Use the same filename in the command in Step 9.

Add:

```yaml
waypoints:
  - [1, 0, 1]
```

**`[1, 0, 1]` means drive forward to x = 1 m, y = 0 m.** The first two numbers are the target coordinates; the last `1` selects forward movement. Starting at (0, 0) and facing +X, the truck travels 1 m in the positive X direction.

Each entry follows this format:

```text
[x, y, direction, optional action]
```

| Value | Meaning |
|---|---|
| First value | Target x-coordinate in meters |
| Second value | Target y-coordinate in meters |
| `1` in the third position | Drive forward |
| `-1` in the third position | Drive backward |
| `dump` in the fourth position | Dump after reaching that waypoint |

The third value is a direction, not a speed or angle.

Save the file. Use spaces for indentation.

> **Picture placeholder:** The YAML file open in VS Code beside a labeled site photo showing A, B, and the positive x/y directions.

---

## Step 9 — Run the Forward Movement

For this example:

- **A = (0, 0):** Starting location
- **B = (1, 0):** Forward destination along +X

The instructor must confirm that both points and the route are inside the clear test area. If different coordinates are needed, use the instructor's coordinates in your YAML.

Have the instructor position the truck at **A = (0, 0)**, facing **+X** toward **B = (1, 0)**. Confirm the position estimate is correct after placement.

On **ROS PC — Terminal 3**, run:

```bash
ros2 action send_goal \
  /truck1/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck1, task_type: waypoint, task_file: '~/ws_conrobotics/CIC-ConRobotics-2026/operations/dump_truck/waypoints/GroupX_lab03.yaml'}" \
  --feedback
```

Run the task command and Command Center on your assigned ROS PC under the **same user account**. The task server expands `~` to that account's home folder and reads your saved YAML file.

Watch the truck. It should drive forward toward B and stop when it reaches the controller's arrival tolerance.

Wait for the result to show:

```text
success: true
```

> **If motion is unexpected, kill the running truck launch in the terminal on the Pi:** switch to the **Dump Truck Pi** window, select the terminal running `dump_truck_pi.launch.py`, and press **Ctrl+C**. Confirm that the truck stops. If it does not, use the instructor-demonstrated physical stop/power control. Stopping only the task terminal on the ROS PC does not stop the Pi's hardware program. Ask the instructor to stop the active task before restarting the Pi launch.

### Checkpoint

- [ ] The task was accepted.
- [ ] The truck moved forward toward B.
- [ ] The truck stopped and the task completed.

---

# Part 6 — Add Backward Movement

## Step 10 — Use Two Waypoints

After the first task finishes, change the same YAML file to:

```yaml
waypoints:
  - [1, 0, 1]
  - [0, 0, -1]
```

The truck reads the list from top to bottom:

1. `[1, 0, 1]`: Drive forward along **+X** to **B = (1, 0)**.
2. `[0, 0, -1]`: Reverse along **−X** to **A = (0, 0)**.

The y-coordinate stays at `0` for both waypoints.

Save the file.

Have the instructor reset the truck to A, facing B, and confirm its position estimate.

Run the **same command from Step 9** again.

### Checkpoint

- [ ] The truck drove forward to B.
- [ ] The truck reversed toward A.
- [ ] The truck stopped and the task completed.

---

# Part 7 — Add the Dump Action

## Step 11 — Move, Reverse, and Dump

After the previous task finishes, add `dump` to the second waypoint:

```yaml
waypoints:
  - [1, 0, 1]
  - [0, 0, -1, dump]
```

**`[1, 0, 1]` drives forward to (1, 0). `[0, 0, -1, dump]` reverses to (0, 0), then dumps.** Both waypoints stay on the X axis.

The truck should now:

1. Drive forward to B.
2. Reverse to A.
3. Stop and raise the dump bed.
4. Lower the bed and finish the task.

Save the file. Have the instructor reset the truck to A, facing B, and confirm its position estimate.

Run the **same command from Step 9** again.

Keep hands clear of the moving truck and dump bed. Wait for the complete dump cycle and task result before sending another task.

### Checkpoint

- [ ] The truck completed both waypoints.
- [ ] The bed raised at the second waypoint.
- [ ] The bed returned to its lowered position.
- [ ] The truck stopped and the task completed.

---

# Part 8 — Look at the ROS Graph

## Step 12 — Open `rqt_graph`

Open **Terminal 4** on your assigned ROS PC.

If you are using **ROS-PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
rqt_graph
```

If you are using **ROS-Backup-PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-backup-pc ros-backup-pc
rqt_graph
```

`rqt_graph` shows the ROS 2 nodes and the topics connecting them.

Refresh the graph while the truck is stationary. Save a screenshot as:

```text
groupX_graph.png
```

Find your truck's topics:

| Topic | What it carries |
|---|---|
| `/truck1/odom` | Position estimated from wheel movement |
| `/truck1/fused_odom` | Combined position estimate |
| `/truck1/cmd_vel` | Driving and turning commands |

Keep the graph open for the final demonstration. Refresh it while the task is running and save another screenshot as:

```text
groupX_graph_action.png
```

Compare the screenshots. Connections may remain the same even when the messages change; a moving truck does not necessarily create new graph connections.

> **Picture placeholder:** Example `rqt_graph` screenshot with the assigned truck's nodes and topics labeled.

---

# Part 9 — Final Demonstration

## Step 13 — Record Your Sequence

Record a short video showing your truck:

1. Moving forward
2. Moving backward
3. Dumping and lowering the bed

Use your final two-waypoint YAML file.

---

# Submission

Submit **one set per group**:

- Short video of the final sequence
- `GroupX_lab03.yaml`
- `groupX_position.png`
- `groupX_graph.png`
- `groupX_graph_action.png`

---

# If Something Does Not Work

Use **ROS PC — Terminal 3**, with the setup from Step 7.

| What you see | What to check |
|---|---|
| `WAITING_FOR_LANDMARKS` or `visible=2/3` | Floor tags 16, 17, and 18 must all be visible together during startup. Check for obstructions or glare. |
| No action server | Confirm the Command Center includes your truck and both computers use the same Zenoh router. |
| No fused odometry | Confirm the truck Pi is running and the truck's AprilTag is visible. Ask the instructor to check wheel odometry if needed. |
| Task file not found | Check the filename and make sure the Command Center can read the YAML at the submitted path. |

To see the truck's current task state, run:

```bash
ros2 topic echo /truck1/status --once
```

Typical states include `idle`, `waiting`, `navigating`, `performing_action`, `completed`, and `fault`. If it reports `fault`, read the accompanying detail and ask the instructor before retrying.

---

# Before You Leave

- Wait for the task to finish and confirm the truck is stationary.
- Confirm the dump bed is lowered.
- In the **Dump Truck Pi** window, stop the terminal running `dump_truck_pi.launch.py` with **Ctrl+C** after the truck has stopped.
- Close `rqt_graph` and your task terminal.
- Ask the instructor before stopping the shared Command Center or Zenoh router.
- Follow the instructor's directions for shutting down the truck.
- Save your YAML file and close your remote connections.

---

# Lab Complete

You used ROS 2 and a YAML waypoint file to move a physical dump truck forward, reverse it, and raise/lower its dump bed (the loading container).

<!-- INSTRUCTOR NOTE — Remove before student distribution.
Prepared and cross-checked with the supplied README (7).md on 2026-10-06 to match the structure and simplicity of the supplied Labs 01 and 02.
Technical reference: https://github.com/CICPSU/CIC-ConRobotics-2026
main commit reviewed: b28f454b0cdb00ad82d85466699662fb66464208.
Checked command_center/README.md, command_center.launch.py, dump_truck_pi.launch.py,
dump_truck_ros_pc.launch.py, ExecuteRobotTask.action, waypoint_action_server_node.py,
tag_odom_fusion_node.py, bucket_action_node.py, and network/setup_zenoh.sh.
The waypoint server reads the YAML on its own host; verify access if students and
the shared Command Center use different accounts or PCs.
Revised 2026-10-08 using the line comments and README (8).md (Lab 2).
The X-axis example uses A=(0,0), B=(1,0). Confirm both points are usable in the arena.
The server expands ~ under its own account; use the same account for the task and Command Center.
Before distribution: insert the four photos and example rqt_graph screenshot;
confirm A/B coordinates, starting heading, physical stop procedure, calibrated bed
positions, and instructor-approved speed settings; run all three exercises on hardware.
Commands and YAML were checked against source; physical operation was not tested here.
-->
