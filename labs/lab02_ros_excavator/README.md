# Lab 02 — ROS 2 Excavator Control

In Lab 01, you connected to the ROS PC and Raspberry Pi using VS Code Remote SSH, built the course workspace, and used Zenoh to send `Hello World` messages from a talker to a listener on different computers. You also inspected nodes and topics with the ROS 2 command line.

In this lab, you will control a **physical model excavator**. You will first learn its joints with a handheld controller, then create a YAML trajectory that moves one joint, two joints, and finally all four. You will use `rqt_graph` to see how your commands reach the robot.

**Estimated time: 30 minutes. Work with your assigned group.**

By the end of this lab, you will be able to:

- identify the swing, boom, arm, and bucket joints
- create a ROS 2 excavator trajectory in YAML
- send that trajectory to the robot's Action server
- identify the trajectory client, controller, and feedback in the ROS graph
- record a short dig-and-dump sequence

> **Instructor preparation:** Before class, fill in the station card below; start or verify the shared Zenoh router, the robot Pi, and the Command Center with the excavator's overhead camera and swing adapter. Wait for the excavator to report READY. Confirm the ROS PC desktop can run `rqt_graph`. Students should not spend this 30-minute lab installing packages or calibrating robots.

| Your station | Instructor fills in |
| --- | --- |
| Robot name (`excavator1`, `excavator3`, etc.) | __________ |
| Approved starting pose and target ranges, in degrees | __________ |
| ROS PC and excavator Pi | __________ |
| Stop/power cutoff procedure | __________ |

---

# Part A — Explore the Physical Excavator

## Step 1 — Identify the Four Joints (4 minutes)

With the handheld controller, move **one joint at a time**. Keep each movement small. Watch what changes at the bucket tip.

| Joint | What moved? |
| --- | --- |
| Swing | |
| Boom | |
| Arm | |
| Bucket | |

Which joint points the excavator toward a truck? Which joints lower, reach, and scoop?

<!-- INSERT IMAGE: images/step01_excavator_joints.png
Photograph of the actual course model with arrows labeling swing, boom, arm, and bucket.
Place here after capturing; same ~900 px display width as Lab 01. -->

### Checkpoint — Joint Identification

- [ ] We can identify all four joints on the physical robot.
- [ ] We know which joint turns the upper structure toward the dump location.

Return the excavator to the instructor-approved starting pose before switching from the handheld controller to the ROS trajectory. Confirm the handheld controller is no longer issuing motion commands.

---

# Part B — Create and Run a Trajectory

## Step 2 — Prepare a ROS PC Terminal (2 minutes)

Open a terminal on the **ROS PC** as in Lab 01. Make sure your terminal prompt is from the ROS PC, not the Raspberry Pi.

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
```

Set the robot assigned to your group (replace the example name):

```bash
ROBOT_NAME=excavator3
```

Check the middleware and the robot's Action server:

```bash
echo "$RMW_IMPLEMENTATION"
ros2 action list
```

Expected middleware: `rmw_zenoh_cpp`. Look for an Action named like:

```text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

If the Action is missing, ask the instructor to check the robot. Do not start a second controller on the same excavator.
The excavator must also finish startup and report `READY` before receiving a trajectory. The instructor will check the robot Pi.

<!-- INSERT IMAGE: images/step02_action_list.png
ROS PC terminal showing rmw_zenoh_cpp and the assigned excavator Action.
Crop to emphasize terminal hostname and Action name. -->

## Step 3 — Create a One-Joint YAML File (3 minutes)

In the ROS PC's VS Code window, create a file in your own group's working area. For example:

```bash
mkdir -p ~/ws_conrobotics/lab02
TRAJECTORY_FILE=~/ws_conrobotics/lab02/my_excavator_trajectory.yaml
code "$TRAJECTORY_FILE"
```

Copy this structure into the file. Replace the two bucket angles with the **instructor-approved starting and target angles** for your assigned robot. Make a small change.

```yaml
trajectory_name: my_excavator_lab
description: One-joint bucket test

joints:
  - bucket

waypoints:
  - name: start
    positions:
      bucket: 0.0     # REPLACE with approved starting angle

  - name: move_bucket
    positions:
      bucket: 35.0    # REPLACE with approved target angle
```

For Excavator 3, the configured startup target is `swing=90, boom=-10, arm=60, bucket=0` degrees; the example bucket target of 35 degrees is a visible but limited motion from that pose. These are **not universal angles**. The instructor must verify the actual pose. All targets are in degrees. Save the file before running it.

> The `joints` list controls which joints the trajectory commands. Each waypoint must contain a position for **every joint listed**, with no extra joint positions. You are writing a motion trajectory, not editing the robot's calibration or GPIO configuration.

<!-- INSERT IMAGE: images/step03_bucket_yaml.png
VS Code editor with the one-joint YAML, highlighting joints and positions. -->

## Step 4 — Send the One-Joint Trajectory (2 minutes)

In the same configured **ROS PC** terminal, run:

```bash
ros2 run construction_site_control \
  excavator_task_client \
  "$TRAJECTORY_FILE" \
  --robot "$ROBOT_NAME" \
  --seconds-per-waypoint 5.0
```

The task client reads the YAML when you run it, so editing this trajectory file does **not** require `colcon build`. Watch the excavator and the terminal for goal acceptance, feedback, and a successful result.

If the motion is unexpected, use the station's stop procedure and tell the instructor. If a goal is rejected, check the robot name, Action name, angles, and YAML indentation before retrying.

### Checkpoint — One Joint

- [ ] The file saved correctly.
- [ ] The goal was accepted.
- [ ] Only the bucket was commanded.
- [ ] We observed feedback or a result.

---

# Part C — Inspect the ROS Graph

## Step 5 — Open `rqt_graph` (2 minutes)

Use a **graphical terminal on the ROS PC desktop** with the same ROS/Zenoh setup as Step 2. Run:

```bash
rqt_graph
```

If the command is unavailable or the graphical window does not open, ask the instructor. Your VS Code Remote SSH terminal may not display a GUI without graphical forwarding.

While a trajectory is running, find the `excavator3_task_client` (for Excavator 3) and excavator controller. In the graph options, enable Action-related connections if they are hidden. Some nodes may disappear after the short client program exits; run the trajectory again while observing the graph.

Take a screenshot named `graph_one_joint.png`. The graph may also show other course nodes, such as the camera or AprilTag system.

<!-- INSERT IMAGE: images/step05_rqt_one_joint.png
Actual rqt_graph showing trajectory client, controller/Action connections, and relevant feedback.
Annotate the two nodes and the command path without obscuring graph labels. -->

### Checkpoint — First Graph

- [ ] We can locate the trajectory client while it is running.
- [ ] We can identify the excavator controller/Action connection.
- [ ] We saved `graph_one_joint.png`.

---

# Part D — Add More Joints

## Step 6 — Command Two Joints (5 minutes)

Edit the **same** YAML file:

1. Add `arm` under `joints`.
2. Add an `arm` position to **every** waypoint.
3. Add one or two short waypoints so the **arm and bucket** produce a scooping movement. Use only approved angles.
4. Save the file and repeat the command from Step 4.

The required shape is:

```yaml
joints:
  - arm
  - bucket

waypoints:
  - name: start
    positions:
      arm: APPROVED_START_ARM
      bucket: APPROVED_START_BUCKET

  - name: scoop
    positions:
      arm: APPROVED_TARGET_ARM
      bucket: APPROVED_TARGET_BUCKET
```

Replace **every** uppercase placeholder with a numeric angle before running. A joint missing from any waypoint will make the YAML invalid.

For Excavator 3, a small example starts at `arm: 60.0, bucket: 0.0` and moves to `arm: 80.0, bucket: 35.0`. The physical response depends on the robot. Get the instructor's go-ahead before running it.

### Checkpoint — Two Joints

- [ ] Both joints are listed under `joints`.
- [ ] Every waypoint has positions for both joints.
- [ ] The robot performed the intended movement.

## Step 7 — Build a Four-Joint Cycle (7 minutes)

Add `swing` and `boom` to `joints`, and give **all four joints** a position at every waypoint. Use a few short waypoints to approximate:

```text
Open/approach → lower → curl arm and bucket → lift → swing → dump → return
```

Follow the approved ranges on your station card. Use `excavator3_lab02_example.yaml` as the **Excavator 3 classroom reference**. It begins at the configured initial pose and moves only to `boom=-25`, `arm=80`, `bucket=35`, and `swing=65` before returning. These example targets have not been physically tested for this lab. Confirm the READY pose, direction, and clearance with the instructor. Do not use its angles for another excavator.

Run your file using the Step 4 command. The movement can be sequential; this exercise does not require all four motors to run at the same instant.

<!-- INSERT IMAGE: images/step07_four_joint_yaml.png
VS Code screenshot of the student trajectory showing the four names under joints
and one complete waypoint with four positions. -->

### Checkpoint — Four Joints

- [ ] The YAML lists swing, boom, arm, and bucket.
- [ ] Every waypoint specifies the four corresponding positions.
- [ ] The motion resembles a short excavation cycle.

## Step 8 — Compare the Graph (2 minutes)

Open or refresh `rqt_graph` and rerun the trajectory if needed to make the client visible. Save `graph_four_joints.png`.

Compare the two screenshots: Did changing the YAML from one joint to four add ROS nodes, or did the **same client and controller** exchange a different trajectory? Identify the connection that carries the command and any feedback you can observe.

<!-- INSERT IMAGE: images/step08_rqt_four_joints.png
Second real rqt_graph screenshot, using the same view settings as Step 5 for comparison. -->

---

# Part E — Record and Submit

## Step 9 — Film the Excavation Motion (3 minutes)

Record a short video showing your group's model excavator performing a recognizable **scoop → lift → turn → dump** motion. Keep people outside the robot's movement area.

Submit **one set per group**:

- the final `my_excavator_trajectory.yaml`
- `graph_one_joint.png` and `graph_four_joints.png`
- the short video
- one or two sentences: Which joints produced the scooping motion, and which joint turned toward the dump location?

### Before You Leave

Stop your own trajectory program and close `rqt_graph`. Follow the instructor's directions for robot power. **Do not stop the shared Zenoh router** while other groups are working.

---

# Lab Complete

You have moved from Lab 01's ROS 2 talker/listener to an Action client controlling a real robot. Your trajectory describes the excavator's joint targets; later labs will coordinate the excavator with a dump truck and run an integrated scenario.

<!-- INSTRUCTOR IMAGE CHECKLIST
Capture actual screenshots before release, then replace the comments above with:
<img src="images/step01_excavator_joints.png" width="900">
<img src="images/step02_action_list.png" width="900">
<img src="images/step03_bucket_yaml.png" width="900">
<img src="images/step05_rqt_one_joint.png" width="900">
<img src="images/step07_four_joint_yaml.png" width="900">
<img src="images/step08_rqt_four_joints.png" width="900">
Use real course equipment, terminal output, and graph state. Do not publish broken image links.
-->
---

# Instructor Setup — Complete Before the 30-Minute Student Activity

The repository root README directs normal physical operation to `command_center/README.md`. The commands below follow that guide for **Excavator 3 alone**. Use the machine-specific configuration already installed on the Pi. Start the shared router only once; if it is running, reuse it.

### ROS PC — Router terminal

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh router ros-pc
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Excavator 3 Pi — Hardware terminal

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client excavator3
sudo pigpiod
ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi \
  robot_name:=excavator3
```

### ROS PC — Command Center terminal

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:="" \
  excavators:=excavator3 \
  start_scenario_manager:=false
```

The Command Center provides the shared camera, AprilTag detection, and swing adapter. Wait for fresh swing feedback and the Pi's final `READY` confirmation. Do not have students send Action goals while initialization is in progress. Confirm the RC controller and ROS controller will not issue conflicting commands during the exercise.

Before release, copy the supplied `excavator3_lab02_example.yaml` to the lab folder or `operations/excavator/trajectories/` on the course ROS PC, check the machine's actual starting pose and physical clearance, and run the sample under instructor supervision. It has been checked against the repository's YAML structure and the configured joint ranges, **not tested on the physical machine**.