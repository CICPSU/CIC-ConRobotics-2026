# Excavator 3 — ROS 2 Task Execution Guide

This guide explains how to operate **Excavator 3** using ROS 2 trajectory files.

Excavator 3 is controlled through the namespaced ROS 2 `FollowJointTrajectory` Action:

```text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

Tasks are defined as YAML trajectory files and sent from the ROS PC to the Raspberry Pi on Excavator 3 through Zenoh.

---

# Current Validation Status

| Joint | Status | Notes |
|---|---|---|
| Boom | ✅ Validated | Closed-loop position control |
| Arm | ✅ Validated | Closed-loop position control; final tuning may continue |
| Bucket | ✅ Validated | Closed-loop position control with ADC filtering; final tuning may continue |
| Swing | ⚠️ Not validated | **Do not command** |

> **Important:** Until Swing has been validated separately, do not include `swing` in physical Excavator 3 trajectory files.

---

# 1. Start Excavator 3

## Raspberry Pi — Excavator03

Move to the repository:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
```

Update the repository if required:

```bash
git pull origin dev
```

Build the excavator package:

```bash
source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select excavator_control

source install/setup.bash
```

Configure the Raspberry Pi as the Excavator 3 Zenoh client:

```bash
source network/setup_zenoh.sh client excavator3
```

If `pigpiod` is not already running:

```bash
sudo pigpiod
```

Start the Excavator 3 trajectory server:

```bash
ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi \
  robot_name:=excavator3 \
  config:=$(ros2 pkg prefix excavator_control)/share/excavator_control/config/excavator3.yaml
```

The server should create:

```text
/excavator3/excavator_trajectory_server
/excavator3/upper_arm_controller/follow_joint_trajectory
/excavator3/joint_states
```

Leave this terminal running.

The current configuration uses:

```text
auto_home_on_startup=false
```

so starting the server should not intentionally command the excavator to a home position.

---

# 2. Set Up the ROS PC

The Zenoh router should already be running on the ROS PC.

Open a ROS PC application terminal:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc
```

Check that Excavator 3 is visible:

```bash
ros2 node list | grep excavator3
```

Expected:

```text
/excavator3/excavator_trajectory_server
```

Check the Action:

```bash
ros2 action list | grep excavator3
```

Expected:

```text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

Check joint feedback:

```bash
ros2 topic echo /excavator3/joint_states --once
```

The message should contain:

```text
swing_joint
boom_joint
arm_joint
bucket_joint
```

Joint positions reported through `/joint_states` are in **radians**.

---

# 3. Trajectory File Format

Excavator tasks are stored under:

```text
operations/excavator/trajectories/
```

A basic Excavator 3 trajectory looks like:

```yaml
trajectory_name: example_task

description: >
  Example Excavator 3 task.

joints:
  - boom
  - arm
  - bucket

waypoints:
  - name: position_1
    positions:
      boom: -40.0
      arm: 100.0
      bucket: 15.0

  - name: position_2
    positions:
      boom: -35.0
      arm: 95.0
      bucket: 25.0
```

Trajectory positions are specified in **degrees**.

The trajectory client converts these values to radians before creating the ROS 2 `FollowJointTrajectory` goal.

Trajectories may contain all supported joints or only a subset.

Joints not listed in the YAML are not included in the trajectory request.

---

# 4. Validate a Trajectory Before Running It

Before sending a new trajectory to Excavator 3, validate it against the Excavator 3 configuration.

```bash
ros2 run excavator_control \
  validate_excavator_trajectory \
  robots/excavator/excavator_control/config/excavator3.yaml \
  operations/excavator/trajectories/<TRAJECTORY_FILE>.yaml
```

A valid trajectory should report:

```text
TRAJECTORY IS VALID
```

The validator checks software-level requirements such as:

- supported joints
- duplicate joints
- waypoint structure
- missing positions
- invalid values
- configured joint limits

> Validation does not guarantee that a trajectory is physically safe. Always inspect the requested positions and the current physical excavator configuration before execution.

---

# 5. Run a Task

Send a trajectory to Excavator 3 with:

```bash
ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/<TRAJECTORY_FILE>.yaml \
  --robot excavator3 \
  --seconds-per-waypoint 5.0
```

The `--robot excavator3` argument targets:

```text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

`--seconds-per-waypoint` controls the requested duration between waypoints.

For initial physical testing, use conservative motions and sufficient time between waypoints.

---

# 6. Example — Boom Only

```yaml
trajectory_name: boom_test

description: >
  Small boom movement for testing.

joints:
  - boom

waypoints:
  - name: boom_position_1
    positions:
      boom: -40.0

  - name: boom_position_2
    positions:
      boom: -45.0
```

Run:

```bash
ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/boom_test.yaml \
  --robot excavator3 \
  --seconds-per-waypoint 5.0
```

---

# 7. Example — Arm Only

```yaml
trajectory_name: arm_test

description: >
  Small arm movement for testing.

joints:
  - arm

waypoints:
  - name: arm_position_1
    positions:
      arm: 100.0

  - name: arm_position_2
    positions:
      arm: 95.0
```

Run:

```bash
ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/arm_test.yaml \
  --robot excavator3 \
  --seconds-per-waypoint 5.0
```

---

# 8. Example — Bucket Only

```yaml
trajectory_name: bucket_test

description: >
  Small bucket movement for testing.

joints:
  - bucket

waypoints:
  - name: bucket_position_1
    positions:
      bucket: 15.0

  - name: bucket_position_2
    positions:
      bucket: 10.0
```

Run:

```bash
ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/bucket_test.yaml \
  --robot excavator3 \
  --seconds-per-waypoint 5.0
```

---

# 9. Example — Boom + Arm + Bucket

Multiple joints can be included in the same task.

```yaml
trajectory_name: three_joint_test

description: >
  Coordinated task using boom, arm, and bucket.
  Swing is intentionally excluded.

joints:
  - boom
  - arm
  - bucket

waypoints:
  - name: pose_1
    positions:
      boom: -40.0
      arm: 100.0
      bucket: 15.0

  - name: pose_2
    positions:
      boom: -35.0
      arm: 95.0
      bucket: 25.0

  - name: pose_3
    positions:
      boom: -40.0
      arm: 100.0
      bucket: 15.0
```

Run:

```bash
ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/three_joint_test.yaml \
  --robot excavator3 \
  --seconds-per-waypoint 5.0
```

---

# 10. Creating a Multi-Step Excavation Task

More complex tasks can be created by adding waypoints.

For example:

```yaml
trajectory_name: excavation_demo

description: >
  Example multi-step excavator motion using boom, arm, and bucket.
  Swing is intentionally excluded.

joints:
  - boom
  - arm
  - bucket

waypoints:
  - name: ready
    positions:
      boom: -40.0
      arm: 100.0
      bucket: 15.0

  - name: approach
    positions:
      boom: -45.0
      arm: 105.0
      bucket: 20.0

  - name: bucket_motion
    positions:
      boom: -45.0
      arm: 105.0
      bucket: 35.0

  - name: lift
    positions:
      boom: -35.0
      arm: 100.0
      bucket: 35.0

  - name: return
    positions:
      boom: -40.0
      arm: 100.0
      bucket: 15.0
```

These values are examples only.

Verify every requested position against the current physical configuration before running a new trajectory.

---

# 11. Check Current Joint Positions

Check the current Excavator 3 joint feedback with:

```bash
ros2 topic echo /excavator3/joint_states --once
```

The joint order is:

```text
swing_joint
boom_joint
arm_joint
bucket_joint
```

The reported positions are in **radians**.

For conversion:

```text
degrees = radians × 180 / π
```

When developing a new task, compare the current physical position with the requested trajectory and begin with small changes.

---

# 12. Recommended Workflow for a New Task

For a new physical Excavator 3 trajectory:

1. Start the Excavator 3 trajectory server.
2. Confirm `/excavator3/joint_states`.
3. Inspect the current physical position.
4. Create or edit the trajectory YAML.
5. Confirm that `swing` is not included.
6. Validate the trajectory against `excavator3.yaml`.
7. Begin with small joint changes.
8. Run the trajectory with conservative waypoint timing.
9. Verify the expected direction and behavior of each joint.
10. Increase motion range or complexity only after the initial motion is confirmed.

Do not begin with a large coordinated motion when testing a new trajectory.

---

# 13. Safety

Always keep the excavator workspace clear before sending a physical trajectory.

Immediately stop testing if:

- a joint moves in the wrong direction
- an unexpected joint moves
- the excavator begins oscillating
- a joint approaches a mechanical limit
- abnormal motor or mechanical noise occurs
- motion does not stop as expected

Use:

```text
Ctrl+C
```

to interrupt the running process when necessary.

For the current Excavator 3 configuration:

> **Do not command Swing until Swing has been separately validated.**

---

# 14. Useful ROS 2 Commands

Check the Excavator 3 node:

```bash
ros2 node list | grep excavator3
```

Check Excavator 3 topics:

```bash
ros2 topic list | grep excavator3
```

Check joint feedback:

```bash
ros2 topic echo /excavator3/joint_states --once
```

Check Excavator 3 Actions:

```bash
ros2 action list | grep excavator3
```

Expected:

```text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

Inspect the Action:

```bash
ros2 action info \
  /excavator3/upper_arm_controller/follow_joint_trajectory
```

With the physical server running, the Action should report:

```text
Action servers: 1
```

---

# 15. Current Excavator 3 Joint Configuration

| Joint | ADC Channel | Physical Validation |
|---|---:|---|
| Boom | A1 | ✅ Closed-loop tested |
| Arm | A2 | ✅ Closed-loop tested |
| Bucket | A0 | ✅ Closed-loop tested with ADC filtering |
| Swing | A3 | ⚠️ Not validated |

Current physical ADC mapping:

```text
A0 → Bucket
A1 → Boom
A2 → Arm
A3 → Swing
```

> The Swing channel is listed for configuration reference only. Swing motion has not yet been physically validated.

---

# Quick Start

## Excavator03 Raspberry Pi

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

## ROS PC

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc
```

Check communication:

```bash
ros2 action info \
  /excavator3/upper_arm_controller/follow_joint_trajectory

ros2 topic echo \
  /excavator3/joint_states \
  --once
```

Validate a trajectory:

```bash
ros2 run excavator_control \
  validate_excavator_trajectory \
  robots/excavator/excavator_control/config/excavator3.yaml \
  operations/excavator/trajectories/<TRAJECTORY_FILE>.yaml
```

Run it:

```bash
ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/<TRAJECTORY_FILE>.yaml \
  --robot excavator3 \
  --seconds-per-waypoint 5.0
```

**Validated operating joints: Boom + Arm + Bucket**

**Do not use Swing until validation is complete.**