# Excavator 3 — ROS 2 Task Execution Guide

This guide explains how to operate **Excavator 3** using ROS 2 trajectory files.

The excavator is controlled through a ROS 2 `FollowJointTrajectory` action server.  
Tasks are defined as YAML trajectory files and sent from the ROS PC to the Raspberry Pi on the excavator through Zenoh.

## Current Validation Status

| Joint | Status | Notes |
|---|---|---|
| Boom | ✅ Validated | Closed-loop position control |
| Arm | ✅ Validated | Closed-loop position control |
| Bucket | ✅ Validated | Closed-loop position control with ADC filtering |
| Swing | ⚠️ Not validated | **Do not command yet** |

> **Important:** Until Swing has been validated separately, do not include `swing` in trajectory files.

---

# 1. Start Excavator 3

## Raspberry Pi — Excavator03

SSH into Excavator03 and move to the repository:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
```

Update the repository:

```bash
git pull origin dev
```

Set up ROS 2 and build the excavator package:

```bash
source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select excavator_control

source install/setup.bash
```

Configure Zenoh:

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
  config:=$(ros2 pkg prefix excavator_control)/share/excavator_control/config/excavator3.yaml
```

Confirm that the terminal reports something similar to:

```text
Detected mode: PI
Config: .../excavator3.yaml
[PI MODE] PiExcavatorTrajectoryServer ready
Action: /upper_arm_controller/follow_joint_trajectory
Publishes feedback: /joint_states
auto_home_on_startup=false
```

Leave this terminal running.

---

# 2. Set Up the ROS PC

Open another terminal on the ROS PC:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
```

Check that joint feedback is available:

```bash
ros2 topic echo /joint_states
```

You should see:

```text
name:
- swing_joint
- boom_joint
- arm_joint
- bucket_joint
position:
- ...
- ...
- ...
- ...
```

Press `Ctrl+C` after confirming that feedback is being received.

---

# 3. Trajectory File Format

Excavator tasks are stored as YAML files under:

```text
operations/excavator/trajectories/
```

A basic trajectory looks like this:

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

Joint positions are specified in **degrees**.

The trajectory client converts these values and sends the corresponding ROS 2 trajectory to the excavator.

---

# 4. Run a Task

For example:

```bash
ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/example_task.yaml \
  --seconds-per-waypoint 5.0
```

`--seconds-per-waypoint` controls the requested duration between waypoints.

For initial testing, use relatively slow movements such as:

```bash
--seconds-per-waypoint 5.0
```

---

# 5. Example — Boom Only

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
  --seconds-per-waypoint 5.0
```

---

# 6. Example — Arm Only

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

---

# 7. Example — Bucket Only

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

---

# 8. Example — Boom + Arm + Bucket

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
  --seconds-per-waypoint 5.0
```

---

# 9. Creating a Multi-Step Excavation Task

More complex tasks can be created by adding additional waypoints.

For example:

```yaml
trajectory_name: excavation_demo

description: >
  Example multi-step excavator motion using boom, arm, and bucket.

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

These values are only an example structure. Verify that every requested position is appropriate for the physical configuration before running a new task.

---

# 10. Check Current Joint Positions

Before creating a new trajectory, check the current excavator configuration:

```bash
ros2 topic echo /joint_states
```

The joint order is:

```text
swing_joint
boom_joint
arm_joint
bucket_joint
```

The reported positions are in **radians**.

For quick conversion:

```text
degrees = radians × 180 / π
```

When developing a new task, start from the current physical configuration and use small changes first.

---

# 11. Recommended Workflow for a New Task

When testing a new task:

1. Start the Excavator 3 ROS server.
2. Confirm `/joint_states`.
3. Check the current physical position of the excavator.
4. Create a new YAML trajectory.
5. Start with small joint changes (approximately 5°).
6. Run the trajectory slowly.
7. Verify that each joint moves in the expected direction.
8. Increase the motion range gradually.
9. Add additional waypoints only after the individual motions are confirmed.

Do not start with a large multi-joint motion on an untested trajectory.

---

# 12. Safety

Always keep the excavator workspace clear before sending a trajectory.

Immediately stop execution if:

- a joint moves in the wrong direction;
- an unexpected joint moves;
- the excavator begins oscillating;
- a joint approaches a mechanical limit;
- abnormal motor or mechanical noise occurs;
- the commanded motion does not stop as expected.

Use:

```text
Ctrl+C
```

to stop the running trajectory/client when necessary.

For the current Excavator 3 configuration:

> **Do not command Swing until Swing has been separately validated.**

---

# 13. Useful ROS 2 Commands

Check available topics:

```bash
ros2 topic list
```

Monitor joint positions:

```bash
ros2 topic echo /joint_states
```

Check the trajectory action:

```bash
ros2 action list
```

Expected action:

```text
/upper_arm_controller/follow_joint_trajectory
```

Inspect it:

```bash
ros2 action info /upper_arm_controller/follow_joint_trajectory
```

---

# Quick Start

### Excavator03

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client excavator3

ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi \
  config:=$(ros2 pkg prefix excavator_control)/share/excavator_control/config/excavator3.yaml
```

### ROS PC

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
```

Check feedback:

```bash
ros2 topic echo /joint_states
```

Run a trajectory:

```bash
ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/<TRAJECTORY_FILE>.yaml \
  --seconds-per-waypoint 5.0
```

---

## Current Excavator 3 Joint Configuration

| Joint | ADC Channel | Validated |
|---|---:|---:|
| Boom | A1 | ✅ |
| Arm | A2 | ✅ |
| Bucket | A0 | ✅ |
| Swing | A3 | ⚠️ Not yet |

**Validated operating joints: Boom + Arm + Bucket**

**Do not use Swing until validation is complete.**