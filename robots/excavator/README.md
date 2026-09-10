# Excavator

ROS 2 software for the CIC ConRobotics physical model excavator.

The excavator software supports both:

- **SIM mode** for software development and testing without physical hardware
- **PI mode** for controlling the physical excavator through a Raspberry Pi

The current interface is based on the standard ROS 2 `FollowJointTrajectory` Action.

---

## 1. System Architecture

The excavator currently uses the following software flow:

```text
Trajectory YAML
      |
      v
excavator_trajectory_client
      |
      | FollowJointTrajectory Action
      v
excavator_trajectory_server
      |
      +----------------------+
      |                      |
      v                      v
   SIM mode                PI mode
      |                      |
      v                      v
/joint_command        GPIO / PWM / ADC
                           |
                           v
                  Physical Excavator
```

The Action interface is:

```text
/upper_arm_controller/follow_joint_trajectory
```

The four supported excavator joints are:

```text
swing
boom
arm
bucket
```

Trajectories may command all four joints or only a subset of the joints.

---

## 2. Repository Structure

```text
robots/excavator/
└── excavator_control/
    ├── excavator_control/
    │   ├── config_loader.py
    │   ├── trajectory_loader.py
    │   ├── validate_config.py
    │   ├── validate_trajectory.py
    │   ├── goal_validation.py
    │   ├── excavator_trajectory_server.py
    │   └── excavator_trajectory_client.py
    │
    ├── config/
    │   ├── excavator_template.yaml
    │   └── excavator1.yaml
    │
    ├── launch/
    │   └── excavator.launch.py
    │
    ├── test/
    │
    ├── package.xml
    ├── setup.cfg
    └── setup.py
```

Operational trajectory files are stored separately from the ROS package:

```text
operations/
└── excavator/
    └── trajectories/
```

This separates:

- machine-specific configuration and calibration
- reusable operational trajectories

---

## 3. Build the Package

From the repository root:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select excavator_control

source install/setup.bash
```

Available executables can be checked with:

```bash
ros2 pkg executables excavator_control
```

Expected executables include:

```text
excavator_control excavator_trajectory_client
excavator_control excavator_trajectory_server
excavator_control validate_excavator_config
excavator_control validate_excavator_trajectory
```

---

## 4. Validate the Excavator Configuration

Before using a machine configuration, validate it.

```bash
ros2 run excavator_control \
  validate_excavator_config \
  robots/excavator/excavator_control/config/excavator1.yaml
```

A valid configuration should report:

```text
CONFIGURATION IS VALID
```

Configuration validation checks the structure and consistency of the configuration file.

It does **not** guarantee that physical calibration values are correct.

---

## 5. Validate a Trajectory

Trajectory files should be validated against the machine configuration before execution.

Example:

```bash
ros2 run excavator_control \
  validate_excavator_trajectory \
  robots/excavator/excavator_control/config/excavator1.yaml \
  operations/excavator/trajectories/excavator_trajectory_template.yaml
```

The validator checks:

- supported joint names
- duplicate joints
- waypoint structure
- missing joint positions
- unexpected joint positions
- numeric and finite values
- configured joint limits

A valid trajectory reports:

```text
TRAJECTORY IS VALID
```

---

## 6. Trajectory YAML Format

A trajectory defines the joints that should be controlled and a sequence of target positions.

Example:

```yaml
trajectory_name: example_trajectory

description: >
  Example excavator trajectory.

joints:
  - swing
  - boom
  - arm
  - bucket

waypoints:
  - name: position_1
    positions:
      swing: 0.0
      boom: -20.0
      arm: 90.0
      bucket: 20.0

  - name: position_2
    positions:
      swing: 20.0
      boom: -30.0
      arm: 100.0
      bucket: 40.0
```

All trajectory positions are specified in **degrees**.

The trajectory client converts them to radians before sending the ROS 2 Action goal.

---

## 7. Subset-Joint Trajectories

A trajectory does not need to command all four joints.

For example, a boom-only trajectory can be written as:

```yaml
trajectory_name: boom_only_example

description: >
  Example trajectory that commands only the boom.

joints:
  - boom

waypoints:
  - name: position_1
    positions:
      boom: -20.0

  - name: position_2
    positions:
      boom: -25.0
```

The resulting ROS 2 trajectory contains only:

```text
boom_joint
```

This is particularly useful for:

- isolated joint testing
- calibration work
- safer hardware debugging
- operations that do not require all excavator joints

Joints not listed in the trajectory are not included in the trajectory command.

---

# Simulation

## 8. Start the Excavator in SIM Mode

SIM mode does not access Raspberry Pi hardware.

Start the server with:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch excavator_control \
  excavator.launch.py \
  mode:=sim
```

The server should report:

```text
Detected mode: SIM
[SIM MODE] ExcavatorTrajectoryServer ready
Action  : /upper_arm_controller/follow_joint_trajectory
Publishes: /joint_command
```

SIM mode publishes commanded joint states to:

```text
/joint_command
```

---

## 9. Send a Trajectory in SIM Mode

Keep the server running.

In another terminal:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/excavator_trajectory_template.yaml \
  --seconds-per-waypoint 3.0
```

The client should report that the Action goal was accepted and eventually completed successfully.

---

## 10. Observe SIM Commands

In another terminal:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /joint_command
```

The topic is published while a trajectory is executing.

If no trajectory is running, `ros2 topic echo` may wait without displaying a message. This is normal.

---

# Physical Excavator

## 11. Network Setup

The excavator Raspberry Pi and ROS computer must first be configured for the project ROS network.

### Excavator Raspberry Pi

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_network.sh \
  excavator_01 \
  ros_laptop_backup
```

### ROS Computer

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_network.sh \
  ros_laptop_backup \
  excavator_01
```

The network setup currently uses:

```text
ROS_DOMAIN_ID=10
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

The required static peers are configured by `network/setup_network.sh`.

---

## 12. Current RMW Configuration

During current excavator testing, ROS 2 Action communication has been verified with **CycloneDDS on both machines**.

Before running the excavator server:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The same RMW implementation must be selected on the ROS computer:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Then run the normal network setup script.

### Important

CycloneDDS is currently a **verified excavator configuration**, not yet a project-wide RMW standard.

The final RMW configuration for the complete multi-robot system is still under evaluation.

---

## 13. Start the Physical Excavator

> **WARNING:** PI mode can command physical motors.

The launch file defaults to:

```text
auto_home_on_startup=false
```

Therefore, simply starting the server should not intentionally move the excavator to its home position.

On the Raspberry Pi:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source network/setup_network.sh \
  excavator_01 \
  ros_laptop_backup

sudo pigpiod

ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi
```

Do not enable automatic homing unless the physical system has been checked and the intended motion is understood.

---

## 14. Check Communication

From the ROS computer:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source network/setup_network.sh \
  ros_laptop_backup \
  excavator_01
```

Check the Action server:

```bash
ros2 action list
```

Expected:

```text
/upper_arm_controller/follow_joint_trajectory
```

Check excavator joint feedback:

```bash
ros2 topic echo /joint_states --once
```

---

## 15. Safe Action Communication Check

The following command intentionally uses an invalid joint name.

It is useful for checking that the ROS 2 Action payload reaches the excavator without commanding a real excavator joint.

```bash
ros2 action send_goal \
  /upper_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [fake_joint], points: [{positions: [0.0], time_from_start: {sec: 1, nanosec: 0}}]}}"
```

The excavator server should reject the goal with a message similar to:

```text
Unknown joints: ['fake_joint']
```

This rejection is expected.

---

## 16. Send an Operational Trajectory

Before sending any trajectory to physical hardware:

1. Inspect the YAML file.
2. Confirm the intended joints.
3. Validate the trajectory against the machine configuration.
4. Confirm that the physical workspace is clear.
5. Confirm that the excavator calibration is appropriate for the test.

Validate first:

```bash
ros2 run excavator_control \
  validate_excavator_trajectory \
  robots/excavator/excavator_control/config/excavator1.yaml \
  operations/excavator/trajectories/YOUR_TRAJECTORY.yaml
```

Then send:

```bash
ros2 run excavator_control \
  excavator_trajectory_client \
  operations/excavator/trajectories/YOUR_TRAJECTORY.yaml \
  --seconds-per-waypoint 3.0
```

---

# Safety

## 17. Hardware Safety

The physical excavator is still undergoing calibration and hardware validation.

Do not assume that a trajectory is physically safe only because:

```text
TRAJECTORY IS VALID
```

Software validation confirms that the trajectory is structurally valid and within the configured software limits.

It does not verify:

- sensor accuracy
- physical calibration accuracy
- motor direction
- mechanical interference
- unexpected actuator behavior
- physical workspace clearance

For initial hardware testing, prefer **single-joint subset trajectories** rather than commanding all joints simultaneously.

---

# Current Development Status

## 18. Verified

The following functionality has been verified:

- ROS 2 Jazzy package builds successfully
- excavator configuration loading
- calibration conversion utilities
- trajectory YAML loading
- trajectory validation
- subset-joint trajectories
- Action goal validation
- SIM trajectory execution
- SIM `/joint_command` output
- launch-based SIM bringup
- ROS 2 Topic communication between the excavator Pi and ROS computer
- ROS 2 Action communication between the excavator Pi and ROS computer using CycloneDDS

Automated tests currently cover configuration loading, trajectory loading, trajectory validation, and Action goal validation.

---

## 19. Pending Hardware Validation

Physical closed-loop excavator control is **not yet considered fully validated**.

Known areas requiring additional hardware work include:

- boom sensor/calibration behavior
- boom closed-loop motion
- swing sensor/control behavior
- complete multi-joint physical trajectories
- final tuning of joint control parameters

Hardware validation should be resumed after the physical system and calibration are ready for controlled testing.

---

## 20. Communication Items for Future Evaluation

The current verified excavator communication configuration uses CycloneDDS.

The following items are intentionally deferred for later evaluation:

### Python Virtual Environment

Earlier excavator development used:

```text
~/excavator_env
```

Current testing has shown that the excavator server can run without activating this environment.

Future testing should determine whether the virtual environment changes behavior when:

- `RMW_IMPLEMENTATION` is not explicitly set
- Fast DDS is explicitly selected

The virtual environment should not be deleted until this evaluation is complete.

### Fast DDS

Explicit Fast DDS startup on the excavator Raspberry Pi has produced a Fast CDR/shared-library symbol error.

This currently requires further investigation before Fast DDS can be considered a supported excavator configuration.

### Zenoh

ROS 2 Zenoh / `rmw_zenoh` may be evaluated as an alternative communication layer for the multi-machine system.

Potential evaluation should include:

- excavator Action communication
- `/joint_states`
- camera and AprilTag nodes
- dump truck communication
- multi-machine discovery
- restart/reconnection behavior

No project-wide RMW decision has been made yet.

---

# Development Tests

Run the excavator unit tests with the system ROS Python environment:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

/usr/bin/python3 -m pytest -v \
  robots/excavator/excavator_control/test/
```

Then run through `colcon`:

```bash
colcon test \
  --packages-select excavator_control \
  --event-handlers console_direct+

colcon test-result --verbose
```

Use `/usr/bin/python3 -m pytest` rather than a user-installed `pytest` executable to avoid accidentally running tests with a Python environment associated with Isaac Sim or Isaac Lab.