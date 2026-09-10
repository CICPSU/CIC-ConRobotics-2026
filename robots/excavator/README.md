# Excavator

ROS 2 software for the CIC ConRobotics physical model excavator.

The excavator software supports both:

- **SIM mode** for software development and testing without physical hardware
- **PI mode** for controlling the physical excavator through a Raspberry Pi

The current interface is based on the standard ROS 2 `FollowJointTrajectory` Action.

Zenoh (`rmw_zenoh_cpp`) is the primary ROS 2 communication method for communication between the ROS PC and the excavator Raspberry Pi.

---

# 1. System Architecture

The excavator currently uses the following software flow:

```text
Trajectory YAML
      │
      ▼
excavator_trajectory_client
      │
      │ FollowJointTrajectory Action
      ▼
excavator_trajectory_server
      │
      ├──────────────────────┐
      │                      │
      ▼                      ▼
   SIM mode                PI mode
      │                      │
      ▼                      ▼
/joint_command        GPIO / PWM / ADC
                             │
                             ▼
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

For multi-machine operation, the excavator communicates through the same Zenoh router used by the rest of the construction robotics platform:

```text
                         ROS PC
                            │
                       Zenoh Router
                            │
              ┌─────────────┴─────────────┐
              │                           │
              ▼                           ▼
      ROS PC Applications          Excavator Pi
              │                           │
              │                           ▼
              │                 Excavator Trajectory
              │                       Server
              │                           │
              └──── ROS 2 Actions ────────┘
```

---

# 2. Repository Structure

```text
robots/excavator/
└── excavator_control/
    ├── excavator_control/
    │   ├── __init__.py
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
    ├── resource/
    │   └── excavator_control
    │
    ├── test/
    │   ├── __init__.py
    │   ├── test_config_loader.py
    │   ├── test_trajectory_loader.py
    │   ├── test_validate_trajectory.py
    │   └── test_goal_validation.py
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
        ├── excavator_trajectory_template.yaml
        └── boom_small_test.yaml
```

This separates:

```text
Machine configuration / calibration
        → robots/excavator/excavator_control/config/

Reusable operational trajectories
        → operations/excavator/trajectories/
```

---

# 3. Build the Package

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

# 4. Validate the Excavator Configuration

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

# 5. Validate a Trajectory

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

# 6. Trajectory YAML Format

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

# 7. Subset-Joint Trajectories

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

# 8. Start the Excavator in SIM Mode

SIM mode does not access Raspberry Pi hardware.

For local simulation testing, no multi-machine network setup is required.

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

# 9. Send a Trajectory in SIM Mode

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

# 10. Observe SIM Commands

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

# 11. Zenoh Network Setup

The excavator Raspberry Pi and ROS PC use Zenoh (`rmw_zenoh_cpp`) for ROS 2 communication.

The same network architecture is shared by the dump trucks and other construction robots.

```text
                         ROS PC
                            │
                       Zenoh Router
                            │
          ┌─────────────────┼─────────────────┐
          │                 │                 │
          ▼                 ▼                 ▼
      Dump Truck        Dump Truck        Excavator
          Pi                Pi                Pi
```

Only one Zenoh router is required.

The excavator does **not** require a separate router.

Network configuration is managed through:

```text
network/
├── devices.sh
├── setup_zenoh.sh
└── setup_network.sh
```

`setup_zenoh.sh` is the primary network setup.

`setup_network.sh` is retained as a DDS-based fallback.

---

# 12. Start the Zenoh Router

**Machine:** ROS PC  
**Terminal:** T1  
**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router

ros2 run rmw_zenoh_cpp rmw_zenohd
```

Only one router should normally be running for the construction robotics system.

If a Zenoh router is already running for the dump truck system, **do not start another router for the excavator**.

---

# 13. Start the Physical Excavator

> **WARNING:** PI mode can command physical motors.

The launch file defaults to:

```text
auto_home_on_startup=false
```

Therefore, simply starting the server should not intentionally move the excavator to its home position.

**Machine:** Excavator1 Raspberry Pi  
**Terminal:** T1  
**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client excavator1

sudo pigpiod

ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi
```

Do not enable automatic homing unless the physical system has been checked and the intended motion is understood.

The excavator client profile connects this Raspberry Pi to the Zenoh router running on the ROS PC.

---

# 14. Configure the ROS PC Client

**Machine:** ROS PC  
**Terminal:** T2

In the ROS PC terminal used to communicate with the excavator:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc
```

This terminal now communicates through the local Zenoh router.

If the Command Center is already running in a Zenoh-configured ROS PC terminal, a separate terminal is not required for normal operation.

---

# 15. Check Communication

From a Zenoh-configured ROS PC terminal:

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

For a longer communication-rate check:

```bash
ros2 topic hz /joint_states
```

Allow the frequency check to run for several seconds.

The excavator `/joint_states` stream has been validated across the Zenoh connection between the excavator Raspberry Pi and ROS PC.

---

# 16. Safe Action Communication Check

The following command intentionally uses an invalid joint name.

It is useful for verifying that the complete ROS 2 Action payload reaches the excavator without commanding a real excavator joint.

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

A rejection containing the requested invalid joint confirms that the Action request and its joint-name payload reached the excavator server.

This test has been successfully performed across the Zenoh connection between the ROS PC and excavator Raspberry Pi.

---

# 17. Send an Operational Trajectory

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

For initial physical tests, prefer a small subset-joint trajectory rather than commanding all excavator joints simultaneously.

An example subset trajectory is stored at:

```text
operations/excavator/trajectories/boom_small_test.yaml
```

A structurally valid trajectory is not automatically a physically validated trajectory.

---

# Command Center Integration

# 18. Excavator in Construction Scenarios

The excavator is integrated with the construction-site Command Center.

The high-level architecture is:

```text
Construction Scenario
        │
        ▼
Scenario Manager
        │
        ▼
Excavator Task Client
        │
        │ FollowJointTrajectory
        ▼
Excavator Trajectory Server
        │
        ▼
Physical Excavator
```

Excavator trajectories can therefore be coordinated with dump truck operations through scenario YAML files stored under:

```text
operations/scenarios/
```

The Scenario Manager supports excavator trajectory tasks in addition to dump truck tasks.

This allows a scenario to contain operations such as:

```text
Dump Truck Task
       │
       ▼
Excavator Trajectory
       │
       ▼
Wait
       │
       ▼
Parallel Robot Operations
       │
       ▼
Next Construction Task
```

The excavator uses the same Zenoh communication backbone as the dump trucks.

---

# Safety

# 19. Hardware Safety

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

Automatic homing should remain disabled unless the physical system has been explicitly prepared for the intended motion.

---

# Current Development Status

# 20. Verified

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
- ROS 2 Topic communication between the excavator Pi and ROS PC
- Zenoh communication between the excavator Pi and ROS PC
- `/joint_states` communication across Zenoh
- `FollowJointTrajectory` Action communication across Zenoh
- safe invalid-joint Action payload verification across Zenoh
- Command Center excavator task integration
- mixed dump truck and excavator scenario integration

Automated tests currently cover configuration loading, trajectory loading, trajectory validation, and Action goal validation.

---

# 21. Pending Hardware Validation

Physical closed-loop excavator control is **not yet considered fully validated**.

Known areas requiring additional hardware work include:

- boom sensor/calibration behavior
- boom closed-loop motion
- swing sensor/control behavior
- complete multi-joint physical trajectories
- final tuning of joint control parameters

These are physical hardware and calibration validation items.

They are separate from the ROS 2 software architecture and Zenoh communication layer, which have been integrated and tested.

Hardware validation should be resumed after the physical system and calibration are ready for controlled testing.

---

# 22. ROS 2 Communication Status

The project currently uses:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

for normal multi-machine ROS 2 operation.

The standard topology is:

```text
ONE ROS PC
    │
    └── ONE Zenoh Router
              │
              ├── Dump Truck Pi
              ├── Dump Truck Pi
              ├── ...
              └── Excavator Pi
```

The excavator has been tested through this architecture for:

- ROS 2 topic discovery
- `/joint_states` transport
- `FollowJointTrajectory` Action discovery
- `FollowJointTrajectory` Action payload transport

Users should normally configure communication using:

```bash
source network/setup_zenoh.sh ...
```

rather than manually exporting middleware or peer settings.

Detailed network configuration is documented in:

```text
network/README.md
```

The previous DDS setup remains available through:

```text
network/setup_network.sh
```

as an alternative/fallback configuration.

---

# 23. Previous DDS Testing

Earlier excavator integration testing evaluated multiple DDS configurations.

CycloneDDS successfully transported both ROS 2 topics and `FollowJointTrajectory` Actions between the ROS computer and excavator Raspberry Pi.

Other DDS configurations showed communication or compatibility issues during testing.

These results were useful during development, but CycloneDDS-specific environment configuration is **no longer the standard excavator startup procedure**.

Normal operation now uses the project-wide Zenoh configuration.

The previous DDS helper remains available for troubleshooting and future communication-layer evaluation:

```text
network/setup_network.sh
```

---

# Development Tests

# 24. Run Unit Tests

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

Use:

```text
/usr/bin/python3 -m pytest
```

rather than a user-installed `pytest` executable.

This avoids accidentally running ROS 2 tests using a Python environment associated with Isaac Sim or Isaac Lab.

---

# 25. Standard Physical System Terminal Layout

For normal physical excavator operation:

```text
ROS PC
│
├── T1 — Zenoh Router
│        KEEP RUNNING
│
└── T2 — ROS PC Applications / Command Center
         KEEP RUNNING


Excavator Raspberry Pi
│
└── T1 — Excavator PI Mode
         KEEP RUNNING
```

If dump trucks are operating simultaneously, they use the same ROS PC router:

```text
                            ROS PC
                               │
                         Zenoh Router
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
          ▼                    ▼                    ▼
      Dumptruck1           Dumptruck3          Excavator1
          Pi                   Pi                   Pi
          │                    │                    │
       Hardware             Hardware          Excavator Server
```

Do not start one Zenoh router per robot.

The intended system architecture is:

```text
1 Zenoh Router
      +
1 Command Center
      +
N Physical Robot Clients
```

---

# 26. Where New Excavator Files Belong

Use the following rule when extending the excavator system:

```text
Excavator control software
    → robots/excavator/excavator_control/

Machine configuration / calibration
    → robots/excavator/excavator_control/config/

Excavator launch files
    → robots/excavator/excavator_control/launch/

Operational excavator trajectories
    → operations/excavator/trajectories/

Construction scenarios
    → operations/scenarios/

Shared ROS interfaces
    → common/construction_site_interfaces/

High-level robot coordination
    → command_center/construction_site_control/

ROS network configuration
    → network/
```

The general design rule is:

> Keep reusable robot software and machine configuration separate from operational trajectory and scenario data.