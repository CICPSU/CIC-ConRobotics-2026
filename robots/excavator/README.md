# Excavator
ROS 2 software for the CIC ConRobotics physical model excavators.

The excavator software supports both:

- ****SIM mode**** for software development and integration testing without physical hardware

- ****PI mode**** for controlling a physical excavator through a Raspberry Pi

The primary motion interface is the standard ROS 2:

```text

control_msgs/action/FollowJointTrajectory

```

Each excavator runs inside its own ROS 2 namespace so that multiple excavators can coexist on the same ROS 2 network.

For example:

```text

/excavator1/upper_arm_controller/follow_joint_trajectory

/excavator1/joint_states

/excavator3/upper_arm_controller/follow_joint_trajectory

/excavator3/joint_states

```

Zenoh (`rmw_zenoh_cpp`) is the standard ROS 2 communication method between the active router host, ROS application terminals, and the excavator Raspberry Pis.

---
# 1. System Architecture
The excavator uses the following software flow:

```text

Trajectory YAML

      │

      ▼

Trajectory Client / Scenario Manager

      │

      │ FollowJointTrajectory

      ▼

/excavatorN/upper_arm_controller/

follow_joint_trajectory

      │

      ▼

Excavator Trajectory Server

      │

      ├──────────────────────┐

      │                      │

      ▼                      ▼

   SIM mode                PI mode

      │                      │

      ▼                      ▼

joint_command         GPIO / PWM / ADC

                             │

                             ▼

                    Physical Excavator

```

The four supported excavator joints are:

```text

swing

boom

arm

bucket

```

Trajectories may command all four joints or only a subset of the joints.

Joints not included in a trajectory are not included in the resulting `FollowJointTrajectory` request.

---
## 1.1 Robot Namespaces
Each excavator server is launched with a `robot_name`.

For example:

```text

robot_name:=excavator1

robot_name:=excavator3

```

The robot name becomes the ROS 2 namespace for that excavator.

The primary interfaces therefore follow this pattern:

```text

/<robot_name>/upper_arm_controller/follow_joint_trajectory

/<robot_name>/joint_states

/<robot_name>/joint_command

```

For Excavator 3:

```text

/excavator3/upper_arm_controller/follow_joint_trajectory

/excavator3/joint_states

/excavator3/joint_command

```

This allows multiple excavator servers to operate simultaneously without sharing a global Action or joint-state topic.

The Zenoh device profile and ROS 2 namespace serve different purposes:

```text

source network/setup_zenoh.sh client excavator3

```

configures how the computer connects to the ROS 2 network, while:

```text

robot_name:=excavator3

```

determines the ROS 2 namespace used by the excavator nodes, topics, and Actions.

---
## 1.2 Multi-Machine Communication

For physical operation, excavators communicate through the same active Zenoh router used by the rest of the construction robotics platform.

The router can run on either:

- `ros-pc` — primary router host
- `ros-backup-pc` — backup router host

```text
                      ACTIVE ROUTER HOST
                   ros-pc OR ros-backup-pc
                              │
                         Zenoh Router
                              │
              ┌───────────────┼───────────────┐
              │               │               │
              ▼               ▼               ▼
          Dump Truck      Excavator 1     Excavator 3
              Pi              Pi              Pi
              │               │               │
              ▼               ▼               ▼
           Hardware        Hardware        Hardware
```

Only one Zenoh router should normally be active.

Each physical robot connects independently to the selected router.

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

    │   ├── excavator1.yaml

    │   └── excavator3.yaml

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

```

This separates:

```text

Machine configuration / calibration

        → robots/excavator/excavator_control/config/

Reusable operational trajectories

        → operations/excavator/trajectories/

```

Each physical excavator should have its own machine configuration file.

For example:

```text

excavator1.yaml

excavator3.yaml

```

These files contain machine-specific information such as sensor calibration, GPIO assignments, motor directions, joint limits, and control parameters.

---
# 3. Build the Package
From the repository root:

```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

colcon build \\

  --symlink-install \\

  --packages-select excavator_control

source install/setup.bash

```

Available executables can be checked with:

```bash

ros2 pkg executables excavator_control

```


---
# 4. Inspect an Excavator Configuration

Before physical operation, inspect the machine configuration for the target excavator.

For Excavator 3, the configuration file is:

```text
robots/excavator/excavator_control/config/excavator3.yaml
```

Machine configuration includes items such as:

- joint limits
- sensor calibration
- GPIO assignments
- motor direction
- motor control parameters
- closed-loop control parameters
- home configuration

Configuration structure and software checks do **not** guarantee that physical calibration values, motor directions, sensor mappings, or mechanical limits are correct.

---

# 5. Inspect a Trajectory Before Physical Execution

Trajectory files should be checked against the configuration of the excavator that will execute them.

Before physical execution, confirm:

- supported joint names
- waypoint structure
- the intended target positions
- the configured joint limits
- the target excavator
- the physical workspace
- whether every commanded joint has been physically validated

A structurally correct trajectory is not automatically a physically safe or physically validated trajectory.

For Excavator 3:

> **Do not include `swing` in a physical trajectory until Swing has been separately tested and validated.**

---

# 6. Trajectory YAML Format
A trajectory defines the joints that should be controlled and a sequence of target positions.

Example:

```yaml

trajectory_name: example_trajectory

description: >

  Example excavator trajectory.

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

All trajectory positions are specified in ****degrees****.

The trajectory client converts these values to radians before sending the ROS 2 Action goal.

> ****Current Excavator 3 restriction:**** Swing has not yet been validated. Do not include `swing` in physical Excavator 3 trajectories until Swing has been separately tested and validated.

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

      boom: -40.0

  - name: position_2

    positions:

      boom: -45.0

```

The resulting ROS 2 trajectory contains only:

```text

boom_joint

```

Subset-joint trajectories are particularly useful for:

- isolated joint testing

- calibration work

- hardware debugging

- operations that do not require all excavator joints

Joints not listed in the trajectory are not included in the trajectory command.

---
# Simulation
# 8. Start an Excavator in SIM Mode
SIM mode does not access Raspberry Pi hardware.

For local simulation testing, no multi-machine network setup is required.

Start a simulated Excavator 1 server with:

```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 launch excavator_control \\

  excavator.launch.py \\

  mode:=sim \\

  robot_name:=excavator1

```

The server should run under:

```text

/excavator1

```

The expected Action is:

```text

/excavator1/upper_arm_controller/follow_joint_trajectory

```

SIM commands are published to:

```text

/excavator1/joint_command

```

Using a namespace in SIM mode keeps software testing consistent with the physical multi-excavator architecture.

---
# 9. Send a Trajectory in SIM Mode

Keep the simulated excavator server running.

In another terminal, use the confirmed Command Center excavator client:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run construction_site_control \
  excavator_task_client \
  operations/excavator/trajectories/YOUR_TRAJECTORY.yaml \
  --robot excavator1 \
  --seconds-per-waypoint 3.0
```

The client targets:

```text
/excavator1/upper_arm_controller/follow_joint_trajectory
```

The Action goal should be accepted when the simulated server and trajectory are valid.

---

# 10. Observe SIM Commands
In another terminal:

```bash

ros2 topic echo /excavator1/joint_command

```

The topic is published while a trajectory is executing.

If no trajectory is running, `ros2 topic echo` may wait without displaying a message. This is normal.

---
# Physical Excavator
# 11. Zenoh Network Setup

The excavator Raspberry Pis and ROS application computers use Zenoh (`rmw_zenoh_cpp`) for ROS 2 communication.

The same communication architecture is shared by the dump trucks and other construction robots.

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

Only one Zenoh router should normally be active.

Do not start a separate router for each excavator.

Network configuration is managed through:

```text
network/
├── devices.sh
├── setup_zenoh.sh
└── setup_network.sh
```

The primary router is `ros-pc`. The backup router is `ros-backup-pc`.

Normal physical operation uses:

```text
setup_zenoh.sh
```

---

# 12. Start the Zenoh Router

## Primary Router

**Machine:** `ros-pc`
**Terminal:** T1  
**Keep this terminal running.**

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

The backward-compatible default is:

```bash
source network/setup_zenoh.sh router
```

## Backup Router

If `ros-backup-pc` is used as the router, run the following **on the physical backup ROS PC**:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-backup-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

Selecting `ros-backup-pc` does not remotely start a router on that computer. The router process must actually run on the selected router host.

Only one router should normally be running for the construction robotics system.

If a router is already active for dump trucks or another excavator, do **not** start another router.

---

# 13. Start Physical Excavator 3
> ****WARNING:**** PI mode can command physical motors.

The launch configuration currently uses:

```text

auto_home_on_startup=false

```

Starting the server therefore should not intentionally command the excavator to its home position.

****Machine:**** Excavator03 Raspberry Pi

****Terminal:**** T1

****Keep this terminal running.****

```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client excavator3

sudo pigpiod

ros2 launch excavator_control \\

  excavator.launch.py \\

  mode:=pi \\

  robot_name:=excavator3 \\

  config:=$(ros2 pkg prefix excavator_control)/share/excavator_control/config/excavator3.yaml

```

The command above uses the default primary router (`ros-pc`).

When `ros-backup-pc` is the active router, use:

```bash
source network/setup_zenoh.sh client excavator3 ros-backup-pc
```

This creates the namespaced ROS 2 interfaces:

```text

/excavator3/upper_arm_controller/follow_joint_trajectory

/excavator3/joint_states

```

Do not enable automatic homing unless the physical system has been explicitly prepared for the intended motion.

---
# 14. Configure the ROS Application Client

The ROS application terminal must use the same active router as the excavator.

## Primary ROS PC

**Machine:** `ros-pc`

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc ros-pc
```

The shorter default command is also valid:

```bash
source network/setup_zenoh.sh client ros-pc
```

## Backup ROS PC

**Machine:** `ros-backup-pc`

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-backup-pc ros-backup-pc
```

A local application on the active router host uses the local router endpoint configured by `setup_zenoh.sh`.

If the Command Center is already running in a correctly configured terminal, a separate ROS application client terminal is not required for normal operation.

---

# 15. Check Excavator 3 Communication
From a Zenoh-configured ROS PC terminal:

```bash

ros2 node list | grep excavator3

```

Expected server:

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

Inspect the Action:

```bash

ros2 action info \\

  /excavator3/upper_arm_controller/follow_joint_trajectory

```

The running physical server should appear as one Action server.

Check joint feedback:

```bash

ros2 topic echo /excavator3/joint_states --once

```

For a longer rate check:

```bash

ros2 topic hz /excavator3/joint_states

```

The namespaced Excavator 3 node, Action, and joint-state feedback have been validated across the Zenoh connection.

---
# 16. Safe Action Communication Check
The following command intentionally uses an invalid joint name.

It can be used to verify that a complete ROS 2 Action request reaches Excavator 3 without intentionally commanding a valid excavator joint.

```bash

ros2 action send_goal \\

  /excavator3/upper_arm_controller/follow_joint_trajectory \\

  control_msgs/action/FollowJointTrajectory \\

  "{trajectory: {joint_names: [fake_joint], points: [{positions: [0.0], time_from_start: {sec: 1, nanosec: 0}}]}}"

```

The excavator server should reject the goal with a message similar to:

```text

Unknown joints: ['fake_joint']

```

This rejection is expected.

---
# 17. Send an Operational Trajectory to Excavator 3

Before sending any trajectory to physical hardware:

1. Inspect the YAML file.
2. Confirm the intended joints.
3. Confirm that `swing` is not included until Swing has been validated.
4. Compare the requested positions with the target machine configuration and joint limits.
5. Confirm that the physical workspace is clear.
6. Confirm that the requested motion is appropriate for the current physical configuration.
7. Begin with conservative motion when testing a new trajectory.

For Excavator 3, machine configuration is stored at:

```text
robots/excavator/excavator_control/config/excavator3.yaml
```

Use the confirmed Command Center excavator client to send the trajectory:

```bash
ros2 run construction_site_control \
  excavator_task_client \
  operations/excavator/trajectories/YOUR_TRAJECTORY.yaml \
  --robot excavator3 \
  --seconds-per-waypoint 5.0
```

This targets:

```text
/excavator3/upper_arm_controller/follow_joint_trajectory
```

For new physical trajectories, begin with small movements and subset-joint trajectories before progressing to coordinated multi-joint motion.

> **Do not command Excavator 3 Swing until it has been separately validated.**

---

# Command Center Integration
# 18. Excavators in Construction Scenarios
Excavators are integrated with the construction-site Command Center.

The high-level architecture is:

```text

Construction Scenario

        │

        ▼

Scenario Manager

        │

        │ robot: excavatorN

        ▼

/excavatorN/upper_arm_controller/

follow_joint_trajectory

        │

        ▼

Excavator Trajectory Server

        │

        ▼

Physical Excavator

```

Scenario YAML files are stored under:

```text

operations/scenarios/

```

An excavator step identifies the target excavator using the `robot` field.

For example:

```yaml

- id: excavator3_move

  type: excavator_trajectory

  robot: excavator3

  task_file: three_joint_Mason.yaml

  seconds_per_waypoint: 5.0

```

Unless an explicit `action_name` override is provided, the Scenario Manager resolves:

```yaml

robot: excavator3

```

to:

```text

/excavator3/upper_arm_controller/follow_joint_trajectory

```

Likewise:

```yaml

robot: excavator1

```

resolves to:

```text

/excavator1/upper_arm_controller/follow_joint_trajectory

```

This allows multiple excavators to participate in the same Command Center architecture without sharing a global excavator Action.

Dump truck and excavator tasks can be combined in the same scenario.

For example:

```text

Dump Truck Task

       │

       ▼

Excavator 3 Trajectory

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

The excavators use the same Zenoh communication backbone as the dump trucks.

---
# Safety
# 19. Hardware Safety
Software validation does not guarantee safe physical motion.

A trajectory that reports:

```text

TRAJECTORY IS VALID

```

has passed software structure and configured-limit checks only.

It does not independently verify:

- sensor accuracy

- physical calibration accuracy

- motor direction

- mechanical interference

- actuator behavior under load

- physical workspace clearance

Immediately stop testing if:

- a joint moves in the wrong direction

- an unexpected joint moves

- a joint begins oscillating

- a joint approaches a mechanical limit

- abnormal motor or mechanical noise occurs

- motion does not stop as expected

Automatic homing should remain disabled unless the physical system has been explicitly prepared for the intended motion.

For Excavator 3:

> ****Do not command Swing until Swing has been separately validated.****

---
# Current Development Status
# 20. Verified
The following software and integration functionality has been verified:

- ROS 2 Jazzy package build

- excavator configuration loading

- calibration conversion utilities

- trajectory YAML loading

- trajectory validation

- subset-joint trajectories

- Action goal validation

- SIM trajectory execution

- namespaced SIM interfaces

- launch-based bringup

- Zenoh communication between excavator Pi and ROS PC

- namespaced excavator node discovery

- namespaced `/joint_states`

- namespaced `FollowJointTrajectory`

- Action payload transport over Zenoh

- Command Center excavator routing

- mixed dump truck and excavator scenario execution

- Action feedback propagation to the Scenario Manager

- Action failure propagation to the Scenario Manager

- Scenario abort when an excavator Action reports failure

The following Excavator 3 physical joints have been individually tested using closed-loop position control:

\| Joint | Current status |

\|---|---|

\| Boom | Validated for closed-loop motion |

\| Arm | Validated for closed-loop motion; final tuning remains |

\| Bucket | Validated for closed-loop motion with ADC filtering; final tuning remains |

\| Swing | ****Not validated — do not command**** |

A physical Truck 1 + Excavator 3 scenario has also been used to verify namespaced Command Center routing.

---
# 21. Remaining Excavator 3 Hardware Work
The higher-level ROS 2, Zenoh, namespace, and Command Center architecture is integrated.

Remaining physical work is primarily associated with machine-level behavior and tuning.

Current items include:

- final Arm tracking/tolerance tuning

- final Bucket feedback/control tuning

- continued validation of coordinated multi-joint trajectories

- Swing sensing and control validation

- final physical motion and safety validation

These items should be treated separately from the multi-robot communication and Command Center architecture.

In particular, an Action may correctly reach Excavator 3 and still return a trajectory tolerance failure if the physical joint does not reach the requested final position.

This is expected behavior: the trajectory server should report failure rather than incorrectly reporting a successful trajectory.

---
# 22. ROS 2 Communication Status

Normal multi-machine operation uses:

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

The standard topology is:

```text
ONE ACTIVE ROUTER HOST
    │
    └── ONE Zenoh Router
              │
              ├── Dump Truck Pi
              ├── Dump Truck Pi
              ├── Excavator Pi
              └── ...
```

The active router host can be:

```text
ros-pc
```

or:

```text
ros-backup-pc
```

The primary `ros-pc` router remains the default configuration.

Excavator communication has been tested through the Zenoh architecture for:

- ROS 2 node discovery
- namespaced topic discovery
- `/excavatorN/joint_states` transport
- namespaced `FollowJointTrajectory` discovery
- `FollowJointTrajectory` goal transport
- Action feedback
- Action results
- Command Center scenario routing

Users should configure communication using:

```bash
source network/setup_zenoh.sh ...
```

rather than manually exporting middleware or peer settings.

Detailed network configuration is documented in:

```text
network/README.md
```

---

# 23. Legacy DDS Configuration
Earlier integration work evaluated DDS-based communication.

Those configurations are retained for development history and specialized troubleshooting, but they are not the standard communication path for the excavator system.

Normal physical operation uses:

```text

rmw_zenoh_cpp

```

through:

```text

network/setup_zenoh.sh

```

Do not switch a physical excavator to another ROS 2 communication backend as part of normal startup or hardware troubleshooting.

---
# Development Tests
# 24. Run Unit Tests
Run the excavator unit tests with the system ROS Python environment:

```bash

cd \~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

/usr/bin/python3 -m pytest -v \\

  robots/excavator/excavator_control/test/

```

Then run through `colcon`:

```bash

colcon test \\

  --packages-select excavator_control \\

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

For normal physical excavator operation using the primary router:

```text
ros-pc
│
├── T1 — Zenoh Router
│
└── T2 — ROS Applications / Command Center

Excavator Raspberry Pi
│
└── T1 — Excavator PI Mode
```

For backup-router operation:

```text
ros-backup-pc
│
├── T1 — Zenoh Router
│
└── T2 — ROS Applications / Command Center

Excavator Raspberry Pi
│
└── T1 — Excavator PI Mode
```

If dump trucks and multiple excavators operate simultaneously, they all use the same active router:

```text
                      ACTIVE ROUTER HOST
                   ros-pc OR ros-backup-pc
                              │
                         Zenoh Router
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
          ▼                   ▼                   ▼
      Dumptruck1          Excavator1          Excavator3
          Pi                  Pi                  Pi
          │                   │                   │
       Hardware        Excavator Server    Excavator Server
```

The intended architecture is:

```text
1 Active Router Host
      +
1 Zenoh Router
      +
1 Command Center
      +
N Physical Robot Clients
```

Each excavator uses its own ROS 2 namespace.

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