# Excavator 3 and Truck 1 Physical Integration
This document describes the physical multi-robot workflow validated with

Excavator 3, Truck 1, the overhead AprilTag system, ROS 2 Jazzy, and Zenoh.

The validated operation is defined by:

```text

operations/scenarios/dtex_integration.yaml

```

The scenario executes:

1. Excavator 3 performs the excavation, loading, dumping, and return cycle.

2. Truck 1 follows its validated waypoint route.

3. Truck 1 activates its dump mechanism at the final waypoint.

The validated repository commit is:

```text

3ef66b8 Validate excavator and dump truck integration cycle

```

## 1. System Architecture
```text

                           ROS PC

            ┌────────────────────────────────┐

            │ Zenoh router                   │

            │ Overhead camera                │

            │ AprilTag detector              │

            │ Swing position adapter         │

            │ Truck localization             │

            │ Robot Action servers           │

            │ Scenario Manager               │

            └───────────────┬────────────────┘

                            │ Zenoh

                 ┌──────────┴──────────┐

                 │                     │

          Excavator03 Pi          Dumptruck1 Pi

                 │                     │

        Joint motor control      Drive and dump control

```

Only one Zenoh router should be active.

The normal router host is `ros-pc`.

## 2. AprilTag and Swing Configuration
The overhead camera detects dump trucks, excavators, and fixed landmarks through

the same AprilTag detector.

Excavator swing tag assignments are:

| Excavator | AprilTag ID | Feedback topic |

|---|---:|---|

| excavator1 | 7 | `/excavator1/swing_joint_state` |

| excavator2 | 8 | `/excavator2/swing_joint_state` |

| excavator3 | 9 | `/excavator3/swing_joint_state` |

| excavator4 | 10 | `/excavator4/swing_joint_state` |

| excavator5 | 11 | `/excavator5/swing_joint_state` |

| excavator6 | 12 | `/excavator6/swing_joint_state` |

The swing adapter converts ceiling-camera yaw into the site convention:

```text

site_swing = wrap(camera_yaw - 90 degrees)

```

For the validated physical layout:

```text

left = 0 degrees

up = +90 degrees

down = -90 degrees

```

This is a fixed coordinate transform. It is not startup zeroing and does not

depend on the excavator pose when the adapter starts.

Excavator 3 uses:

```text

Command range: -95 to +95 degrees

Hard observed range: -105 to +105 degrees

```

Targets outside the command range are rejected. The larger observed range

allows the controller to monitor inertial coast without permitting a target in

that region.

## 3. Validated Operational Files
### Scenario
```text

operations/scenarios/dtex_integration.yaml

```

```yaml

scenario_name: truck1_excavator3_integration_test

steps:

  - id: excavator3_move

    type: excavator_trajectory

    robot: excavator3

    task_file: excavator3_excavation_cycle_test.yaml

    seconds_per_waypoint: 5.0

  - id: truck1_short_move

    type: task

    robot: truck1

    task_type: waypoint

    task_file: truck1_waypoints3.yaml

```

Sequential steps advance only after the previous Action reports success.

### Excavator trajectory
```text

operations/excavator/trajectories/excavator3_excavation_cycle_test.yaml

```

The validated seven-waypoint cycle is:

1. Open the bucket for approach at `swing=90`.

2. Lower the boom to the dig position.

3. Curl the bucket and arm inward.

4. Lift the boom.

5. Swing to the truck at `swing=0`.

6. Dump the bucket.

7. Swing back to `swing=90`.

### Truck waypoints
```text

operations/dump_truck/waypoints/truck1_waypoints3.yaml

```

```yaml

waypoints:

  - [0.0, 0.0, 1]

  - [1.4, -1.0, 1]

  - [1.4, 0.0, -1, dump]

```

## 4. Startup Sequence

The current integrated architecture requires one Zenoh router on the ROS PC,

one hardware launch on each robot Raspberry Pi, and one Command Center launch

on the ROS PC.

The Command Center owns the shared overhead camera, AprilTag detector,

Excavator 3 swing-position adapter, Truck 1 localization, robot Action servers,

and Scenario Manager. A separate perception terminal is not required for the

normal integrated workflow.

### Terminal 1: Zenoh router on ROS PC
```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh router ros-pc

ros2 run rmw_zenoh_cpp rmw_zenohd

```

Keep this terminal running. Only one Zenoh router should be active.

### Excavator03 Raspberry Pi
```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client excavator3

sudo pigpiod 2>/dev/null || true

ros2 launch excavator_control \

  excavator.launch.py \

  mode:=pi \

  robot_name:=excavator3 \

  config:=$(ros2 pkg prefix excavator_control)/share/excavator_control/config/excavator3.yaml

```

Expected Action:

```text

/excavator3/upper_arm_controller/follow_joint_trajectory

```

Keep this terminal running.

### Dumptruck1 Raspberry Pi
```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client dumptruck1

sudo pigpiod 2>/dev/null || true

ros2 launch dump_truck_bringup \

  dump_truck_pi.launch.py \

  truck_name:=truck1

```

Keep this terminal running.

Note that `dumptruck1` is the Zenoh device profile, while `truck1` is the ROS

robot name.

### Terminal 2: Integrated Command Center on ROS PC
Start this terminal only after both physical robots are running and the

workspace is clear.

```bash

cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash

source install/setup.bash

source network/setup_zenoh.sh client ros-pc

ros2 launch construction_site_control \

  command_center.launch.py \

  trucks:=truck1 \

  excavators:=excavator3 \

  start_camera:=true \

  start_apriltag:=true \

  start_excavator_perception:=true \

  start_localization:=true \

  start_action_servers:=true \

  start_scenario_manager:=true \

  scenario:=dtex_integration.yaml

```

This integrated launch starts the shared perception stack and the selected

robot-side ROS PC components. Do not start a second `overhead_camera.launch.py`

for normal integrated operation.

The scenario begins when the Scenario Manager starts.

## 5. Preflight Checks
Run these checks from a Zenoh-configured ROS PC terminal before starting the

scenario.

### Perception nodes
```bash

ros2 node list | grep -E \

  'usb_cam|apriltag|swing_position'

```

Expected:

```text

/apriltag

/swing_position_adapter_excavator3

/usb_cam

```

### AprilTag detections
```bash

ros2 topic hz /detections

```

### Excavator swing feedback
```bash

ros2 topic hz /excavator3/swing_joint_state

```

The validated system produced approximately 30 Hz feedback.

Check one message:

```bash

ros2 topic echo /excavator3/swing_joint_state --once

```

### Excavator feedback
```bash

ros2 topic echo /excavator3/joint_states --once

```

### Actions
```bash

ros2 action list -t

```

Confirm that the Excavator 3 and Truck 1 Action servers are available before

the scenario begins.

## 6. Control and Safety Behavior
The Excavator 3 swing controller uses the external sensor-neutral topic:

```text

/excavator3/swing_joint_state

```

The excavator controller is not coupled directly to AprilTag. A future sensing

method can replace the adapter while preserving the same `JointState` topic.

The swing controller includes:

- stale-feedback protection;

- hard observed-angle limits;

- progress monitoring;

- opposite-direction monitoring;

- separate drive-stop and final-acceptance tolerances;

- pulse/coast behavior near the target;

- shared-PWM arbitration awareness.

When another joint owns the shared PWM resource, the swing progress watchdog

is paused. It resumes when Swing owns PWM again. A real Swing stall while Swing

owns PWM still triggers the watchdog.

The controller stops driving at a tighter tolerance and accepts Action

completion at a wider operational tolerance. This allows scenarios to continue

after small residual errors caused by inertia, backlash, or soil load, while

large errors still fail the Action and stop sequential scenario execution.

## 7. Troubleshooting
### Swing feedback is missing
```bash

ros2 node list | grep swing_position

ros2 topic hz /excavator3/swing_joint_state

ros2 topic info /excavator3/swing_joint_state --verbose

```

Confirm that the expected node is:

```text

/swing_position_adapter_excavator3

```

### Swing feedback is stale
Check both detection and adapter rates:

```bash

ros2 topic hz /detections

ros2 topic hz /excavator3/swing_joint_state

```

Do not bypass the stale-feedback watchdog for physical operation.

### Camera or AprilTag node is duplicated

```bash

ros2 node list | grep -E 'usb_cam|apriltag'

```

For normal integrated operation, let `command_center.launch.py` own the shared

camera and AprilTag detector. Do not start a separate

`overhead_camera.launch.py` at the same time.

### Wrong Swing angle
Check the adapter parameters:

```bash

ros2 param get \

  /swing_position_adapter_excavator3 \

  tag_frame

ros2 param get \

  /swing_position_adapter_excavator3 \

  output_topic

ros2 param get \

  /swing_position_adapter_excavator3 \

  camera_to_site_yaw_offset_deg

```

Expected:

```text

tag36h11_9

/excavator3/swing_joint_state

-90.0

```

### Scenario cannot find an Action server
```bash

ros2 action list -t

ros2 node list | sort

```

Confirm that both robot Pis, perception, localization, and the Command Center

are using the same active Zenoh router.

## 8. Files to Keep
These files are part of the validated implementation:

```text

perception/construction_robot_perception/

  construction_robot_perception/swing_position_adapter.py

  config/swing_position_adapters.yaml

  launch/overhead_camera.launch.py

  launch/excavator_perception.launch.py

robots/excavator/excavator_control/

  config/excavator3.yaml

  excavator_control/excavator_trajectory_server.py

operations/

  scenarios/dtex_integration.yaml

  excavator/trajectories/excavator3_excavation_cycle_test.yaml

  dump_truck/waypoints/truck1_waypoints3.yaml

```

Keep the standalone monitor for diagnostics:

```text

tools/perception/swing_tag_monitor.py

```

The following legacy adapter is obsolete after package integration:

```text

tools/perception/swing_position_adapter.py

```

## 9. Validation Record
The physical integration test successfully demonstrated:

- overhead AprilTag tracking for Truck 1 and Excavator 3;

- approximately 30 Hz Swing feedback;

- site-frame absolute Swing angle without startup zeroing;

- four-joint Excavator 3 trajectory execution;

- sequential Scenario Manager coordination;

- Truck 1 waypoint navigation;

- Truck 1 dump actuation;

- ROS PC, Excavator03 Pi, and Dumptruck1 Pi communication through Zenoh.

The validated code and operational data were pushed to `origin/dev` at commit:

```text

3ef66b8

```