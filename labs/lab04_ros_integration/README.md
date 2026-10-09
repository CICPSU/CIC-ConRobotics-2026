# Lab 04 — ROS 2 Excavation and Hauling

In this lab, your team will create a construction scenario using **one excavator and one dump truck**.

The truck begins at the loading position. The excavator digs, loads the truck, and moves its bucket clear. **Only after the excavator trajectory succeeds will the truck drive to the hauling destination.**

You will:

1. Prepare your assigned robot pair and ROS PC.
2. Start the Command Center and check both robots.
3. Prepare and test an excavation-and-loading trajectory.
4. Prepare and test a truck route.
5. Combine the two tasks in a scenario YAML.
6. Observe the ROS 2 system and record the complete operation.

The task you are going to execute is:

```text
Truck waiting at the loading position
                  ↓
Excavator: dig → lift → load truck → clear truck
                  ↓
Excavator Action succeeds
                  ↓
Truck: follow hauling waypoints → stop
                  ↓
Scenario complete
```

The required scenario ends when the truck reaches its destination and stops. Truck-bed dumping and a return trip are optional extensions after the basic scenario works.

---

# Part 1 — Prepare Your Robot Pair

## Step 1 — Connect to Your Assigned Robots

Use VS Code Remote SSH as in the previous labs. Open connections to the ROS PC, the Excavator Pi, and the Dump Truck Pi.

The examples use these names:

| Component | Example | Your assignment |
| --- | --- | --- |
| Active ROS PC / Zenoh router | `ros-pc` | __________ |
| Excavator | `excavator3` | __________ |
| Dump Truck | `dumptruck1` | __________ |
| Group label | `GroupX` | __________ |

Replace these names consistently throughout the commands and YAML files. **`dumptruck1` is the network profile; `truck1` is the ROS robot name.**

Keep separate terminals for these jobs:

| Terminal | Computer | Job |
| --- | --- | --- |
| 1 | ROS PC | Zenoh router |
| 2 | ROS PC | Command Center / Scenario Manager |
| 3 | ROS PC | Individual tests|
| 4 | ROS PC | Comuncation Checks, etc. |
| 1 | Excavator Pi | Excavator control |
| 1 | Dump Truck Pi | Truck hardware |

If the instructor has already started the router or Command Center, use those running services. Do not start duplicate copies.

## Step 2 — Prepare and Build the Repository

Check the existing repository:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
```

If the repository is not installed yet, clone it once instead:

```bash
mkdir -p ~/ws_conrobotics
cd ~/ws_conrobotics
git clone https://github.com/CICPSU/CIC-ConRobotics-2026.git
```

Build on the ROS PC:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Wait for a successful build. 

## Step 3 — Plan the Loading and Hauling Operation

Sketch the construction site. Mark the excavator, digging area, parked truck, departure path, and final truck position.

| Stage | Excavator | Dump truck |
| --- | --- | --- |
| Start | At the checked starting pose | Parked at the loading position |
| Dig and lift | Scoops and raises its bucket | Remains stationary |
| Load | Positions and empties its bucket over the truck bed | Remains stationary |
| Clear | Moves the bucket and arm out of the departure path | Remains stationary |
| Haul | Holds its final pose | Drives through the route and stops |

For the rehearsal, demonstrate the motions with an empty bucket. 
The excavator trajectory must finish with the truck's departure path clear. The software does not measure whether the truck is loaded or whether the bucket is clear; your team must establish those conditions through the tested motion.

### Checkpoint

- [ ] We have one assigned excavator and one assigned truck.
- [ ] We can identify the correct computer for every terminal.
- [ ] The ROS PC build completed successfully.
- [ ] We have agreed on the loading position, hauling destination, and station stop procedure.

---

# Part 2 — Start the ROS 2 System

## Step 4 — Start the Zenoh Router

In **Terminal 1 on the ROS PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh router ros-pc
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Keep this terminal running. Use the one instructor-designated router for the station.

**If the assigned router is ROS-Backup-PC:** use the following setup lines in place of the corresponding Zenoh lines throughout this lab. Every client must select the same router.

| Terminal location and role | Replacement setup line |
| --- | --- |
| Backup PC — router | `source network/setup_zenoh.sh router ros-backup-pc` |
| Backup PC — any client terminal | `source network/setup_zenoh.sh client ros-backup-pc ros-backup-pc` |
| Excavator Pi — client | `source network/setup_zenoh.sh client excavator3 ros-backup-pc` |
| Truck Pi — client | `source network/setup_zenoh.sh client dumptruck1 ros-backup-pc` |

## Step 5 — Start the Command Center

Make sure the overhead camera can see all three fixed floor AprilTags: **16, 17, and 18**. They must be visible together during startup so the system can register the camera to the site map.

In **Terminal 2 on the ROS PC**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc

ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:=truck1 \
  excavators:=excavator3 \
  start_scenario_manager:=false
```

Keep this terminal running during the setup of the individual tests. This starts shared perception, truck localization and its task server, and excavator perception. Setting `start_scenario_manager:=false` lets you check the system and test each motion before starting the combined scenario.

## Step 6 — Start Both Physical Robots

Clear the excavator's working area before startup; initialization can move the boom, arm, and bucket.

In **Terminal 1 on the Excavator Pi**, run:

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

In **Terminal 1 on the Dump Truck Pi**, run:

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

Keep both terminals running. Ask the instructor for the `sudo` password if needed. If `pigpiod` is already running, use the existing daemon.

The excavator needs fresh AprilTag swing feedback to finish initialization. Wait for its successful initialization / `READY` message before sending a trajectory.

## Step 7 — Check Communication and Localization

In **each of Terminals 3 and 4 on the ROS PC**, prepare the environment:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
```

In Terminal 4, run:

```bash
ros2 action list -t
```

Find these two Actions for the example pair:

```text
/excavator3/upper_arm_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]
/truck1/execute_robot_task [construction_site_interfaces/action/ExecuteRobotTask]
```

Look for `state: LOCKED` in the status content, or `SITE REGISTRATION LOCKED` in the Command Center log. Then check the robot feedback:

```bash
ros2 topic echo /truck1/fused_odom --once
ros2 topic echo /excavator3/joint_states --once
```

Confirm that the truck's reported pose agrees with its position on the site map and that feedback continues updating. An Action appearing in the list does not by itself prove that the robot is ready. After registration locks, temporary occlusion of the floor tags is allowed. 

### Checkpoint

- [ ] Both expected Actions are available.
- [ ] The excavator completed initialization.
- [ ] Truck site registration is locked and localization is updating.
- [ ] The camera can observe the required robot tags.

---

# Part 3 — Prepare the Excavator Motion

## Step 8 — Create Your Excavation-and-Loading Trajectory

On the **ROS PC**, create this group file:

```text
operations/excavator/trajectories/GroupX_lab04_excavation.yaml
```

Use your tested Lab 02 four-joint trajectory as the starting point. Copy it into this location using VS Code, then adapt it to the truck's loading position.

The example's angles belong to a particular setup. Inspect and adjust them for your assigned excavator and truck position **before running the copy**.

Set `trajectory_name` to `GroupX_lab04_excavation`. Include all four joints:

```yaml
joints:
  - swing
  - boom
  - arm
  - bucket
```

Build a sequence with these outcomes. You may use additional waypoints to keep the bucket clear of the truck:

| Suggested waypoint | Required outcome |
| --- | --- |
| `approach` | Bucket is positioned above the digging area |
| `lower` | Bucket approaches the material |
| `scoop` | Bucket and arm produce the digging motion |
| `lift` | Bucket rises clear of the digging area |
| `swing_to_truck` | Raised bucket moves over the parked truck bed |
| `load_truck` | Bucket opens over the bed |
| `clear_truck` | Bucket and arm finish outside the truck's departure path |

Every waypoint must contain numeric positions for `swing`, `boom`, `arm`, and `bucket`. Trajectory values are in **degrees**; `/joint_states` reports positions in **radians**.

Use the actual machine's configured joint limits. Swing targets are site-frame headings between `-180` and `180` degrees. The controller selects the shortest rotation from the measured heading; do not add `swing_direction`. Avoid an exactly 180-degree swing change, which the controller rejects as ambiguous.

Check the complete swept path, including the movement from the robot's actual starting pose to the first waypoint. Recheck it when repeating a run.

## Step 9 — Test the Excavator by Itself

With the instructor, rehearse the trajectory independently before combining it with truck motion. Keep the truck stationary and make sure the bucket clears the truck throughout the loading motion.

In **Terminal 3 on the ROS PC**, run:

```bash
ros2 run construction_site_control \
  excavator_task_client \
  operations/excavator/trajectories/GroupX_lab04_excavation.yaml \
  --robot excavator3 \
  --seconds-per-waypoint 5.0
```

Watch the whole motion and check the Action result. The final pose must let the truck leave without contacting the bucket, arm, or excavator body.

`--seconds-per-waypoint` sets trajectory timing. It is not a separate timer that authorizes the truck to move.

### Checkpoint

- [ ] The trajectory uses the intended four joints and checked angles.
- [ ] The excavator performs the digging and loading sequence.
- [ ] The final pose clears the truck's departure path.
- [ ] The individual trajectory succeeds.

---

# Part 4 — Prepare the Dump Truck Route

## Step 10 — Create Your Hauling Waypoints

On the **ROS PC**, create:

```text
operations/dump_truck/waypoints/GroupX_lab04_haul.yaml
```

Use a previously tested truck route if it fits this loading position and destination. Otherwise, create a short route with an exit point and a destination point, using the instructor's site map.

The following is an **editing template**. Replace every `REPLACE_WITH_...` item with a measured numeric coordinate before testing:

```yaml
waypoints:
  - [REPLACE_WITH_DESTINATION_X, REPLACE_WITH_DESTINATION_Y, 1]
```

| Field | Meaning |
| --- | --- |
| First value | Target `x` in meters in the site/map frame |
| Second value | Target `y` in meters in the same frame |
| Third value | Travel direction: `1` forward, `-1` reverse |
| Fourth value | dump: `dump` |

Coordinates are **absolute map positions**, not distances from the truck's current position. Choose reachable points that account for its starting heading and turning space. A waypoint list does not provide obstacle avoidance.

For this first scenario, use a short hauling route that ends with the truck stopped. Do not add a fourth `dump` field unless the instructor has approved and tested that extra truck-bed operation.

Record your route:

| Position | x (m) | y (m) | Direction / heading note |
| --- | --- | --- | --- |
| Truck loading/start pose | | | |
| Exit waypoint | | | |
| Hauling destination | | | |

## Step 11 — Test the Truck by Itself

Place the excavator in its checked final clearance pose. Position the empty truck at the loading/start pose and verify its localization.

In **Terminal 3 on the ROS PC**, send the truck task:

```bash
ros2 action send_goal \
  /truck1/execute_robot_task \
  construction_site_interfaces/action/ExecuteRobotTask \
  "{robot_name: truck1, task_type: waypoint, task_file: GroupX_lab04_haul.yaml}" \
  --feedback
```

This command starts truck motion. Observe its departure, turns, and final stop. Look for `success: true` in the result; a goal being accepted is only the start of the task.

After the test, return the truck to the same loading position and heading using the station's approved procedure. Verify localization again. Do not rerun the route from its destination and assume it will behave the same way.

### Checkpoint

- [ ] Every waypoint contains numeric site coordinates and a valid direction.
- [ ] The truck leaves the loading position without contacting the excavator.
- [ ] It reaches the hauling destination, stops, and reports success.
- [ ] We restored the starting conditions for the combined run.

---

# Part 5 — Create the Combined Scenario

## Step 12 — Write the Scenario YAML

On the **ROS PC**, create:

```text
operations/scenarios/GroupX_lab04_dig_then_haul.yaml
```

Paste the following, then replace the group label and robot names with your assignments:

```yaml
scenario_name: GroupX_lab04_dig_then_haul

steps:
  - id: excavate_load_and_clear
    type: excavator_trajectory
    robot: excavator3
    task_file: GroupX_lab04_excavation.yaml
    seconds_per_waypoint: 5.0

  - id: haul_to_destination
    type: task
    robot: truck1
    task_type: waypoint
    task_file: GroupX_lab04_haul.yaml
```

Both `- id:` entries are at the same indentation directly under `steps:`. They execute in order.

| Step | Action | Requirement for the next step |
| --- | --- | --- |
| `excavate_load_and_clear` | Excavator `FollowJointTrajectory` | The entire trajectory succeeds, including its clearance pose |
| `haul_to_destination` | Truck `ExecuteRobotTask` | The route succeeds; the scenario can then complete |

If the excavator Action fails or its goal is rejected, the Scenario Manager aborts the sequence and does not send the truck task. If it is still waiting for the excavator result, the truck step has not started.

Before continuing, confirm all three files exist on the ROS PC:

```bash
ls operations/excavator/trajectories/GroupX_lab04_excavation.yaml \
   operations/dump_truck/waypoints/GroupX_lab04_haul.yaml \
   operations/scenarios/GroupX_lab04_dig_then_haul.yaml
```

**Editing only these YAML files does not require rebuilding the workspace.**

### Checkpoint

- [ ] The excavation step appears before the hauling step.
- [ ] The robot names match the Command Center launch arguments.
- [ ] Both `task_file` names exactly match our tested files.
- [ ] We can explain what authorizes the truck to start.

---

# Part 6 — Run the Excavation-and-Hauling Scenario

## Step 13 — Check the Start and Run the Scenario

Before pressing Enter, confirm:

- [ ] The truck is stationary at the checked loading position and heading.
- [ ] The excavator is ready and its actual pose is suitable for the trajectory.
- [ ] Localization is current and the digging, loading, and departure areas are clear.
- [ ] Both individual tests succeeded using these same files.
- [ ] No individual task, teleoperation program, or other Scenario Manager is commanding either robot.

Keep the Zenoh running on your **Terminal 1 on the ROS PC**:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh router ros-pc
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Keep running **Terminal 1 on the Excavator Pi**:

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

In **Terminal 1 on the Dump Truck Pi**, run:

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

Return to **Terminal 2 on the ROS PC**, where the Command Center is running with `start_scenario_manager:=false`.

After both individual tasks have finished and both robots are stationary, press **`Ctrl+C` in Terminal 2** to stop that Command Center. Wait for its processes to exit and the terminal prompt to return. Leave the Zenoh router in Terminal 1 and both robot Pi terminals running.

In **the same Terminal 2**, restart the Command Center with the scenario enabled:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc

ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:=truck1 \
  excavators:=excavator3 \
  start_scenario_manager:=true \
  scenario:=GroupX_lab04_dig_then_haul.yaml
```

The `scenario` filename refers to your file in `operations/scenarios/`. Replace it and both robot names with your group's actual assignments.

**This launch starts the Scenario Manager automatically inside the Command Center.** Be ready for robot motion before pressing Enter; there is no separate start prompt. The Command Center also restarts perception and localization, so watch for a new `SITE REGISTRATION LOCKED` message during startup.

From this point onward, **Terminal 2 runs the Command Center with `start_scenario_manager:=true`**, including the Scenario Manager. Keep it open to monitor the complete operation. Terminal 5 is available for checks; do not start a separate Scenario Manager there.

If motion is unexpected, use the station's stop procedure immediately. **Pressing `Ctrl+C` in Terminal 2 stops the Command Center and its Scenario Manager, but does not guarantee cancellation of an Action already running on a robot Pi. Make sure you kill the terminal with Ctrl+C on the Pi** Confirm both robots are stopped before resetting the setup.

If Terminal 2 reports `SCENARIO ABORTED` or `SCENARIO ERROR`, identify the failed step before trying again. After `SCENARIO COMPLETE`, the Command Center remains running; the scenario does not automatically repeat.

For another run, first confirm that both robots are stopped and no earlier Action remains active. Restore the checked start poses using the station's approved procedure, verify feedback, and ensure all three floor tags are visible. Then stop the Command Center with `Ctrl+C` in Terminal 2, wait for it to exit, and repeat the Step 3 launch command with `start_scenario_manager:=true`. This restarts site registration and runs the scenario from the beginning.

**Record a video of your sequence** for submission.


### Final Checkpoint

- [ ] Terminal 2 is running the Command Center with `start_scenario_manager:=true` and our scenario file.
- [ ] The excavator completed one digging-and-loading cycle.
- [ ] The bucket cleared the departure path before the truck moved.
- [ ] The truck started after the excavator's successful result.
- [ ] The truck completed its route and stopped.
- [ ] We recorded the combined operation and its log.

---

# Submission

Submit one set per group:
- Short video of the scenario 
- Three yaml files: Excavation trajectory, Dump Truck Waypoints, Scenario

# Before You Leave

- Close all the terminal with Ctrl+C. 
- Follow the instructor's directions.
