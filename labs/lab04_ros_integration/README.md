# Lab 04 — Integrating an Excavator and Dump Truck

In Lab 01, you established ROS 2 communication across computers using Zenoh. In Lab 02, you sent a YAML trajectory to a model excavator. In Lab 03, you sent a waypoint task to a dump truck. This lab connects the two robots through the **Command Center**.

You will run the same two types of robot task in two ways:

| Scenario | What happens? | When is it finished? |
| --- | --- | --- |
| **Sequential** | Truck 1 finishes its waypoint task; then Excavator 3 starts its trajectory. | Both Actions succeed in order. |
| **Parallel** | The truck task and excavator trajectory start in the same parallel block. | Both child Actions succeed. |

**Estimated time: 30 minutes. Work with your assigned group.**

By the end of this lab, you will be able to:

- distinguish a truck waypoint task from an excavator trajectory in a scenario YAML
- explain the difference between **task after task** and **parallel** execution
- launch a scenario using the Command Center
- use the terminal output and physical robots to tell when each task started and completed

> **Instructor preparation:** Start or verify the one shared Zenoh router, both robot Pis, and one Command Center with `start_scenario_manager:=false` before the 30-minute activity. Have Truck 1 localized, Excavator 3 reporting `READY`, and the shared camera/AprilTag setup working. Precheck the two task files and the physical paths **separately and together**. See the setup appendix for exact commands.

| Your station | Instructor fills in |
| --- | --- |
| ROS PC and assigned robot pair | __________ |
| Truck route approved for both runs | __________ |
| Excavator trajectory approved for both runs | __________ |
| Start positions and space between robot work areas | __________ |
| Stop/power cutoff procedure | __________ |

The supplied example scenarios use `truck1_waypoints3.yaml` and `excavator3_excavation_cycle_test.yaml`, which already exist in the course repository. **Their presence does not by itself validate their physical use in your station.** If you used different, validated files in Labs 02 and 03, replace the two `task_file` values in **both** scenario files with those filenames before running.

---

# Part A — Identify What the Command Center Will Run

## Step 1 — Confirm the Starting Conditions (4 minutes)

Look at the physical robots and the instructor's station card. Confirm:

- [ ] The truck starts where its validated route expects it and localization is current.
- [ ] The excavator has completed initialization and reports `READY`.
- [ ] The overhead camera sees the required AprilTags.
- [ ] The truck route stays clear of the excavator and everyone around it.
- [ ] The group's stop procedure is understood.

Open a **ROS PC** terminal, not a Raspberry Pi terminal. As in the previous labs, load ROS 2, your workspace, and the Zenoh client:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc
echo "$RMW_IMPLEMENTATION"
```

Expected: `rmw_zenoh_cpp`. The instructor will confirm both robots and their Actions are available before scenario execution:

```text
/truck1/execute_robot_task
/excavator3/upper_arm_controller/follow_joint_trajectory
```

> The truck uses `ExecuteRobotTask` with a waypoint YAML. The excavator uses `FollowJointTrajectory` with a joint trajectory YAML. The scenario tells the Command Center **which task to send to which robot and when**.

<!-- INSERT IMAGE: images/step01_two_robots.png
Photo of Truck 1 and Excavator 3 in their approved separate work areas.
Label truck route and excavator working area; use actual course equipment. -->

### Checkpoint — Ready to Start

- [ ] We can point to both robots and their separate work areas.
- [ ] We know which Action belongs to each robot.

---

# Part B — Task After Task

## Step 2 — Read the Sequential Scenario (5 minutes)

In a **ROS PC** terminal, create your group's copy. Replace `group1` with your assigned group number:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
cp operations/scenarios/lab04_sequential.yaml \
  operations/scenarios/lab04_group1_sequential.yaml
```

Open your new file in VS Code. The example contains two top-level steps:

```yaml
scenario_name: lab04_truck_then_excavator

steps:
  - id: truck1_route_first
    type: task
    robot: truck1
    task_type: waypoint
    task_file: truck1_waypoints3.yaml

  - id: excavator3_cycle_after_truck
    type: excavator_trajectory
    robot: excavator3
    task_file: excavator3_excavation_cycle_test.yaml
    seconds_per_waypoint: 5.0
```

Before you run it, make a group copy in `operations/scenarios/` with a unique name, such as `lab04_group1_sequential.yaml`. Edit only the `scenario_name`, step `id` values if desired, and the two `task_file` values if your validated Lab 02/03 files have different names.

**Read the indentation:** Both `- id:` lines are directly under `steps:`. They are separate steps. The Scenario Manager waits for the first Action to **succeed** before starting the second. If the first fails, the scenario aborts.

Which robot should move first? `__________`  
What must happen before the other robot starts? `__________`

<!-- INSERT IMAGE: images/step02_sequential_yaml.png
VS Code screenshot showing the two top-level steps and task_file lines. -->

### Checkpoint — Sequential YAML

- [ ] The first step names `truck1` and a validated waypoint task.
- [ ] The second step names `excavator3` and a validated trajectory.
- [ ] Both referenced files exist on the ROS PC under their respective `operations/` folders.

## Step 3 — Run the Sequential Scenario (5 minutes)

The instructor's Command Center stays running without an automatic scenario. When the instructor confirms the space is clear, open a **second ROS PC terminal** for the Scenario Manager:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc

ros2 run construction_site_control scenario_manager_node \
  --ros-args \
  -p scenario:=operations/scenarios/lab04_group1_sequential.yaml \
  -p trucks:=truck1
```

Replace `lab04_group1_sequential.yaml` with your **actual group filename**. The Scenario Manager starts its scenario automatically, so check the work area **before pressing Enter**. Keep this terminal open and observe both robots. The Command Center, router, and robot Pi processes stay running separately.

Look for `SCENARIO START`, a truck task, an excavator trajectory, and finally `SCENARIO COMPLETE`. Note the order in which the two robot tasks begin. If you see `SCENARIO ABORTED`, stop and ask the instructor to check the failed step before trying again.

Take a screenshot named `lab04_sequential_log.png` showing the task order and result. A short video may help you compare the two runs.

<!-- INSERT IMAGE: images/step03_sequential_log.png
Actual ROS PC terminal screenshot showing distinct truck and excavator steps
and SCENARIO COMPLETE; crop to keep step IDs readable. -->

### Checkpoint — Sequential Run

- [ ] The truck's Action began first.
- [ ] The excavator began only after the truck succeeded.
- [ ] The terminal reported `SCENARIO COMPLETE`.

---

# Part C — Run Both Robot Tasks in Parallel

## Step 4 — Read and Prepare the Parallel Scenario (4 minutes)

Make your group's parallel copy on the **ROS PC** (again, replace `group1`):

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
cp operations/scenarios/lab04_parallel.yaml \
  operations/scenarios/lab04_group1_parallel.yaml
```

Open the copy in VS Code. Use the **same validated task files** as in Step 2.

```yaml
scenario_name: lab04_truck_and_excavator_parallel

steps:
  - id: truck_and_excavator_together
    type: parallel
    tasks:
      - id: truck1_parallel_route
        type: task
        robot: truck1
        task_type: waypoint
        task_file: truck1_waypoints3.yaml

      - id: excavator3_parallel_cycle
        type: excavator_trajectory
        robot: excavator3
        task_file: excavator3_excavation_cycle_test.yaml
        seconds_per_waypoint: 5.0
```

Here there is **one top-level step** of type `parallel`. Its `tasks:` list contains one child for each robot. The Scenario Manager starts both child Actions and waits for **both** results. “Parallel” means the tasks overlap in time; the joints of the excavator itself may still move in sequence.

The two robots must stay in **separate, checked work areas**. Parallel execution alone does not coordinate collision avoidance. If either child fails, do not assume the other robot stopped immediately; follow the station's stop procedure.

<!-- INSERT IMAGE: images/step04_parallel_yaml.png
VS Code screenshot highlighting the parent type: parallel, child tasks,
and the distinct robot fields. -->

### Checkpoint — Parallel YAML

- [ ] There is one `type: parallel` parent under `steps:`.
- [ ] Its `tasks:` list contains exactly one truck and one excavator child.
- [ ] Both children reference validated task files.

## Step 5 — Reset and Run the Parallel Scenario (6 minutes)

After `SCENARIO COMPLETE` appears, stop **only the previous Scenario Manager** with `Ctrl + C`. Leave the Command Center, shared Zenoh router, and robot Pis running. Do not start a second Scenario Manager while the first remains active.

With the instructor, return the truck to a valid starting position for its waypoint route. Confirm its localization again. Check the excavator's actual pose and `READY` status, and recheck the two work areas.

From the **ROS PC Scenario Manager terminal**, run:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh client ros-pc

ros2 run construction_site_control scenario_manager_node \
  --ros-args \
  -p scenario:=operations/scenarios/lab04_group1_parallel.yaml \
  -p trucks:=truck1
```

Replace the group filename with yours. **The scenario starts automatically**, so only press Enter after the instructor confirms that both robots are ready and the paths are clear.

Watch for `PARALLEL START` and two child tasks. The parallel step finishes only after both child results succeed; then look for `PARALLEL COMPLETE` and `SCENARIO COMPLETE`.

Save a screenshot named `lab04_parallel_log.png` showing the parallel block and results. Film a short clip that includes **both robots in one frame** so their overlapping motion is visible.

<!-- INSERT IMAGE: images/step05_parallel_log.png
Actual terminal output showing PARALLEL START, both child IDs,
PARALLEL COMPLETE, and SCENARIO COMPLETE. -->

### Checkpoint — Parallel Run

- [ ] Both tasks were started under the same parallel block.
- [ ] Both robots visibly carried out their individual tasks.
- [ ] The manager waited for both to succeed.

---

# Part D — Compare and Submit

## Step 6 — Explain the Difference (3 minutes)

Complete this comparison with your group:

| Observation | Sequential | Parallel |
| --- | --- | --- |
| When did the excavator task start relative to the truck task? | | |
| How many top-level `steps` did the YAML contain? | | |
| What had to succeed before the scenario completed? | | |

**Question:** If Truck 1's Action fails during the sequential scenario, will the excavator task start? What must your group do if one robot has a problem during the parallel scenario?

Submit **one set per group**:

- your `lab04_group1_sequential.yaml` and `lab04_group1_parallel.yaml` (use your actual group filenames)
- `lab04_sequential_log.png` and `lab04_parallel_log.png`
- the short video showing the parallel run
- two or three sentences answering the comparison question

### Before You Leave

Stop your group's Scenario Manager with `Ctrl + C` when the run is complete. The instructor will manage the Command Center and robot power. **Do not stop the shared Zenoh router** while another group is using it.

---

# Lab Complete

You used one Command Center and two robot Actions to run two scheduling patterns. The truck's waypoint task and the excavator's joint trajectory remained the same; the scenario YAML determined whether the tasks ran **one after the other** or **at the same time**. Later, you can add conditions or other tasks to build a larger site scenario.

<!-- INSTRUCTOR IMAGE CHECKLIST
Before distributing this README, capture real course screenshots and replace the comments:
<img src="images/step01_two_robots.png" width="900">
<img src="images/step02_sequential_yaml.png" width="900">
<img src="images/step03_sequential_log.png" width="900">
<img src="images/step04_parallel_yaml.png" width="900">
<img src="images/step05_parallel_log.png" width="900">
Match Lab 01's image width and put each image beside its corresponding step.
Do not publish links to images that have not been captured.
-->

---

# Instructor Setup — Before the 30-Minute Activity

Use the current `command_center/README.md` for the operational sequence. Lab 04 uses **one shared router, Truck 1 Pi, Excavator 3 Pi, and one Command Center**. The example below assumes those are the assigned robots. Configure every terminal for Zenoh; keep the three underlying services running between the two student scenario runs.

Before class, place the three supplied files into the course repository:

| Supplied file | Repository destination |
| --- | --- |
| `Lab04_ROS_Integration_Draft.md` | `labs/lab04_ros_integration/README.md` |
| `lab04_sequential.yaml` | `operations/scenarios/lab04_sequential.yaml` |
| `lab04_parallel.yaml` | `operations/scenarios/lab04_parallel.yaml` |

The student `cp` commands in Steps 2 and 4 expect these scenario files at those destinations. Capture the real lab screenshots into `labs/lab04_ros_integration/images/` before replacing the image comments above with links.

### ROS PC — Router terminal

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source network/setup_zenoh.sh router ros-pc
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Start it only if the shared router is not already running.

### Truck 1 Pi — Hardware terminal

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
  trucks:=truck1 \
  excavators:=excavator3 \
  start_scenario_manager:=false
```

Start the Command Center once with `start_scenario_manager:=false`, as recommended in the operating manual for validating new physical scenarios, and keep it running while students execute each YAML through `scenario_manager_node`. Do not keep two Command Center processes running. The excavator Pi waits for fresh AprilTag swing feedback before READY and does not turn swing at startup. The sample excavation cycle has fixed site-frame swing targets; inspect the measured starting heading and expected sweep before running it. Validate the assigned waypoint route and excavator trajectory individually, then inspect both physical work areas for the parallel run.

Both example scenario YAMLs refer to existing repository filenames. If a Lab 02 or Lab 03 file uses a different name, copy that validated file into its respective `operations/excavator/trajectories/` or `operations/dump_truck/waypoints/` directory and update both scenario references. For repeated truck runs, verify the route is meaningful from **each** actual start pose; return the truck to an approved start between runs if needed. The example scenarios have been checked for YAML structure and file references, **not physically run**.