# New Excavator Setup

Do these steps **in order**.

Replace:

```text
excavatorX
```

with the actual excavator name.

---

# 1. Measure Potentiometers

## Required

Connect to Pi with SSH.

```text
Computer
├── Terminal 1 → SSH to Excavator Pi → pot_test.py
└── Terminal 2 → SSH to Excavator Pi → full_extest.py
```

---

## Terminal 1 — Read Potentiometers

```bash
python3 ~/Desktop/pot_test.py
```

Leave this running.

Record the potentiometer values for:

```text
Boom
Arm
Bucket
```

at the required minimum and maximum angles.

Use:

```text
Boom:    -60° to +5°
Arm:      52° to 112°
Bucket:    0° to 90°
```

Record:

| Joint | ADC Channel | Raw @ Min Angle | Raw @ Max Angle |
|---|---:|---:|---:|
| Boom | ___ | ___ | ___ |
| Arm | ___ | ___ | ___ |
| Bucket | ___ | ___ | ___ |

---

## Terminal 2 — Move the Robot

```bash
sudo pigpiod
```

Open:

```text
~/Desktop/full_extest.py
```

Comment out all movements except the **one movement you need**.

Uncomment only the movement required to reach the angle being measured.

Then run:

```bash
python3 ~/Desktop/full_extest.py
```

Repeat until all Boom, Arm, and Bucket min/max values are recorded.

---

# 2. Create `excavatorX.yaml`

## Required

```text
Excavator Pi
└── Terminal 1
```

First update the Pi.

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

git checkout dev
git pull origin dev

source /opt/ros/jazzy/setup.bash
source ~/excavator_env/bin/activate

colcon build --symlink-install 

source install/setup.bash
```

**Always build after pulling.**

Open:

```text
robots/excavator/excavator_control/config/excavator_template.yaml
```

Use **Save As** and create:

```text
robots/excavator/excavator_control/config/excavatorX.yaml
```

Update only the machine-specific items.

## Required Changes

### Name

```yaml
excavator_name: excavatorX
```

### Boom / Arm / Bucket

Enter the values measured in Step 1:

```yaml
adc_channel:
raw_at_min_angle:
raw_at_max_angle:
```

Keep the correct angle ranges:

```text
Boom:    -60° to +5°
Arm:      52° to 112°
Bucket:    0° to 90°
```

### Swing Topic

```yaml
position_topic: /excavatorX/swing_joint_state
```

### AprilTag

Use:

| Excavator | AprilTag ID |
|---|---:|
| Excavator 1 | 7 |
| Excavator 2 | 8 |
| Excavator 3 | 9 |
| Excavator 4 | BACKUP |
| Excavator 5 | 10 |
| Excavator 6 | 11 |
| Excavator 7 | 12 |

### First Test Only

Before motor directions are verified:

```yaml
initial_position:
  enabled: false
```

This prevents automatic startup movement during the first joint test.

---

# 3. Build on the Pi

## Required

```text
Excavator Pi
└── Terminal 1
```

After creating or changing `excavatorX.yaml`:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source ~/excavator_env/bin/activate

colcon build --symlink-install \
  --packages-select excavator_control

source install/setup.bash
```

Validate the config:

```bash
ros2 run excavator_control \
  validate_excavator_config \
  robots/excavator/excavator_control/config/excavatorX.yaml
```

Do not continue if validation fails.

---

# 4. Test Each Joint

Use about **30 to 45° of movement**.

Test:

```text
Swing
Boom
Arm
Bucket
```

one at a time.

---

## 4.1 Prepare the ROS PC

### ROS PC — Setup Terminal

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

git checkout dev
git pull origin dev

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

**Always build after pulling.**

---

## 4.2 Check the AprilTag Setup

Check:

```text
perception/construction_robot_perception/config/tags_multi_truck.yaml
```

and:

```text
perception/construction_robot_perception/config/swing_position_adapters.yaml
```

Make sure the new excavator uses the correct Tag ID from this table:

| Excavator | Tag |
|---|---:|
| 1 | 7 |
| 2 | 8 |
| 3 | 9 |
| 4 | BACKUP |
| 5 | 10 |
| 6 | 11 |
| 7 | 12 |

The adapter must publish:

```text
/excavatorX/swing_joint_state
```

If these files are changed:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

# 4.3 Start the System

## ROS PC — Terminal 1

### Zenoh Router

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh router ros-pc

ros2 run rmw_zenoh_cpp rmw_zenohd
```

**KEEP RUNNING.**

---

## ROS PC — Terminal 2

### Command Center

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc

ros2 launch construction_site_control \
  command_center.launch.py \
  trucks:="" \
  excavators:=excavatorX \
  start_scenario_manager:=false
```

**KEEP RUNNING.**

---

## Excavator Pi — Terminal 1

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source ~/excavator_env/bin/activate
source install/setup.bash

source network/setup_zenoh.sh client excavatorX

sudo pigpiod

ros2 launch excavator_control \
  excavator.launch.py \
  mode:=pi \
  robot_name:=excavatorX
```

**KEEP RUNNING.**

---

## ROS PC — Terminal 3

Use this terminal for test trajectories.

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source install/setup.bash

source network/setup_zenoh.sh client ros-pc
```

---

# 4.4 Swing Direction

Swing uses **site-frame angles**.

```text
                    WINDOW
                 +180 / -180
                      ↑

     CounterClockWise = negative ← Excavator → ClockWise = positive

                      ↓
                     0°
              Opposite the window
              Dump Truck +X direction
```

Rules:

```text
0°       = opposite the window
+ angle  = clockwise
- angle  = counterclockwise
±180°    = window side
```

Keep every target between:

```text
-180° and +180°
```

The controller uses the **shortest rotation**.

Never command a swing target outside ±180°.

Avoid a move that is exactly 180° from the current heading.

---

# 4.5 Test About 30 to 45°

Create one small test trajectory:

```text
operations/excavator/trajectories/excavatorX_joint_test.yaml
```

Example:

```yaml
trajectory_name: excavatorX_joint_test

joints:
  - boom

waypoints:
  - name: position_1
    positions:
      boom: -10.0

  - name: position_2
    positions:
      boom: -50.0

  - name: return
    positions:
      boom: -10.0
```

Change the joint and angles for each test.

Suggested motions:

```text
Boom:    -10 → -50 → -10
Arm:      60 → 100 → 60
Bucket:    0 → 45 → 0
Swing:   current → about ±45° → current
```

Run:

```bash
ros2 run construction_site_control \
  excavator_task_client \
  operations/excavator/trajectories/excavatorX_joint_test.yaml \
  --robot excavatorX \
  --seconds-per-waypoint 5.0
```

For every joint:

```text
[ ] Correct joint moved
[ ] Correct direction
[ ] About 45° movement
[ ] Returned correctly
```

## If Direction is Wrong

Open:

```text
robots/excavator/excavator_control/config/excavatorX.yaml
```

Change that joint's:

```yaml
invert_motor:
```

Then **build again on the Pi**, restart the Pi launch, and repeat the test.

Do not fix a reversed motor by reversing the trajectory.

Continue only after:

```text
[ ] Swing PASS
[ ] Boom PASS
[ ] Arm PASS
[ ] Bucket PASS
```

---

# 5. Run the Excavation Cycle

After all four joints pass, open the current working excavation trajectory:

```text
operations/excavator/trajectories/excavator2_excavation_cycle_test.yaml
```

Copy its contents manually.

Create:

```text
operations/excavator/trajectories/excavatorX_excavation_cycle_test.yaml
```

Adjust the trajectory for the new excavator.

Especially check the **Swing headings**.

Never use a value outside:

```text
[-180°, +180°]
```

---

## Enable Normal Startup

On the Pi, change:

```yaml
initial_position:
  enabled: true
  mode: move
```

Build again:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026

source /opt/ros/jazzy/setup.bash
source ~/excavator_env/bin/activate

colcon build --symlink-install \
  --packages-select excavator_control

source install/setup.bash
```

Restart the Excavator Pi launch.

Wait until the excavator reports:

```text
READY
```

Then run the excavation trajectory from **ROS PC Terminal 3**:

```bash
ros2 run construction_site_control \
  excavator_task_client \
  operations/excavator/trajectories/excavatorX_excavation_cycle_test.yaml \
  --robot excavatorX \
  --seconds-per-waypoint 5.0
```

Confirm:

```text
[ ] Dig
[ ] Scoop
[ ] Lift
[ ] Swing
[ ] Dump
[ ] Return
```

If one complete cycle works, the excavator is validated.

---

# 6. Share the Final Files

Share these two files with James:

```text
robots/excavator/excavator_control/config/excavatorX.yaml
```

and:

```text
operations/excavator/trajectories/excavatorX_excavation_cycle_test.yaml
```

Done.

---

# Optional — Scenario-Based Test

The direct test above uses:

```text
excavator_task_client
```

**Scenario-based operation starts slightly differently.**

After the direct excavation test works, read:

```text
GitHub
CIC-ConRobotics-2026
└── command_center/
    └── README.md
        ├── Section 2 — Run a Scenario
        └── Section 5 — How to Create a Scenario
```

Scenario files are stored in:

```text
operations/scenarios/
```

Scenario-based operation starts the Command Center with:

```text
start_scenario_manager:=true
```

and specifies:

```text
scenario:=YOUR_SCENARIO.yaml
```

Do **not** use the direct-test startup procedure as the scenario procedure.

If time allows, run the validated excavator trajectory once through a simple scenario after the direct test passes.