# Model Digging Stack

This version is aligned with the current working setup:
- `robot/modelexcavator.urdf` is the active URDF.
- `scoop_trajectory_server_v2.py` bridges `FollowJointTrajectory` to `/joint_command`.
- `scoop_trajectory_client_v2.py` sends a spec-defined joint trajectory.
- `specs/scoop_cycle_side_dump_v5.yaml` is the current working motion spec.
- Isaac Sim has already been verified to run this stack successfully.

## Files in the folder

- `robot_model_v2.py`
  Shared URDF parser for joints, limits, and chain information.

- `scoop_trajectory_server_v2.py`
  Action server that receives `FollowJointTrajectory` goals and publishes joint commands for Isaac Sim.

- `scoop_trajectory_client_v2.py`
  Spec-driven trajectory client for sending joint-space trajectories.

- `robot/modelexcavator.urdf`
  Current excavator URDF matched to the working scanned model.

- `specs/scoop_cycle_side_dump_v6.yaml` 
  Current working side-dump cycle spec.

## Quick start

### 0) Start Isaac Sim
Open Isaac SIM and .usd file in a different terminal
	•	Load the excavator .usd
	•	Ensure the ROS bridge is active
	•	Isaac should be idle but running
	
### 1) Start the trajectory server
```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026/model_excavator/model_digging_stack
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10
python3 scoop_trajectory_server_v2.py
```

2) In another terminal:

```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026/model_excavator/model_digging_stack
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10
python3 scoop_trajectory_client_v2.py --spec specs/scoop_cycle_side_dump_v6.yaml
```

## Notes

- This stack is currently using **joint-space specs**.
- Angles are expected in **radians** end-to-end.
- `ROS_DOMAIN_ID` should match the Isaac Sim side.
- If you rename or replace the URDF, update the path expected by the server/client.
- If you create a new motion file, place it under `specs/` and pass it with `--spec`.

## Current verified workflow

The following combination is the current known-good setup:
- `robot/modelexcavator.urdf`
- `scoop_trajectory_server_v2.py`
- `scoop_trajectory_client_v2.py`
- `specs/scoop_cycle_side_dump_v6.yaml`

If something stops working, first verify those four items before changing anything else.
