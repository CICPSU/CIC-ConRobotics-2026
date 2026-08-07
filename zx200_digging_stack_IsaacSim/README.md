ZX200 Digging Stack (Spec-driven)

Files:
- robot_model.py                 Shared URDF parser (joints/limits/chain)
- scoop_trajectory_server.py     Action -> /joint_command bridge (rad, clamps via URDF)
- scoop_trajectory_client.py     Spec-driven client (joint specs + task specs via IK)
- robot/zx200.urdf               URDF (safe to replace/upgrade later)
- specs/*.yaml / specs/*.json    Motion specs (edit these to iterate)

Quick start:

0) Open Isaac Sim and .usd file in a different terminal

	•	Load the excavator .usd
	•	Ensure the ROS bridge is active in Isaac Sim
	•	Isaac Sim should be idle but running

1) From this folder open a terminal different from the one running Isaac Sim:
```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026/zx200_digging_stack
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10
python3 scoop_trajectory_server.py
```

2) In another terminal(in total you need to open 3 terminals):
```bash
cd ~/ws_conrobotics/CIC-ConRobotics-2026/zx200_digging_stack
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10
python3 scoop_trajectory_client.py --spec specs/scoop_cycle_side_dump_v5.yaml
```


Notes:
- YAML requires PyYAML. If you don't want to install it, use the .json specs.
- This stack assumes angles in radians end-to-end.
