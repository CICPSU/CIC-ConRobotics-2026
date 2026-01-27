ZX200 Digging Stack (Spec-driven)

Files:
- robot_model.py                 Shared URDF parser (joints/limits/chain)
- scoop_trajectory_server.py     Action -> /joint_command bridge (rad, clamps via URDF)
- scoop_trajectory_client.py     Spec-driven client (joint specs + task specs via IK)
- ik/ik_interface.py             IK interface
- ik/ik_planar_approx.py         Simple geometric IK (good enough to iterate)
- robot/zx200.urdf               URDF (safe to replace/upgrade later)
- specs/*.yaml / specs/*.json    Motion specs (edit these to iterate)

Quick start:
1) From this folder:
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID = 10
python3 scoop_trajectory_server.py
```

2) In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID = 10
python3 scoop_trajectory_client.py --spec specs/scoop_cycle_v5.yaml
```

Task-space (IK) version:
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID = 10
python3 scoop_trajectory_client.py --spec specs/scoop_cycle_task_v3.yaml
```

Notes:
- YAML requires PyYAML. If you don't want to install it, use the .json specs.
- This stack assumes angles in radians end-to-end.
