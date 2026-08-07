#!/usr/bin/env python3
"""
Truck 4 runner that uses the overhead AprilTag yaw continuously.

Why:
Truck 4 wheel odometry is not reliable enough for heading feedback.
The original fusion runner uses the AprilTag yaw only at startup because
use_tag_yaw_correction defaults to false. This runner keeps correcting yaw
from tag36h11_3 while retaining the original waypoint controller.
"""

import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "10"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SUBNET"
os.environ["ROS_STATIC_PEERS"] = (
    "10.170.32.181;10.170.32.194;10.170.32.192;"
    "10.170.32.45;10.170.32.219"
)

home = Path.home()
code_folder = home / "codes"
yaml_folder = home / "AprilTag"


def get_waypoints_yaml() -> Path:
    args = sys.argv[1:]

    for i, arg in enumerate(args):
        if (
            arg == "-p"
            and i + 1 < len(args)
            and args[i + 1].startswith("waypoints_yaml:=")
        ):
            return Path(args[i + 1].split(":=", 1)[1]).expanduser()

        if arg.startswith("waypoints_yaml:="):
            return Path(arg.split(":=", 1)[1]).expanduser()

    if args and args[0].endswith((".yaml", ".yml")):
        return Path(args[0]).expanduser()

    return yaml_folder / "truck4_waypoints.yaml"


waypoints_yaml = get_waypoints_yaml()

required = [
    code_folder / "Odom_test_multi.py",
    code_folder / "tag_odom_fusion_node_landmarks_v4.py",
    code_folder / "ppwyr_fused_multi_v2.py",
    yaml_folder / "landmarks.yaml",
    waypoints_yaml,
]

missing = [str(path) for path in required if not path.exists()]
if missing:
    raise SystemExit("Missing required file(s):\n" + "\n".join(missing))

commands = [
    (
        "ODOM",
        f"source /opt/ros/jazzy/setup.bash && "
        f"cd {shlex.quote(str(code_folder))} && "
        f"python3 Odom_test_multi.py --ros-args "
        f"-p wheel_states_topic:=/truck4/wheel_states "
        f"-p odom_topic:=/truck4/odom "
        f"-p odom_frame:=truck4/odom "
        f"-p base_frame:=truck4/base_link "
        f"-p publish_tf:=false"
    ),
    (
        "FUSION",
        f"source /opt/ros/jazzy/setup.bash && "
        f"cd {shlex.quote(str(code_folder))} && "
        f"python3 tag_odom_fusion_node_landmarks_v4.py --ros-args "
        f"-p odom_topic:=/truck4/odom "
        f"-p fused_odom_topic:=/truck4/fused_odom "
        f"-p robot_tag_child_frame:=tag36h11_3 "
        f"-p fused_child_frame:=truck4/base_link_fused "
        f"-p alpha:=0.25 "
        f"-p tag_parent_frame:=default_cam "
        f"-p camera_y_scale:=-1.0 "
        f"-p camera_yaw_scale:=-1.0 "
        f"-p initial_yaw_source:=tag "
        f"-p use_initial_yaw_alignment:=true "
        f"-p use_tag_yaw_correction:=true "
        f"-p odom_x_scale:=1.0 "
        f"-p odom_y_scale:=1.0 "
        f"-p odom_yaw_scale:=0.0 "
        f"-p reject_tag_jump:=false "
        f"-p tag_yaw_offset:=1.570796326795 "
        f"-p landmarks_yaml:={shlex.quote(str(yaml_folder / 'landmarks.yaml'))}"
    ),
    (
        "CONTROLLER",
        f"source /opt/ros/jazzy/setup.bash && "
        f"cd {shlex.quote(str(code_folder))} && "
        f"python3 ppwyr_fused_multi_v2.py --ros-args "
        f"-p odom_topic:=/truck4/fused_odom "
        f"-p cmd_vel_topic:=/truck4/cmd_vel "
        f"-p bucket_action_cmd_topic:=/truck4/bucket_action_cmd "
        f"-p bucket_action_status_topic:=/truck4/bucket_action_status "
        f"-p waypoints_yaml:={shlex.quote(str(waypoints_yaml))}"
    ),
]

processes = []

try:
    for label, command in commands:
        process = subprocess.Popen(
            ["bash", "-c", command],
            preexec_fn=os.setsid,
        )
        processes.append(process)
        time.sleep(1.0)

        if process.poll() is not None:
            raise RuntimeError(
                f"{label} exited immediately with code {process.returncode}"
            )

        print(f"{label} started, pid={process.pid}")

    for process in processes:
        process.wait()

except (KeyboardInterrupt, RuntimeError) as error:
    print(f"\nStopping truck 4: {error}")

finally:
    for process in processes:
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
