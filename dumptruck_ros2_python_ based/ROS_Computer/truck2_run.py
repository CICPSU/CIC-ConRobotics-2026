import subprocess
import os
import signal
import time
import sys
import shlex

os.environ["ROS_DOMAIN_ID"] = "10"
os.environ["ROS_AUTOMATIC_DISCOVERY"] = "SUBNET"
os.environ["ROS_STATIC_PEERS"] = "10.170.32.181;10.170.32.194;10.170.32.192"

home = os.path.expanduser("~")
code_folder = f"{home}/codes"
yaml_folder = f"{home}/AprilTag"

default_waypoints = f"{yaml_folder}/truck2_waypoints.yaml"


def get_waypoints_yaml():
    args = sys.argv[1:]

    for i, arg in enumerate(args):
        if arg == "-p" and i + 1 < len(args):
            next_arg = args[i + 1]
            if next_arg.startswith("waypoints_yaml:="):
                return next_arg.split(":=", 1)[1]

        if arg.startswith("waypoints_yaml:="):
            return arg.split(":=", 1)[1]

    if len(args) >= 1 and (args[0].endswith(".yaml") or args[0].endswith(".yml")):
        return args[0]

    return default_waypoints


waypoints_yaml = os.path.expanduser(get_waypoints_yaml())

print(f"Using Truck 2 waypoint YAML: {waypoints_yaml}")

commands = [
    f"""source /opt/ros/jazzy/setup.bash && cd {code_folder} && python3 Odom_test_multi.py --ros-args \
-p wheel_states_topic:=/truck2/wheel_states \
-p odom_topic:=/truck2/odom \
-p odom_frame:=truck2/odom \
-p base_frame:=truck2/base_link \
-p publish_tf:=false""",

    f"""source /opt/ros/jazzy/setup.bash && cd {code_folder} && python3 tag_odom_fusion_node_landmarks_v4.py --ros-args \
-p odom_topic:=/truck2/odom \
-p fused_odom_topic:=/truck2/fused_odom \
-p robot_tag_child_frame:=tag36h11_1 \
-p fused_child_frame:=truck2/base_link_fused \
-p alpha:=0.03 \
-p tag_parent_frame:=default_cam \
-p camera_y_scale:=-1.0 \
-p camera_yaw_scale:=-1.0 \
-p initial_yaw_source:=tag \
-p use_initial_yaw_alignment:=true \
-p odom_x_scale:=1.0 \
-p odom_y_scale:=1.0 \
-p odom_yaw_scale:=1.0 \
-p tag_yaw_offset:=1.57079632679 \
-p landmarks_yaml:={yaml_folder}/landmarks.yaml""",

    f"""source /opt/ros/jazzy/setup.bash && cd {code_folder} && python3 ppwyr_fused_multi_v2.py --ros-args \
-p odom_topic:=/truck2/fused_odom \
-p cmd_vel_topic:=/truck2/cmd_vel \
-p bucket_action_cmd_topic:=/truck2/bucket_action_cmd \
-p bucket_action_status_topic:=/truck2/bucket_action_status \
-p waypoints_yaml:={shlex.quote(waypoints_yaml)}""",
]

processes = []

try:
    for cmd in commands:
        p = subprocess.Popen(["bash", "-c", cmd], preexec_fn=os.setsid)
        processes.append(p)
        time.sleep(1)

    for p in processes:
        p.wait()

except KeyboardInterrupt:
    print("\nStopping truck2 ROS nodes...")
    for p in processes:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)