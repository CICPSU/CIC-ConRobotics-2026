import subprocess
import os
import signal
import time

os.environ["ROS_DOMAIN_ID"] = "10"
os.environ["ROS_AUTOMATIC_DISCOVERY"] = "SUBNET"
os.environ["ROS_STATIC_PEERS"] = "10.170.32.181;10.170.32.194;10.170.32.192;10.170.32.45"

folder = os.path.expanduser("~/Diffdrive")

commands = [
    f"""source /opt/ros/jazzy/setup.bash && cd {folder} && python3 Motor_Drive_Node_multi.py --ros-args \
-p cmd_vel_topic:=/truck1/cmd_vel \
-p wheel_states_topic:=/truck1/wheel_states \
-p node_name_suffix:=truck1""",

    f"""source /opt/ros/jazzy/setup.bash && cd {folder} && python3 bucket_action_node_multi_truck1.py --ros-args \
-p bucket_action_cmd_topic:=/truck1/bucket_action_cmd \
-p bucket_action_status_topic:=/truck1/bucket_action_status"""
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
    print("\nStopping truck1 Pi nodes...")
    for p in processes:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)