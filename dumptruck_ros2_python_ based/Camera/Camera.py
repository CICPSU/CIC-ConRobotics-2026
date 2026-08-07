import subprocess
import os
import signal
import time

os.environ["ROS_DOMAIN_ID"] = "10"
os.environ["ROS_AUTOMATIC_DISCOVERY"] = "SUBNET"
os.environ["ROS_STATIC_PEERS"] = "10.170.32.181;10.170.32.194;10.170.32.192;10.170.32.45;10.170.32.219"

home = os.path.expanduser("~")

commands = [
    f"source /opt/ros/jazzy/setup.bash && ros2 run usb_cam usb_cam_node_exe --ros-args --params-file {home}/AprilTag/usb_cam_osbot.yaml",

    f"source /opt/ros/jazzy/setup.bash && ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info --params-file {home}/AprilTag/tags_multi_truck.yaml"
]

processes = []

try:
    # Start camera first
    p1 = subprocess.Popen(["bash", "-c", commands[0]], preexec_fn=os.setsid)
    processes.append(p1)

    time.sleep(2)

    # Start AprilTag
    p2 = subprocess.Popen(["bash", "-c", commands[1]], preexec_fn=os.setsid)
    processes.append(p2)

    for p in processes:
        p.wait()

except KeyboardInterrupt:
    print("\nStopping camera and AprilTag...")
    for p in processes:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
