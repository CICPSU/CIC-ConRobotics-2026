# AprilTag Localization (ROS2 Jazzy)
This repository provides a minimal pipeline to detect AprilTags and estimate their 3D pose using a USB camera.

## System Overview
- Camera: USB (tested with OBSBOT Tiny 2 Lite)
- Resolution: 1920x1080
- ROS2: Jazzy
- Detection: apriltag_ros
- Output: /tf (camera → tag)

Before moving on, save the file 'camera_info' under the location below.
/home/XXXX/.ros/
The file should be located at /home/XXXX/.ros/camera_info/


## 1. Install dependencies

```bash
sudo apt update
sudo apt install \
  ros-jazzy-usb-cam \
  ros-jazzy-v4l2-camera \
  ros-jazzy-apriltag-ros \
  ros-jazzy-camera-calibration \
  ros-jazzy-rqt-image-view
```

## 2. Start Camera
Terminal 1
```bash
source /opt/ros/jazzy/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ~/usb_cam_obsbot.yaml
```

## 3. Check image
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
```

## 4. Run AprilTag detection
Terminal 2
```bash
source /opt/ros/jazzy/setup.bash
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/image_raw \
  -r camera_info:=/camera_info \
  --params-file ./tags.yaml
```

## 5. Check output
Terminal 3
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /tf | grep -A 20 tag36h11_0
```
Terminal 4
```bash
source /opt/ros/jazzy/setup.bash
python3 ./read_tag_tf.py
```



Notes
* Tag size must match real-world size in tags.yaml
* Camera must be calibrated for accurate pose estimation



