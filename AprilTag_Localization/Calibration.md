Open Camera

```bash
source /opt/ros/jazzy/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ~/usb_cam_obsbot.yaml
```

```bash
Confirm image
ros2 run rqt_image_view rqt_image_view
close
```

Do the Calibration with the checkr flag in the lab.
```bash
source /opt/ros/jazzy/setup.bash
ros2 run camera_calibration cameracalibrator \
  --no-service-check \
  --size 8x6 \
  --square 0.025 \
  --ros-args --remap image:=/image_raw
```

Once the calibration is done, rename the osbot_tiny_2_lite.yaml calibration file under the camera_info and update it.

Resoures:
https://docs.ros.org/en/kilted/p/camera_calibration/doc/tutorial_mono.html