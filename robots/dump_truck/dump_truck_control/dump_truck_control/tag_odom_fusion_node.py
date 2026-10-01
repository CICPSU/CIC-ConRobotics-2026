#!/usr/bin/env python3
"""
tag_odom_fusion_node_landmarks_v4.py

AprilTag landmark calibrated absolute-map pose + odom fusion for a differential-drive robot.

Purpose
-------
This node extends the working v3 idea:

    AprilTag robot pose = absolute observation
    wheel odom          = smooth relative motion

by adding fixed floor AprilTags as landmarks. The landmark tags are used to estimate
a startup-validated, run-locked 2D transform:

    camera/tag raw XY  -->  field/map XY

Then the robot tag is converted through that transform and fused with odom.

Recommended setup
-----------------
- Overhead fixed camera.
- One AprilTag on the vehicle. Its top edge points to the vehicle front.
- Three AprilTags on the floor as fixed landmarks.
- Put landmark coordinates in a YAML file in your desired absolute map frame.

Core fusion model
-----------------
1. The node requires fresh Tag 16/17/18 observations from the same images.
   After a stable multi-image window passes validation, camera_to_map is LOCKED.
   It is never recomputed during this node lifetime, even after landmark loss.
   Relocated tags require new surveyed center coordinates in landmarks.yaml.
2. The robot tag TF is converted into an absolute map pose.
3. At startup, the node records:
       initial robot map pose from AprilTag
       initial odom pose
4. After startup:
       predicted pose = initial map pose + rotated odom delta
5. If alpha > 0 and robot tag is visible:
       slowly correct predicted pose toward current AprilTag absolute pose

This keeps waypoints fully absolute in map coordinates.
"""

import json
import math
from typing import Dict, Optional, Tuple, List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
import tf2_ros

from dump_truck_control.site_registration import (
    Similarity2D,
    StartupSiteRegistration,
    load_landmark_layout,
)


Pose2D = Tuple[float, float, float]
Point2D = Tuple[float, float]


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def rotate_2d(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def mean_angle(angles: List[float]) -> float:
    if not angles:
        return 0.0
    sx = sum(math.cos(a) for a in angles)
    sy = sum(math.sin(a) for a in angles)
    return math.atan2(sy, sx)


class TagOdomFusionLandmarksNode(Node):
    def __init__(self):
        super().__init__('tag_odom_fusion_landmarks_node')

        # Topics / frames
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('fused_odom_topic', '/fused_odom')
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('tag_parent_frame', 'default_cam')
        self.declare_parameter('robot_tag_child_frame', 'tag36h11_0')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('fused_child_frame', 'base_link_fused')

        # Startup-only floor-tag registration. The legacy transform-alpha
        # parameter remains declared for compatibility but is NOT used to
        # update a locked transform. Quality limits live in landmarks.yaml.
        self.declare_parameter('landmarks_yaml', '')
        self.declare_parameter('use_landmark_calibration', True)
        self.declare_parameter('allow_landmark_scale', True)
        self.declare_parameter('min_landmarks_for_update', 3)
        self.declare_parameter('landmark_timeout_sec', 1.0)
        self.declare_parameter('landmark_transform_alpha', 0.15)
        self.declare_parameter('publish_landmark_debug', True)

        # Fallback/manual camera transform, also used as initial transform before landmarks are seen.
        # These are applied to raw camera tag translation before similarity estimation/use.
        # Keep camera_y_scale=-1.0 if camera Y and field/map Y are opposite in your setup.
        self.declare_parameter('camera_x_scale', 1.0)
        self.declare_parameter('camera_y_scale', -1.0)
        self.declare_parameter('camera_yaw_scale', -1.0)
        self.declare_parameter('manual_camera_to_map_yaw', 0.0)
        self.declare_parameter('manual_camera_map_x_offset', 0.0)
        self.declare_parameter('manual_camera_map_y_offset', 0.0)

        # Robot tag orientation and geometry
        # You found -90 deg worked in v3. Keep it unless your physical tag orientation changes.
        self.declare_parameter('tag_yaw_offset', -1.57079632679)
        self.declare_parameter('tag_to_base_forward', 0.0)
        self.declare_parameter('tag_to_base_left', 0.0)

        # Fusion behavior
        self.declare_parameter('alpha', 0.0)
        self.declare_parameter('robot_tag_timeout_sec', 0.5)
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('reject_tag_jump', True)
        self.declare_parameter('max_tag_correction_m', 0.35)
        self.declare_parameter('use_tag_yaw_correction', False)
        self.declare_parameter('tag_yaw_alpha', 0.08)
        self.declare_parameter('max_tag_yaw_step_deg', 20.0)
        self.declare_parameter('max_tag_yaw_reanchor_deg', 90.0)
        self.declare_parameter('stable_tag_yaw_observations', 3)
        self.declare_parameter('max_yaw_correction_step_deg', 5.0)

        # Odom delta correction and yaw alignment
        self.declare_parameter('odom_x_scale', 1.0)
        self.declare_parameter('odom_y_scale', 1.0)
        self.declare_parameter('odom_yaw_scale', 1.0)
        self.declare_parameter('odom_to_map_yaw_offset', 0.0)
        self.declare_parameter('use_initial_yaw_alignment', True)
        self.declare_parameter('initial_yaw_source', 'tag')  # tag or fixed
        self.declare_parameter('fixed_initial_yaw', 0.0)

        # Read parameters
        self.odom_topic = self.get_parameter('odom_topic').value
        self.fused_odom_topic = self.get_parameter('fused_odom_topic').value
        self.tf_topic = self.get_parameter('tf_topic').value
        self.tag_parent_frame = self.get_parameter('tag_parent_frame').value
        self.robot_tag_child_frame = self.get_parameter('robot_tag_child_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.fused_child_frame = self.get_parameter('fused_child_frame').value

        self.landmarks_yaml = self.get_parameter('landmarks_yaml').value
        self.use_landmark_calibration = bool(self.get_parameter('use_landmark_calibration').value)
        self.allow_landmark_scale = bool(self.get_parameter('allow_landmark_scale').value)
        self.min_landmarks_for_update = int(self.get_parameter('min_landmarks_for_update').value)
        self.landmark_timeout_sec = float(self.get_parameter('landmark_timeout_sec').value)
        self.landmark_transform_alpha = float(self.get_parameter('landmark_transform_alpha').value)
        self.publish_landmark_debug = bool(self.get_parameter('publish_landmark_debug').value)

        self.camera_x_scale = float(self.get_parameter('camera_x_scale').value)
        self.camera_y_scale = float(self.get_parameter('camera_y_scale').value)
        self.camera_yaw_scale = float(self.get_parameter('camera_yaw_scale').value)
        self.manual_camera_to_map_yaw = float(self.get_parameter('manual_camera_to_map_yaw').value)
        self.manual_camera_map_x_offset = float(self.get_parameter('manual_camera_map_x_offset').value)
        self.manual_camera_map_y_offset = float(self.get_parameter('manual_camera_map_y_offset').value)

        self.tag_yaw_offset = float(self.get_parameter('tag_yaw_offset').value)
        self.tag_to_base_forward = float(self.get_parameter('tag_to_base_forward').value)
        self.tag_to_base_left = float(self.get_parameter('tag_to_base_left').value)

        self.alpha = float(self.get_parameter('alpha').value)
        self.robot_tag_timeout_sec = float(self.get_parameter('robot_tag_timeout_sec').value)
        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.reject_tag_jump = bool(self.get_parameter('reject_tag_jump').value)
        self.max_tag_correction_m = float(self.get_parameter('max_tag_correction_m').value)
        self.use_tag_yaw_correction = bool(self.get_parameter('use_tag_yaw_correction').value)
        self.tag_yaw_alpha = max(0.0, min(1.0, float(self.get_parameter('tag_yaw_alpha').value)))
        self.max_tag_yaw_step = math.radians(float(self.get_parameter('max_tag_yaw_step_deg').value))
        self.max_tag_yaw_reanchor = math.radians(float(self.get_parameter('max_tag_yaw_reanchor_deg').value))
        self.stable_tag_yaw_observations = max(2, int(self.get_parameter('stable_tag_yaw_observations').value))
        self.max_yaw_correction_step = math.radians(float(self.get_parameter('max_yaw_correction_step_deg').value))

        self.odom_x_scale = float(self.get_parameter('odom_x_scale').value)
        self.odom_y_scale = float(self.get_parameter('odom_y_scale').value)
        self.odom_yaw_scale = float(self.get_parameter('odom_yaw_scale').value)
        self.odom_to_map_yaw_offset = float(self.get_parameter('odom_to_map_yaw_offset').value)
        self.use_initial_yaw_alignment = bool(self.get_parameter('use_initial_yaw_alignment').value)
        self.initial_yaw_source = str(self.get_parameter('initial_yaw_source').value).lower()
        self.fixed_initial_yaw = float(self.get_parameter('fixed_initial_yaw').value)

        # Landmark definitions: child_frame -> map pose/point
        self.landmarks: Dict[str, Dict[str, float]] = self.load_landmarks(self.landmarks_yaml)

        self.startup_registration = None
        if self.use_landmark_calibration:
            if self.min_landmarks_for_update != 3:
                raise ValueError(
                    'Startup registration requires min_landmarks_for_update=3; '
                    'a two-tag fallback is not permitted.')
            self.startup_registration = StartupSiteRegistration(
                self.landmarks,
                self.landmark_layout.registration,
                allow_scale=self.allow_landmark_scale,
                timeout_sec=self.landmark_timeout_sec,
                not_before_ns=self.get_clock().now().nanoseconds,
            )

        # Live TF cache: child_frame -> (raw_x, raw_y, raw_yaw, time)
        self.latest_tag_raw: Dict[str, Tuple[float, float, float, rclpy.time.Time]] = {}

        # Manual transform is used ONLY when landmark calibration is explicitly
        # disabled. With calibration enabled, initialization is gated on LOCKED.
        self.camera_to_map = Similarity2D(
            scale=1.0,
            theta=self.manual_camera_to_map_yaw,
            tx=self.manual_camera_map_x_offset,
            ty=self.manual_camera_map_y_offset,
        )
        self.landmark_transform_ready = False

        # Live inputs
        self.current_odom_pose: Optional[Pose2D] = None
        self.current_odom_twist = None
        self.latest_robot_tag_pose_map: Optional[Pose2D] = None
        self.latest_robot_tag_time = None
        self.robot_tag_sequence = 0
        self.last_corrected_tag_sequence = 0
        self.last_robot_tag_stamp_ns = None
        self.last_trusted_tag_yaw = None
        self.yaw_candidate = None
        self.yaw_candidate_count = 0

        # Initialization anchors
        self.initialized = False
        self.initial_odom_pose: Optional[Pose2D] = None
        self.initial_map_pose: Optional[Pose2D] = None
        self.yaw_align = 0.0

        # Persistent correction bias from AprilTag observations.
        self.correction_x = 0.0
        self.correction_y = 0.0
        self.correction_yaw = 0.0

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(TFMessage, self.tf_topic, self.tf_callback, 10)
        self.fused_pub = self.create_publisher(Odometry, self.fused_odom_topic, 10)
        self.registration_status_pub = self.create_publisher(
            String, '~/site_registration', 1)
        self.registration_status_timer = self.create_timer(
            1.0, self.publish_registration_status)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        period = 1.0 / max(self.update_rate_hz, 1.0)
        self.create_timer(period, self.update)

        self.get_logger().info(
            'Tag/Odom fusion v4 landmarks started. '
            f'robot_tag={self.robot_tag_child_frame}, landmarks={list(self.landmarks.keys())}, '
            f'alpha={self.alpha:.3f}, landmark_calib={self.use_landmark_calibration}, '
            f'tag_yaw_offset={self.tag_yaw_offset:.3f}'
        )

    def load_landmarks(self, path: str) -> Dict[str, Dict[str, float]]:
        self.landmark_layout = None
        if not self.use_landmark_calibration:
            self.get_logger().warn(
                'Landmark calibration is explicitly disabled: using MANUAL camera-to-map. '
                'This is not the normal physical-course startup mode.')
            return {}
        try:
            self.landmark_layout = load_landmark_layout(path, self.map_frame)
        except (OSError, ValueError, TypeError) as exc:
            self.get_logger().error(f'SITE REGISTRATION CONFIG ERROR: {exc}')
            raise RuntimeError(f'Invalid landmarks_yaml={path}: {exc}') from exc
        self.get_logger().info(
            f'Surveyed landmark layout: {path} '
            f'(fingerprint={self.landmark_layout.fingerprint}); '
            'Tag 16/17/18 startup lock enabled.')
        return dict(self.landmark_layout.landmarks)

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.current_odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )
        self.current_odom_twist = msg.twist.twist

    def tf_callback(self, msg: TFMessage):
        now = self.get_clock().now()
        for t in msg.transforms:
            if t.header.frame_id != self.tag_parent_frame:
                continue

            if (self.startup_registration is not None
                    and t.child_frame_id in self.landmarks):
                source_ns = (t.header.stamp.sec * 1_000_000_000
                             + t.header.stamp.nanosec)
                self.startup_registration.observe(
                    t.child_frame_id,
                    (t.transform.translation.x * self.camera_x_scale,
                     t.transform.translation.y * self.camera_y_scale),
                    source_ns,
                    now.nanoseconds,
                )
                continue

            q = t.transform.rotation
            raw_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

            # Apply only axis sign fixes here. Rotation/translation to map is handled by Similarity2D.
            raw_x = t.transform.translation.x * self.camera_x_scale
            raw_y = t.transform.translation.y * self.camera_y_scale
            raw_yaw = self.camera_yaw_scale * raw_yaw

            if t.child_frame_id == self.robot_tag_child_frame:
                stamp_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
                # A forwarded TF for the same image is not another observation.
                if stamp_ns and self.last_robot_tag_stamp_ns is not None and stamp_ns <= self.last_robot_tag_stamp_ns:
                    continue
                if stamp_ns:
                    self.last_robot_tag_stamp_ns = stamp_ns
                self.robot_tag_sequence += 1

            self.latest_tag_raw[t.child_frame_id] = (raw_x, raw_y, raw_yaw, now)

            if t.child_frame_id == self.robot_tag_child_frame:
                pose = self.convert_robot_raw_to_map_pose(raw_x, raw_y, raw_yaw)
                self.latest_robot_tag_pose_map = pose
                self.latest_robot_tag_time = now

    def raw_tag_is_fresh(self, child_frame: str, timeout_sec: float) -> bool:
        item = self.latest_tag_raw.get(child_frame)
        if item is None:
            return False
        age = (self.get_clock().now() - item[3]).nanoseconds / 1e9
        return age <= timeout_sec

    def robot_tag_is_fresh(self) -> bool:
        if self.latest_robot_tag_pose_map is None or self.latest_robot_tag_time is None:
            return False
        age = (self.get_clock().now() - self.latest_robot_tag_time).nanoseconds / 1e9
        return age <= self.robot_tag_timeout_sec

    def update_camera_to_map_from_landmarks(self):
        # CRITICAL INVARIANT: a locked camera-to-map is never updated, smoothed,
        # cleared, or invalidated merely because fixed floor tags disappear.
        if not self.use_landmark_calibration or self.landmark_transform_ready:
            return
        status = self.startup_registration.status(self.get_clock().now().nanoseconds)
        if not self.startup_registration.locked:
            self.get_logger().info(
                'SITE REGISTRATION WAIT: '
                f"visible={len(status['visible_frames'])}/3 "
                f"samples={status['samples']}/{status['required_samples']} "
                f"{status['reason']}",
                throttle_duration_sec=2.0,
            )
            return
        self.camera_to_map = self.startup_registration.transform
        self.landmark_transform_ready = True
        t = self.camera_to_map
        self.get_logger().info(
            'SITE REGISTRATION LOCKED: '
            f"samples={status['samples']} span={status['sample_span_sec']:.2f}s "
            f'scale={t.scale:.6f} yaw_deg={math.degrees(t.theta):+.3f} '
            f't=({t.tx:+.4f},{t.ty:+.4f})m '
            f"RMS={status['rms_m']:.4f}m max={status['max_point_error_m']:.4f}m; "
            'floor-tag occlusion is allowed. Stop robots and restart the FULL '
            'Command Center to register again after camera/tag relocation.'
        )

    def publish_registration_status(self):
        if self.startup_registration is None:
            status = {
                'state': 'MANUAL',
                'reason': 'use_landmark_calibration is explicitly false',
            }
        else:
            status = self.startup_registration.status(self.get_clock().now().nanoseconds)
            status['layout_fingerprint'] = self.landmark_layout.fingerprint
        status['map_frame'] = self.map_frame
        status['tag_parent_frame'] = self.tag_parent_frame
        status['transform_input'] = 'camera XY after axis sign/scale corrections'
        status['camera_x_scale'] = self.camera_x_scale
        status['camera_y_scale'] = self.camera_y_scale
        status['camera_yaw_scale'] = self.camera_yaw_scale
        status['fusion_initialized'] = self.initialized
        message = String()
        message.data = json.dumps(status, sort_keys=True, allow_nan=False)
        self.registration_status_pub.publish(message)

    def convert_robot_raw_to_map_pose(self, raw_x: float, raw_y: float, raw_yaw: float) -> Pose2D:
        tag_x, tag_y = self.camera_to_map.apply(raw_x, raw_y)
        tag_yaw = normalize_angle(self.camera_to_map.yaw_apply(raw_yaw) + self.tag_yaw_offset)

        # Convert tag center to base center using robot-frame offset.
        # Tag top edge is assumed to point to the vehicle front.
        base_dx, base_dy = rotate_2d(self.tag_to_base_forward, self.tag_to_base_left, tag_yaw)
        return tag_x + base_dx, tag_y + base_dy, tag_yaw

    def try_initialize(self) -> bool:
        if self.initialized:
            return True
        if self.use_landmark_calibration and not self.landmark_transform_ready:
            return False
        if self.current_odom_pose is None:
            self.get_logger().info('Waiting for /odom...', throttle_duration_sec=1.0)
            return False
        if not self.robot_tag_is_fresh():
            self.get_logger().info('Waiting for fresh robot AprilTag TF...', throttle_duration_sec=1.0)
            return False

        tag_x, tag_y, tag_yaw = self.latest_robot_tag_pose_map
        odom_x, odom_y, odom_yaw = self.current_odom_pose

        if self.initial_yaw_source == 'tag':
            initial_map_yaw = tag_yaw
        else:
            initial_map_yaw = self.fixed_initial_yaw

        self.initial_odom_pose = self.current_odom_pose
        self.initial_map_pose = (tag_x, tag_y, normalize_angle(initial_map_yaw))

        if self.use_initial_yaw_alignment:
            self.yaw_align = normalize_angle(initial_map_yaw - odom_yaw + self.odom_to_map_yaw_offset)
        else:
            self.yaw_align = self.odom_to_map_yaw_offset

        self.correction_x = 0.0
        self.correction_y = 0.0
        self.correction_yaw = 0.0
        self.last_trusted_tag_yaw = tag_yaw
        self.last_corrected_tag_sequence = self.robot_tag_sequence
        self.initialized = True

        self.get_logger().info(
            'Initialized v4 fusion. '
            f'initial_map_pose=({tag_x:.3f}, {tag_y:.3f}, {initial_map_yaw:.3f}), '
            f'initial_odom_pose=({odom_x:.3f}, {odom_y:.3f}, {odom_yaw:.3f}), '
            f'yaw_align={self.yaw_align:.3f}, alpha={self.alpha:.3f}'
        )
        return True

    def predict_from_odom(self) -> Pose2D:
        odom_x, odom_y, odom_yaw = self.current_odom_pose
        init_odom_x, init_odom_y, init_odom_yaw = self.initial_odom_pose
        init_map_x, init_map_y, init_map_yaw = self.initial_map_pose

        odom_dx = (odom_x - init_odom_x) * self.odom_x_scale
        odom_dy = (odom_y - init_odom_y) * self.odom_y_scale
        odom_dyaw = normalize_angle(odom_yaw - init_odom_yaw)

        map_dx, map_dy = rotate_2d(odom_dx, odom_dy, self.yaw_align)

        pred_x = init_map_x + map_dx
        pred_y = init_map_y + map_dy
        pred_yaw = normalize_angle(init_map_yaw + self.odom_yaw_scale * odom_dyaw)
        return pred_x, pred_y, pred_yaw

    def apply_persistent_tag_correction(self, predicted_pose: Pose2D) -> Pose2D:
        pred_x, pred_y, pred_yaw = predicted_pose
        fused_x = pred_x + self.correction_x
        fused_y = pred_y + self.correction_y
        fused_yaw = normalize_angle(pred_yaw + self.correction_yaw)

        if not self.robot_tag_is_fresh() or self.robot_tag_sequence == self.last_corrected_tag_sequence:
            return fused_x, fused_y, fused_yaw

        self.last_corrected_tag_sequence = self.robot_tag_sequence

        tag_x, tag_y, tag_yaw = self.latest_robot_tag_pose_map
        tag_error = math.hypot(tag_x - fused_x, tag_y - fused_y)

        position_accepted = not (self.reject_tag_jump and tag_error > self.max_tag_correction_m)
        if not position_accepted:
            self.get_logger().warn(
                f'Rejected robot-tag correction jump: error={tag_error:.3f} m '
                f'> max_tag_correction_m={self.max_tag_correction_m:.3f} m',
                throttle_duration_sec=1.0,
            )
        elif self.alpha > 0.0:
            self.correction_x += self.alpha * (tag_x - fused_x)
            self.correction_y += self.alpha * (tag_y - fused_y)

        # A position outlier must not block recovery from wheel-induced yaw drift.
        # Check consecutive tag headings to avoid following an isolated bad pose.
        if self.use_tag_yaw_correction and self.tag_yaw_alpha > 0.0 and self.tag_yaw_is_stable(tag_yaw):
            yaw_error = normalize_angle(tag_yaw - fused_yaw)
            step = max(-self.max_yaw_correction_step,
                       min(self.max_yaw_correction_step, self.tag_yaw_alpha * yaw_error))
            self.correction_yaw = normalize_angle(self.correction_yaw + step)

        fused_x = pred_x + self.correction_x
        fused_y = pred_y + self.correction_y
        fused_yaw = normalize_angle(pred_yaw + self.correction_yaw)
        return fused_x, fused_y, fused_yaw

    def tag_yaw_is_stable(self, yaw: float) -> bool:
        if self.last_trusted_tag_yaw is None:
            self.last_trusted_tag_yaw = yaw
            return True
        if abs(normalize_angle(yaw - self.last_trusted_tag_yaw)) > self.max_tag_yaw_reanchor:
            self.yaw_candidate = None
            self.yaw_candidate_count = 0
            self.get_logger().warn(
                'Ignoring AprilTag yaw more than 90 degrees from last trusted yaw',
                throttle_duration_sec=1.0,
            )
            return False
        if abs(normalize_angle(yaw - self.last_trusted_tag_yaw)) <= self.max_tag_yaw_step:
            self.last_trusted_tag_yaw = yaw
            self.yaw_candidate = None
            self.yaw_candidate_count = 0
            return True
        if (self.yaw_candidate is not None and
                abs(normalize_angle(yaw - self.yaw_candidate)) <= self.max_tag_yaw_step):
            self.yaw_candidate_count += 1
        else:
            self.yaw_candidate_count = 1
        self.yaw_candidate = yaw
        if self.yaw_candidate_count >= self.stable_tag_yaw_observations:
            self.last_trusted_tag_yaw = yaw
            self.yaw_candidate = None
            self.yaw_candidate_count = 0
            return True
        self.get_logger().warn(
            'Holding abrupt AprilTag yaw change until consecutive observations agree',
            throttle_duration_sec=1.0,
        )
        return False

    def update(self):
        # Accept the startup lock first, then convert the robot pose using
        # that same immutable transform before any fusion initialization.
        self.update_camera_to_map_from_landmarks()

        robot_raw = self.latest_tag_raw.get(self.robot_tag_child_frame)
        if robot_raw is not None and self.raw_tag_is_fresh(self.robot_tag_child_frame, self.robot_tag_timeout_sec):
            raw_x, raw_y, raw_yaw, t = robot_raw
            self.latest_robot_tag_pose_map = self.convert_robot_raw_to_map_pose(raw_x, raw_y, raw_yaw)
            self.latest_robot_tag_time = t

        if not self.try_initialize():
            return
        if self.current_odom_pose is None:
            return

        predicted_pose = self.predict_from_odom()
        fused_pose = self.apply_persistent_tag_correction(predicted_pose)
        self.publish_fused_odom(fused_pose)

    def publish_fused_odom(self, pose: Pose2D):
        x, y, yaw = pose
        now_msg = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.fused_child_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        if self.current_odom_twist is not None:
            odom.twist.twist = self.current_odom_twist

        self.fused_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.fused_child_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f'fused_pose=({x:.3f}, {y:.3f}, {yaw:.3f}) '
            f'yaw_align={self.yaw_align:.3f} '
            f'corr=({self.correction_x:.3f}, {self.correction_y:.3f}, {self.correction_yaw:.3f}) '
            f'alpha={self.alpha:.3f} robot_tag_fresh={self.robot_tag_is_fresh()} '
            f'landmark_ready={self.landmark_transform_ready}',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TagOdomFusionLandmarksNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
