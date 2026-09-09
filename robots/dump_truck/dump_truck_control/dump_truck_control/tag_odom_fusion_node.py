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
an updated 2D transform:

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
1. The node estimates camera_to_map from visible landmarks.
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

import math
from typing import Dict, Optional, Tuple, List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_ros

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


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


class Similarity2D:
    """2D similarity transform: map = scale * R(theta) * camera + translation."""

    def __init__(self, scale: float = 1.0, theta: float = 0.0, tx: float = 0.0, ty: float = 0.0):
        self.scale = scale
        self.theta = theta
        self.tx = tx
        self.ty = ty

    def apply(self, x: float, y: float) -> Point2D:
        rx, ry = rotate_2d(x, y, self.theta)
        return self.scale * rx + self.tx, self.scale * ry + self.ty

    def yaw_apply(self, yaw: float) -> float:
        return normalize_angle(self.theta + yaw)


def estimate_similarity_2d(camera_pts: List[Point2D], map_pts: List[Point2D], allow_scale: bool) -> Optional[Similarity2D]:
    """
    Estimate best-fit 2D similarity transform from camera_pts to map_pts.

    Uses a lightweight closed-form Procrustes solution without numpy.
    Needs at least 2 non-identical point pairs. Three landmarks are recommended.
    """
    n = min(len(camera_pts), len(map_pts))
    if n < 2:
        return None

    cx = sum(p[0] for p in camera_pts) / n
    cy = sum(p[1] for p in camera_pts) / n
    mx = sum(p[0] for p in map_pts) / n
    my = sum(p[1] for p in map_pts) / n

    a = 0.0  # sum ux*vx + uy*vy
    b = 0.0  # sum ux*vy - uy*vx
    denom = 0.0
    for (px, py), (qx, qy) in zip(camera_pts, map_pts):
        ux = px - cx
        uy = py - cy
        vx = qx - mx
        vy = qy - my
        a += ux * vx + uy * vy
        b += ux * vy - uy * vx
        denom += ux * ux + uy * uy

    if denom < 1e-9:
        return None

    theta = math.atan2(b, a)
    if allow_scale:
        scale = math.sqrt(a * a + b * b) / denom
    else:
        scale = 1.0

    rcx, rcy = rotate_2d(cx, cy, theta)
    tx = mx - scale * rcx
    ty = my - scale * rcy
    return Similarity2D(scale, theta, tx, ty)


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

        # Landmark calibration
        self.declare_parameter('landmarks_yaml', '')
        self.declare_parameter('use_landmark_calibration', True)
        self.declare_parameter('allow_landmark_scale', True)
        self.declare_parameter('min_landmarks_for_update', 2)
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

        self.odom_x_scale = float(self.get_parameter('odom_x_scale').value)
        self.odom_y_scale = float(self.get_parameter('odom_y_scale').value)
        self.odom_yaw_scale = float(self.get_parameter('odom_yaw_scale').value)
        self.odom_to_map_yaw_offset = float(self.get_parameter('odom_to_map_yaw_offset').value)
        self.use_initial_yaw_alignment = bool(self.get_parameter('use_initial_yaw_alignment').value)
        self.initial_yaw_source = str(self.get_parameter('initial_yaw_source').value).lower()
        self.fixed_initial_yaw = float(self.get_parameter('fixed_initial_yaw').value)

        # Landmark definitions: child_frame -> map pose/point
        self.landmarks: Dict[str, Dict[str, float]] = self.load_landmarks(self.landmarks_yaml)

        # Live TF cache: child_frame -> (raw_x, raw_y, raw_yaw, time)
        self.latest_tag_raw: Dict[str, Tuple[float, float, float, rclpy.time.Time]] = {}

        # Current camera-to-map transform. Initial fallback uses manual transform.
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
        if not path:
            self.get_logger().warn('No landmarks_yaml provided. Landmark calibration will not update.')
            return {}
        if yaml is None:
            self.get_logger().error('PyYAML is not installed. Install with: sudo apt install python3-yaml')
            return {}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read landmarks_yaml={path}: {exc}')
            return {}

        raw = data.get('landmarks', data)
        result: Dict[str, Dict[str, float]] = {}
        for key, val in raw.items():
            if val is None:
                continue
            frame = str(val.get('frame', f'tag36h11_{key}'))
            result[frame] = {
                'x': float(val['x']),
                'y': float(val['y']),
                'yaw': float(val.get('yaw', 0.0)),
            }
        return result

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

            q = t.transform.rotation
            raw_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

            # Apply only axis sign fixes here. Rotation/translation to map is handled by Similarity2D.
            raw_x = t.transform.translation.x * self.camera_x_scale
            raw_y = t.transform.translation.y * self.camera_y_scale
            raw_yaw = self.camera_yaw_scale * raw_yaw

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
        if not self.use_landmark_calibration or not self.landmarks:
            return

        camera_pts: List[Point2D] = []
        map_pts: List[Point2D] = []
        landmark_yaw_errors: List[float] = []

        for frame, known in self.landmarks.items():
            if not self.raw_tag_is_fresh(frame, self.landmark_timeout_sec):
                continue
            raw_x, raw_y, raw_yaw, _ = self.latest_tag_raw[frame]
            camera_pts.append((raw_x, raw_y))
            map_pts.append((known['x'], known['y']))
            landmark_yaw_errors.append(normalize_angle(known.get('yaw', 0.0) - raw_yaw))

        if len(camera_pts) < max(2, self.min_landmarks_for_update):
            self.get_logger().info(
                f'Waiting for landmarks: visible={len(camera_pts)}, needed={max(2, self.min_landmarks_for_update)}',
                throttle_duration_sec=1.0,
            )
            return

        estimated = estimate_similarity_2d(camera_pts, map_pts, self.allow_landmark_scale)
        if estimated is None:
            self.get_logger().warn('Could not estimate camera_to_map from landmarks.', throttle_duration_sec=1.0)
            return

        # Position-based transform is the main source. If landmark yaw is reliable, it should be close.
        # We do not force theta from yaw because AprilTag yaw can have convention offsets.
        a = max(0.0, min(1.0, self.landmark_transform_alpha))
        if not self.landmark_transform_ready:
            self.camera_to_map = estimated
            self.landmark_transform_ready = True
        else:
            # Smooth scale and translation linearly; smooth angle circularly.
            old = self.camera_to_map
            new_theta = normalize_angle(old.theta + a * normalize_angle(estimated.theta - old.theta))
            self.camera_to_map = Similarity2D(
                scale=(1.0 - a) * old.scale + a * estimated.scale,
                theta=new_theta,
                tx=(1.0 - a) * old.tx + a * estimated.tx,
                ty=(1.0 - a) * old.ty + a * estimated.ty,
            )

        if self.publish_landmark_debug:
            # Residual debug
            errs = []
            for cp, mp in zip(camera_pts, map_pts):
                px, py = self.camera_to_map.apply(cp[0], cp[1])
                errs.append(math.hypot(px - mp[0], py - mp[1]))
            rms = math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else 0.0
            yaw_hint = mean_angle(landmark_yaw_errors)
            self.get_logger().info(
                f'landmark camera_to_map: visible={len(camera_pts)} '
                f'scale={self.camera_to_map.scale:.4f} theta={self.camera_to_map.theta:.3f} '
                f't=({self.camera_to_map.tx:.3f},{self.camera_to_map.ty:.3f}) '
                f'rms={rms:.3f} yaw_hint={yaw_hint:.3f}',
                throttle_duration_sec=1.0,
            )

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
        if self.current_odom_pose is None:
            self.get_logger().info('Waiting for /odom...', throttle_duration_sec=1.0)
            return False
        if not self.robot_tag_is_fresh():
            self.get_logger().info('Waiting for fresh robot AprilTag TF...', throttle_duration_sec=1.0)
            return False
        if self.use_landmark_calibration and self.landmarks and not self.landmark_transform_ready:
            self.get_logger().info('Waiting for landmark-based camera_to_map transform...', throttle_duration_sec=1.0)
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

        if self.alpha <= 0.0 or not self.robot_tag_is_fresh():
            return fused_x, fused_y, fused_yaw

        tag_x, tag_y, tag_yaw = self.latest_robot_tag_pose_map
        tag_error = math.hypot(tag_x - fused_x, tag_y - fused_y)

        if self.reject_tag_jump and tag_error > self.max_tag_correction_m:
            self.get_logger().warn(
                f'Rejected robot-tag correction jump: error={tag_error:.3f} m '
                f'> max_tag_correction_m={self.max_tag_correction_m:.3f} m',
                throttle_duration_sec=1.0,
            )
            return fused_x, fused_y, fused_yaw

        self.correction_x += self.alpha * (tag_x - fused_x)
        self.correction_y += self.alpha * (tag_y - fused_y)

        if self.use_tag_yaw_correction:
            yaw_error = normalize_angle(tag_yaw - fused_yaw)
            self.correction_yaw = normalize_angle(self.correction_yaw + self.alpha * yaw_error)

        fused_x = pred_x + self.correction_x
        fused_y = pred_y + self.correction_y
        fused_yaw = normalize_angle(pred_yaw + self.correction_yaw)
        return fused_x, fused_y, fused_yaw

    def update(self):
        # Update camera->map from landmarks first, then refresh robot tag pose if raw tag exists.
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
