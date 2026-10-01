"""ROS-independent startup-only registration of three fixed floor AprilTags.

Coordinates here are the EXISTING fusion convention: camera x/y after its axis
sign corrections. This is a 2-D similarity, not a publishable rigid 3-D TF.
Once locked, the transform is immutable for this object's entire lifetime.
"""

from collections import deque
from dataclasses import dataclass, fields
import hashlib
import json
import math
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import yaml


Point2D = Tuple[float, float]
REQUIRED_FRAMES = ('tag36h11_16', 'tag36h11_17', 'tag36h11_18')


def _angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Similarity2D:
    """Map point = scale * R(theta) * corrected-camera point + translation."""

    scale: float = 1.0
    theta: float = 0.0
    tx: float = 0.0
    ty: float = 0.0

    def apply(self, x: float, y: float) -> Point2D:
        c, s = math.cos(self.theta), math.sin(self.theta)
        return (self.scale * (c * x - s * y) + self.tx,
                self.scale * (s * x + c * y) + self.ty)

    def yaw_apply(self, yaw: float) -> float:
        return _angle(self.theta + yaw)


def estimate_similarity_2d(
    camera_pts: Sequence[Point2D],
    map_pts: Sequence[Point2D],
    allow_scale: bool,
) -> Optional[Similarity2D]:
    """Use the same closed-form least-squares model as the existing fusion."""
    if len(camera_pts) != len(map_pts) or len(camera_pts) < 2:
        return None
    if not all(math.isfinite(v) for p in (*camera_pts, *map_pts) for v in p):
        return None
    n = len(camera_pts)
    cx, cy = (sum(p[k] for p in camera_pts) / n for k in (0, 1))
    mx, my = (sum(p[k] for p in map_pts) / n for k in (0, 1))
    a = b = denom = 0.0
    for (px, py), (qx, qy) in zip(camera_pts, map_pts):
        ux, uy, vx, vy = px - cx, py - cy, qx - mx, qy - my
        a += ux * vx + uy * vy
        b += ux * vy - uy * vx
        denom += ux * ux + uy * uy
    if denom < 1e-9 or math.hypot(a, b) < 1e-12:
        return None
    theta = math.atan2(b, a)
    scale = math.hypot(a, b) / denom if allow_scale else 1.0
    c, s = math.cos(theta), math.sin(theta)
    result = Similarity2D(scale, theta, mx - scale * (c * cx - s * cy),
                          my - scale * (s * cx + c * cy))
    return result if all(math.isfinite(v) for v in (
        result.scale, result.theta, result.tx, result.ty)) else None


def _point_errors(transform, camera_pts, map_pts):
    return [math.dist(transform.apply(*cp), mp)
            for cp, mp in zip(camera_pts, map_pts)]


def _geometry_is_valid(points: Sequence[Point2D]) -> bool:
    """Reject coincident/very close centers and an almost straight triangle."""
    if len(points) != 3:
        return False
    distances = [math.dist(points[i], points[j])
                 for i, j in ((0, 1), (0, 2), (1, 2))]
    if min(distances) < 0.10:
        return False
    (ax, ay), (bx, by), (cx, cy) = points
    twice_area = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
    # Dimensionless; independent of the allowed similarity scale.
    return twice_area / (max(distances) ** 2) >= 0.02


@dataclass(frozen=True)
class RegistrationConfig:
    """Initial engineering limits, to be validated on both physical cameras."""

    min_samples: int = 20
    min_duration_sec: float = 1.0
    max_sample_gap_sec: float = 0.5
    max_rms_m: float = 0.02
    max_point_error_m: float = 0.03
    max_position_deviation_m: float = 0.02
    max_yaw_deviation_deg: float = 1.0
    max_scale_deviation: float = 0.01
    min_scale: float = 0.90
    max_scale: float = 1.10

    def __post_init__(self):
        if type(self.min_samples) is not int or not 3 <= self.min_samples <= 1000:
            raise ValueError('registration.min_samples must be an integer in [3, 1000]')
        for f in fields(self):
            if f.name == 'min_samples':
                continue
            value = getattr(self, f.name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0.0):
                raise ValueError(f'registration.{f.name} must be a positive finite number')
        if self.min_scale > 1.0 or self.max_scale < 1.0:
            raise ValueError('registration scale limits must include 1.0')
        if self.min_scale >= self.max_scale:
            raise ValueError('registration.min_scale must be below max_scale')
        if self.max_rms_m > self.max_point_error_m:
            raise ValueError('registration.max_rms_m must not exceed max_point_error_m')
        if self.max_yaw_deviation_deg >= 90.0:
            raise ValueError('registration.max_yaw_deviation_deg must be below 90')


class _UniqueKeyLoader(yaml.SafeLoader):
    """Do not silently accept overwritten IDs, coordinates, or settings."""


def _unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f'Duplicate YAML key: {key!r}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


@dataclass(frozen=True)
class LandmarkLayout:
    landmarks: Mapping[str, Mapping[str, float]]
    registration: RegistrationConfig
    fingerprint: str


def load_landmark_layout(path: str, expected_map_frame: str = 'map') -> LandmarkLayout:
    """Require explicit confirmation of surveyed centers; fail closed otherwise."""
    if not path:
        raise ValueError('landmarks_yaml is required when landmark calibration is enabled')
    try:
        data = yaml.load(Path(path).read_text(encoding='utf-8'), Loader=_UniqueKeyLoader)
    except yaml.YAMLError as exc:
        raise ValueError(f'Invalid landmarks YAML syntax: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError('landmarks.yaml must be a mapping')
    allowed_root = {'layout_confirmed', 'map_frame', 'landmarks', 'registration'}
    if set(data) - allowed_root:
        raise ValueError(f'Unknown landmarks.yaml fields: {sorted(set(data) - allowed_root)}')
    if data.get('layout_confirmed') is not True:
        raise ValueError(
            'SITE LAYOUT NOT CONFIRMED: measure Tag 16/17/18 CENTER coordinates '
            'in the existing map frame, update landmarks.yaml, and only then '
            'set layout_confirmed: true. Do not reuse old coordinates after relocation.')
    if data.get('map_frame') != expected_map_frame:
        raise ValueError(f'landmarks.yaml map_frame must equal {expected_map_frame!r}')
    raw = data.get('landmarks')
    if not isinstance(raw, dict):
        raise ValueError('landmarks.yaml requires a landmarks mapping')
    result = {}
    for key, entry in raw.items():
        if not isinstance(entry, dict):
            raise ValueError(f'Invalid landmark entry {key!r}')
        if set(entry) - {'frame', 'x', 'y', 'yaw'}:
            raise ValueError(f'Unknown fields for landmark {key!r}')
        frame = entry.get('frame', f'tag36h11_{key}')
        if not isinstance(frame, str) or frame not in REQUIRED_FRAMES or frame in result:
            raise ValueError(f'Unexpected or duplicate landmark frame {frame!r}')
        values = {}
        for name in ('x', 'y', 'yaw'):
            value = entry.get(name, 0.0 if name == 'yaw' else None)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                raise ValueError(f'{frame}.{name} must be a finite number (meters/radians)')
            values[name] = float(value)
        result[frame] = values
    if set(result) != set(REQUIRED_FRAMES):
        raise ValueError('Exactly Tag 16, Tag 17, and Tag 18 must be configured')
    if not _geometry_is_valid([(result[f]['x'], result[f]['y']) for f in REQUIRED_FRAMES]):
        raise ValueError('Landmark centers must form a well-spread non-collinear triangle')
    raw_config = data.get('registration', {})
    if not isinstance(raw_config, dict):
        raise ValueError('registration must be a mapping')
    unknown = set(raw_config) - {f.name for f in fields(RegistrationConfig)}
    if unknown:
        raise ValueError(f'Unknown registration settings: {sorted(unknown)}')
    config = RegistrationConfig(**raw_config)
    identity = {'map_frame': expected_map_frame, 'landmarks': result}
    digest = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()[:12]
    return LandmarkLayout(result, config, digest)


@dataclass(frozen=True)
class _Sample:
    stamp_ns: int
    points: Tuple[Point2D, ...]
    transform: Similarity2D


class StartupSiteRegistration:
    """Collect fresh complete images, validate stability, then lock permanently.

    Only observations with the SAME source header stamp form one three-point
    sample. Repeated TF delivery is not another observation. No services or
    live parameter changes can unlock the resulting transform.
    """

    def __init__(self, landmarks, config=None, *, allow_scale=True,
                 timeout_sec=1.0, not_before_ns=0):
        self.config = config or RegistrationConfig()
        if set(landmarks) != set(REQUIRED_FRAMES):
            raise ValueError('Startup registration requires exactly Tag 16/17/18')
        self.map_points = tuple((float(landmarks[f]['x']), float(landmarks[f]['y']))
                                for f in REQUIRED_FRAMES)
        if (not all(math.isfinite(v) for p in self.map_points for v in p)
                or not _geometry_is_valid(self.map_points)):
            raise ValueError('Invalid landmark geometry')
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('landmark_timeout_sec must be positive and finite')
        self.allow_scale = bool(allow_scale)
        self.timeout_ns = int(timeout_sec * 1e9)
        self.not_before_ns = int(not_before_ns)
        self._locked = None
        self._pending: Dict[int, Dict[str, Point2D]] = {}
        self._latest_stamp: Dict[str, int] = {}
        self._last_consumed_stamp = -1
        self._samples = deque()
        self._last_now_ns = None
        self.reason = 'Waiting for three fresh landmarks in the same image'
        self.rms_m = None
        self.max_error_m = None
        self.position_deviation_m = None
        self.yaw_deviation_deg = None
        self.scale_deviation = None

    @property
    def transform(self) -> Optional[Similarity2D]:
        return self._locked

    @property
    def locked(self) -> bool:
        return self._locked is not None

    def _reset_samples(self, reason):
        self._samples.clear()
        self.reason = reason

    def tick(self, now_ns: int):
        """Expire unfinished STARTUP observations; never expire a locked transform."""
        if self.locked:
            return
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            # A ROS/simulation clock reset invalidates the unfinished window.
            self._pending.clear()
            self._latest_stamp.clear()
            self._last_consumed_stamp = -1
            self.not_before_ns = now_ns
            self._reset_samples('Clock moved backwards; restarting startup registration')
        self._last_now_ns = now_ns
        self._pending = {s: pts for s, pts in self._pending.items()
                         if -50_000_000 <= now_ns - s <= self.timeout_ns}
        if self._samples:
            last_stamp = self._samples[-1].stamp_ns
            # Allow source latency as well as the permitted inter-image gap.
            if now_ns - last_stamp > self.timeout_ns + int(self.config.max_sample_gap_sec * 1e9):
                self._reset_samples('Fresh complete landmark images stopped; waiting again')

    def observe(self, frame: str, point: Point2D, stamp_ns: int, now_ns: int):
        """Accept one floor-tag center. Other tags do not affect registration."""
        if self.locked or frame not in REQUIRED_FRAMES:
            return
        self.tick(now_ns)
        if stamp_ns <= 0 or stamp_ns < self.not_before_ns:
            self.reason = 'Ignoring unstamped/pre-start landmark TF'
            return
        age = now_ns - stamp_ns
        if age < -50_000_000 or age > self.timeout_ns:
            self.reason = 'Ignoring stale/future landmark TF; check source timestamps'
            return
        if stamp_ns <= self._last_consumed_stamp:
            return
        # Ignore per-tag duplicate and out-of-order packets without refreshing age.
        if stamp_ns <= self._latest_stamp.get(frame, -1):
            return
        self._latest_stamp[frame] = stamp_ns
        if len(point) != 2 or not all(math.isfinite(v) for v in point):
            self._pending.pop(stamp_ns, None)
            self._reset_samples(f'Invalid center for {frame}; startup sample rejected')
            return
        self._pending.setdefault(stamp_ns, {})[frame] = tuple(point)
        # Bound memory if a tag never appears (up to 64 incomplete image sets).
        for old_stamp in sorted(self._pending)[:-64]:
            del self._pending[old_stamp]
        points = self._pending.get(stamp_ns, {})
        if set(points) != set(REQUIRED_FRAMES):
            return
        camera_points = tuple(points[f] for f in REQUIRED_FRAMES)
        self._last_consumed_stamp = stamp_ns
        self._pending = {s: pts for s, pts in self._pending.items() if s > stamp_ns}
        self._accept_complete_image(camera_points, stamp_ns)

    def _accept_complete_image(self, camera_points, stamp_ns):
        config = self.config
        if not _geometry_is_valid(camera_points):
            self._reset_samples('Observed landmark geometry is degenerate')
            return
        candidate = estimate_similarity_2d(camera_points, self.map_points, self.allow_scale)
        if candidate is None or not config.min_scale <= candidate.scale <= config.max_scale:
            self._reset_samples('Scale/fit rejected; check tag sizes, intrinsics, and survey')
            return
        errors = _point_errors(candidate, camera_points, self.map_points)
        rms = math.sqrt(sum(e * e for e in errors) / len(errors))
        if rms > config.max_rms_m or max(errors) > config.max_point_error_m:
            self._reset_samples(
                f'Landmark fit rejected: RMS={rms:.4f} m, max={max(errors):.4f} m')
            return
        if self._samples and stamp_ns - self._samples[-1].stamp_ns > int(config.max_sample_gap_sec * 1e9):
            self._reset_samples('Complete-image gap exceeded; collecting a new stable window')
        sample = _Sample(stamp_ns, camera_points, candidate)
        self._samples.append(sample)
        # Keep a rolling window spanning at least min_duration and min_samples.
        cutoff = stamp_ns - int(config.min_duration_sec * 1e9)
        while len(self._samples) > config.min_samples and self._samples[1].stamp_ns <= cutoff:
            self._samples.popleft()
        if len(self._samples) > 10000:
            self._reset_samples('Excessive sample rate/window length; check registration settings')
            return
        transforms = [s.transform for s in self._samples]
        n = len(transforms)
        mean = Similarity2D(
            sum(t.scale for t in transforms) / n,
            math.atan2(sum(math.sin(t.theta) for t in transforms),
                       sum(math.cos(t.theta) for t in transforms)),
            sum(t.tx for t in transforms) / n,
            sum(t.ty for t in transforms) / n,
        )
        self.position_deviation_m = max(math.hypot(t.tx - mean.tx, t.ty - mean.ty)
                                        for t in transforms)
        self.yaw_deviation_deg = max(abs(math.degrees(_angle(t.theta - mean.theta)))
                                    for t in transforms)
        self.scale_deviation = max(abs(t.scale - mean.scale) for t in transforms)
        if (self.position_deviation_m > config.max_position_deviation_m
                or self.yaw_deviation_deg > config.max_yaw_deviation_deg
                or self.scale_deviation > config.max_scale_deviation):
            self._reset_samples('Transform not stable; starting a new sample window')
            self._samples.append(sample)
            return
        elapsed = (self._samples[-1].stamp_ns - self._samples[0].stamp_ns) / 1e9
        self.reason = (f'Collecting stable images: {n}/{config.min_samples}, '
                       f'{elapsed:.2f}/{config.min_duration_sec:.2f} s')
        if n < config.min_samples or elapsed < config.min_duration_sec:
            return
        # Validate the FINAL mean against every observed landmark, not just a
        # single image or averaged points whose errors could cancel out.
        all_errors = [e for s in self._samples
                      for e in _point_errors(mean, s.points, self.map_points)]
        self.rms_m = math.sqrt(sum(e * e for e in all_errors) / len(all_errors))
        self.max_error_m = max(all_errors)
        if self.rms_m > config.max_rms_m or self.max_error_m > config.max_point_error_m:
            self._reset_samples('Final averaged transform failed residual validation')
            self._samples.append(sample)
            return
        self._locked = mean
        self._pending.clear()
        self.reason = 'LOCKED for this node lifetime; floor-tag visibility is no longer required'

    def status(self, now_ns: int) -> dict:
        """Return JSON-safe diagnostics. LOCKED does not claim current visibility."""
        self.tick(now_ns)
        visible = [f for f in REQUIRED_FRAMES if f in self._latest_stamp
                   and -50_000_000 <= now_ns - self._latest_stamp[f] <= self.timeout_ns]
        t = self.transform
        return {
            'state': 'LOCKED' if self.locked else 'WAITING_FOR_LANDMARKS',
            'reason': self.reason,
            'required_frames': list(REQUIRED_FRAMES),
            'visible_frames': None if self.locked else visible,
            'samples': len(self._samples),
            'required_samples': self.config.min_samples,
            'sample_span_sec': ((self._samples[-1].stamp_ns - self._samples[0].stamp_ns) / 1e9
                                if self._samples else 0.0),
            'rms_m': self.rms_m,
            'max_point_error_m': self.max_error_m,
            'position_deviation_m': self.position_deviation_m,
            'yaw_deviation_deg': self.yaw_deviation_deg,
            'scale_deviation': self.scale_deviation,
            'camera_to_map': None if t is None else {
                'scale': t.scale, 'theta_rad': t.theta, 'yaw_deg': math.degrees(t.theta),
                'tx_m': t.tx, 'ty_m': t.ty,
            },
        }


def main(argv=None):
    """Validate a survey file without starting ROS, a camera, or any robot."""
    import argparse
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('landmarks_yaml')
    parser.add_argument('--map-frame', default='map')
    args = parser.parse_args(argv)
    try:
        layout = load_landmark_layout(args.landmarks_yaml, args.map_frame)
    except (OSError, ValueError, TypeError) as exc:
        parser.exit(2, f'SITE REGISTRATION CONFIG ERROR: {exc}\n')
    print(json.dumps({
        'layout_confirmed': True,
        'map_frame': args.map_frame,
        'layout_fingerprint': layout.fingerprint,
        'landmarks': layout.landmarks,
        'registration': {f.name: getattr(layout.registration, f.name)
                         for f in fields(RegistrationConfig)},
    }, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
