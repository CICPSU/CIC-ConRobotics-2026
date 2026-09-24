"""Exercise the real fusion methods without requiring a ROS installation."""

import ast
import math
from pathlib import Path
from types import MethodType


def load_correction_methods():
    path = Path(__file__).parents[1] / 'dump_truck_control' / 'tag_odom_fusion_node.py'
    source = ast.parse(path.read_text(encoding='utf-8'))
    klass = next(node for node in source.body if isinstance(node, ast.ClassDef)
                 and node.name == 'TagOdomFusionLandmarksNode')
    methods = [node for node in klass.body if isinstance(node, ast.FunctionDef)
               and node.name in ('apply_persistent_tag_correction', 'tag_yaw_is_stable')]
    namespace = {'math': math, 'Pose2D': tuple}
    normalize = next(node for node in source.body if isinstance(node, ast.FunctionDef)
                     and node.name == 'normalize_angle')
    exec(compile(ast.Module(body=[normalize] + methods, type_ignores=[]),
                 str(path), 'exec'), namespace)
    return namespace


class FakeFusion:
    def __init__(self):
        self.alpha = 0.06
        self.tag_yaw_alpha = 0.08
        self.max_tag_correction_m = 0.35
        self.reject_tag_jump = True
        self.use_tag_yaw_correction = True
        self.max_tag_yaw_step = math.radians(20)
        self.max_tag_yaw_reanchor = math.radians(90)
        self.stable_tag_yaw_observations = 3
        self.max_yaw_correction_step = math.radians(5)
        self.correction_x = self.correction_y = self.correction_yaw = 0.0
        self.robot_tag_sequence = 1
        self.last_corrected_tag_sequence = 0
        self.latest_robot_tag_pose_map = (0.1, 0.0, 0.0)
        self.last_trusted_tag_yaw = 0.0
        self.yaw_candidate = None
        self.yaw_candidate_count = 0
        self.fresh = True
        self.messages = []
        methods = load_correction_methods()
        self.apply_persistent_tag_correction = MethodType(
            methods['apply_persistent_tag_correction'], self)
        self.tag_yaw_is_stable = MethodType(methods['tag_yaw_is_stable'], self)

    def robot_tag_is_fresh(self):
        return self.fresh

    def get_logger(self):
        return self

    def warn(self, msg, **kwargs):
        self.messages.append(msg)


def test_one_correction_per_observation_and_no_stale_correction():
    fusion = FakeFusion()
    pose = fusion.apply_persistent_tag_correction((0.0, 0.0, 0.0))
    assert math.isclose(pose[0], 0.006)
    assert fusion.apply_persistent_tag_correction((0.0, 0.0, 0.0)) == pose
    fusion.robot_tag_sequence += 1
    fusion.fresh = False
    assert fusion.apply_persistent_tag_correction((0.0, 0.0, 0.0)) == pose


def test_yaw_recovers_when_position_jump_is_rejected():
    fusion = FakeFusion()
    fusion.latest_robot_tag_pose_map = (1.0, 0.0, 0.0)
    pose = fusion.apply_persistent_tag_correction((0.0, 0.0, math.radians(45)))
    assert pose[0] == 0.0
    assert math.degrees(pose[2]) < 45.0
    assert fusion.correction_yaw < 0.0


def test_isolated_yaw_flip_is_ignored_then_stable_yaw_is_accepted():
    fusion = FakeFusion()
    assert not fusion.tag_yaw_is_stable(math.radians(170))
    assert fusion.tag_yaw_is_stable(math.radians(1))
    for candidate in (38, 39):
        assert not fusion.tag_yaw_is_stable(math.radians(candidate))
    assert fusion.tag_yaw_is_stable(math.radians(39))


def test_yaw_does_not_follow_persistent_half_turn_tag_flip():
    fusion = FakeFusion()
    for _ in range(4):
        assert not fusion.tag_yaw_is_stable(math.radians(179))
    assert fusion.last_trusted_tag_yaw == 0.0
