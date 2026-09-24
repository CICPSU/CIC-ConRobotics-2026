#!/usr/bin/env python3

import math
import os
import threading
import time

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from ament_index_python.packages import get_package_share_directory

from action_msgs.msg import GoalStatus
from construction_site_interfaces.action import ExecuteRobotTask
from construction_site_interfaces.msg import RobotStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


EXCAVATOR_JOINT_NAMES = {
    'swing': 'swing_joint',
    'boom': 'boom_joint',
    'arm': 'arm_joint',
    'bucket': 'bucket_joint',
}

def get_excavator_action_name(
    robot_name,
):
    robot_name = str(
        robot_name
    ).strip().strip('/')

    if not robot_name:
        raise RuntimeError(
            'Excavator robot name cannot be empty.'
        )

    return (
        f'/{robot_name}/'
        'upper_arm_controller/'
        'follow_joint_trajectory'
    )


def resolve_scenario_yaml(scenario_name):
    """
    Resolve scenario YAML.

    Search order:
    1. Explicit filesystem path.
    2. Repository operations/scenarios directory.
    3. Installed construction_site_control
       share/scenarios directory (legacy fallback).
    """

    if not scenario_name:
        return None

    explicit_path = os.path.abspath(
        os.path.expanduser(scenario_name)
    )

    if os.path.isfile(explicit_path):
        return explicit_path

    search_dir = os.path.dirname(
        os.path.realpath(__file__)
    )

    while True:
        candidate = os.path.join(
            search_dir,
            'operations',
            'scenarios',
            scenario_name,
        )

        if os.path.isfile(candidate):
            return candidate

        parent = os.path.dirname(
            search_dir
        )

        if parent == search_dir:
            break

        search_dir = parent

    try:
        package_share = (
            get_package_share_directory(
                'construction_site_control'
            )
        )

        installed_path = os.path.join(
            package_share,
            'scenarios',
            scenario_name,
        )

        if os.path.isfile(installed_path):
            return installed_path

    except Exception:
        pass

    return None


def resolve_excavator_trajectory_yaml(
    trajectory_name,
):
    """
    Resolve excavator trajectory YAML.

    Search order:
    1. Explicit filesystem path.
    2. Repository operations/excavator/trajectories.
    """

    if not trajectory_name:
        return None

    explicit_path = os.path.abspath(
        os.path.expanduser(
            trajectory_name
        )
    )

    if os.path.isfile(
        explicit_path
    ):
        return explicit_path

    search_dir = os.path.dirname(
        os.path.realpath(__file__)
    )

    while True:
        candidate = os.path.join(
            search_dir,
            'operations',
            'excavator',
            'trajectories',
            trajectory_name,
        )

        if os.path.isfile(
            candidate
        ):
            return candidate

        parent = os.path.dirname(
            search_dir
        )

        if parent == search_dir:
            break

        search_dir = parent

    return None


def load_excavator_trajectory(
    trajectory_path,
):
    """
    Load and validate the structural content of an
    excavator trajectory YAML.

    Machine-specific joint-limit validation remains
    the responsibility of the excavator Action server.
    """

    with open(
        trajectory_path,
        'r',
        encoding='utf-8',
    ) as stream:
        data = yaml.safe_load(
            stream
        ) or {}

    trajectory_name = str(
        data.get(
            'trajectory_name',
            os.path.basename(
                trajectory_path
            ),
        )
    ).strip()

    joints = data.get(
        'joints',
        [],
    )

    waypoints = data.get(
        'waypoints',
        [],
    )

    if not isinstance(
        joints,
        list,
    ):

        raise RuntimeError(
            'Excavator trajectory "joints" '
            'must be a list.'
        )

    if not joints:

        raise RuntimeError(
            'Excavator trajectory contains '
            'no joints.'
        )

    normalized_joints = []

    for joint in joints:

        joint_name = str(
            joint
        ).strip()

        if (
            joint_name
            not in EXCAVATOR_JOINT_NAMES
        ):

            raise RuntimeError(
                'Unknown excavator joint '
                f'"{joint_name}".'
            )

        if (
            joint_name
            in normalized_joints
        ):

            raise RuntimeError(
                'Duplicate excavator joint '
                f'"{joint_name}".'
            )

        normalized_joints.append(
            joint_name
        )

    if not isinstance(
        waypoints,
        list,
    ):

        raise RuntimeError(
            'Excavator trajectory "waypoints" '
            'must be a list.'
        )

    if not waypoints:

        raise RuntimeError(
            'Excavator trajectory contains '
            'no waypoints.'
        )

    normalized_waypoints = []
    waypoint_names = set()

    for (
        index,
        waypoint,
    ) in enumerate(
        waypoints
    ):

        if not isinstance(
            waypoint,
            dict,
        ):

            raise RuntimeError(
                f'Excavator waypoint {index + 1} '
                'must be a mapping.'
            )

        waypoint_name = str(
            waypoint.get(
                'name',
                f'waypoint_{index + 1}',
            )
        ).strip()

        if not waypoint_name:

            raise RuntimeError(
                f'Excavator waypoint {index + 1} '
                'has an empty name.'
            )

        if (
            waypoint_name
            in waypoint_names
        ):

            raise RuntimeError(
                'Duplicate excavator waypoint '
                f'name "{waypoint_name}".'
            )

        waypoint_names.add(
            waypoint_name
        )

        positions = waypoint.get(
            'positions',
            {},
        )

        if not isinstance(
            positions,
            dict,
        ):

            raise RuntimeError(
                f'Waypoint "{waypoint_name}" '
                '"positions" must be a mapping.'
            )

        missing = (
            set(normalized_joints)
            - set(positions.keys())
        )

        if missing:

            raise RuntimeError(
                f'Waypoint "{waypoint_name}" '
                'is missing positions for '
                f'{sorted(missing)}.'
            )

        extra = (
            set(positions.keys())
            - set(normalized_joints)
        )

        if extra:

            raise RuntimeError(
                f'Waypoint "{waypoint_name}" '
                'contains positions for joints '
                'not listed in "joints": '
                f'{sorted(extra)}.'
            )

        position_values = []

        for joint_name in (
            normalized_joints
        ):

            try:
                value = float(
                    positions[
                        joint_name
                    ]
                )

            except (
                TypeError,
                ValueError,
            ) as exc:

                raise RuntimeError(
                    f'Waypoint "{waypoint_name}" '
                    f'joint "{joint_name}" '
                    'must be numeric.'
                ) from exc

            if not math.isfinite(
                value
            ):

                raise RuntimeError(
                    f'Waypoint "{waypoint_name}" '
                    f'joint "{joint_name}" '
                    'must be finite.'
                )

            position_values.append(
                value
            )

        direction = waypoint.get('swing_direction')
        if 'swing' in normalized_joints:
            if type(direction) is not int or direction not in (-1, 1):
                raise RuntimeError(
                    f'Waypoint "{waypoint_name}" requires swing_direction: +1 or -1.'
                )
        elif 'swing_direction' in waypoint:
            raise RuntimeError(
                f'Waypoint "{waypoint_name}" specifies swing_direction without swing.'
            )

        normalized_waypoints.append(
            {
                'name': waypoint_name,
                'positions_deg': (
                    position_values
                ),
                'swing_direction': direction,
            }
        )

    return {
        'trajectory_name': (
            trajectory_name
        ),
        'joints': normalized_joints,
        'waypoints': (
            normalized_waypoints
        ),
    }


def seconds_to_duration(
    seconds,
):

    sec = int(
        seconds
    )

    nanosec = int(
        round(
            (
                seconds
                - sec
            )
            * 1_000_000_000
        )
    )

    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000

    return sec, nanosec


class ScenarioManager(Node):

    def __init__(self):

        super().__init__(
            'scenario_manager'
        )

        # -----------------------------------------------------
        # Parameters
        # -----------------------------------------------------

        self.declare_parameter(
            'scenario',
            'truck1_then_truck3.yaml',
        )

        self.declare_parameter(
            'trucks',
            '',
        )

        self.scenario_name = str(
            self.get_parameter(
                'scenario'
            ).value
        ).strip()

        self.trucks_raw = str(
            self.get_parameter(
                'trucks'
            ).value
        ).strip()

        self.allowed_trucks = [
            item.strip()
            for item in self.trucks_raw.split(',')
            if item.strip()
        ]

        # -----------------------------------------------------
        # Load scenario
        # -----------------------------------------------------

        self.scenario_path = (
            resolve_scenario_yaml(
                self.scenario_name
            )
        )

        if self.scenario_path is None:

            raise RuntimeError(
                'Scenario YAML could not be found: '
                f'{self.scenario_name}'
            )

        with open(
            self.scenario_path,
            'r',
            encoding='utf-8',
        ) as f:

            data = yaml.safe_load(
                f
            ) or {}

        self.scenario_title = str(
            data.get(
                'scenario_name',
                self.scenario_name,
            )
        )

        self.steps = data.get(
            'steps',
            [],
        )

        if not isinstance(
            self.steps,
            list,
        ):

            raise RuntimeError(
                'Scenario "steps" must be a list.'
            )

        if not self.steps:

            raise RuntimeError(
                'Scenario contains no steps.'
            )

        # -----------------------------------------------------
        # Dump truck Action clients
        # -----------------------------------------------------

        self.action_clients = {}

        self.action_clients_lock = (
            threading.Lock()
        )

        # -----------------------------------------------------
        # Excavator Action clients
        # -----------------------------------------------------

        self.excavator_action_clients = {}

        self.excavator_action_clients_lock = (
            threading.Lock()
        )

        # -----------------------------------------------------
        # Active Action goals
        #
        # step_id:
        #   {
        #       'robot_name': robot_name,
        #       'goal_handle': goal_handle,
        #   }
        # -----------------------------------------------------

        self.active_goal_handles = {}

        self.active_goal_handles_lock = (
            threading.Lock()
        )

        # -----------------------------------------------------
        # Condition subscriptions
        # -----------------------------------------------------

        self.condition_subscriptions = []

        self.condition_subscriptions_lock = (
            threading.Lock()
        )

        # -----------------------------------------------------
        # Generic publishers
        # -----------------------------------------------------

        self.topic_publishers = {}

        self.topic_publishers_lock = (
            threading.Lock()
        )

        # -----------------------------------------------------
        # Scenario state
        # -----------------------------------------------------

        self.current_step_index = 0

        self.scenario_started = False
        self.scenario_finished = False

        self.execution_thread = None

        # -----------------------------------------------------
        # Start scenario
        # -----------------------------------------------------

        self.start_timer = (
            self.create_timer(
                1.0,
                self.start_once,
            )
        )

        self.get_logger().info(
            'Scenario Manager initialized. '
            f'scenario={self.scenario_title}, '
            f'file={self.scenario_path}, '
            f'steps={len(self.steps)}, '
            f'allowed_trucks={self.allowed_trucks}'
        )

    # =========================================================
    # Startup
    # =========================================================

    def start_once(self):

        if self.scenario_started:
            return

        self.scenario_started = True

        self.start_timer.cancel()

        self.execution_thread = (
            threading.Thread(
                target=self.execute_scenario,
                daemon=True,
            )
        )

        self.execution_thread.start()

    # =========================================================
    # Robot busy check
    # =========================================================

    def is_robot_busy(
        self,
        robot_name,
    ):

        with self.active_goal_handles_lock:

            for active_goal in (
                self.active_goal_handles.values()
            ):

                if (
                    active_goal.get(
                        'robot_name'
                    )
                    == robot_name
                ):

                    return True

        return False

    # =========================================================
    # Dump truck Action client
    # =========================================================

    def get_action_client(
        self,
        robot_name,
    ):

        with self.action_clients_lock:

            if (
                robot_name
                not in self.action_clients
            ):

                action_name = (
                    f'/{robot_name}/'
                    'execute_robot_task'
                )

                self.action_clients[
                    robot_name
                ] = ActionClient(
                    self,
                    ExecuteRobotTask,
                    action_name,
                )

            return self.action_clients[
                robot_name
            ]

    # =========================================================
    # Excavator Action client
    # =========================================================

    def get_excavator_action_client(
        self,
        action_name,
    ):

        with (
            self.excavator_action_clients_lock
        ):

            if (
                action_name
                not in self.excavator_action_clients
            ):

                self.excavator_action_clients[
                    action_name
                ] = ActionClient(
                    self,
                    FollowJointTrajectory,
                    action_name,
                )

            return (
                self.excavator_action_clients[
                    action_name
                ]
            )

    # =========================================================
    # Dump truck feedback
    # =========================================================

    def feedback_callback(
        self,
        robot_name,
        step_id,
        feedback_msg,
    ):

        feedback = (
            feedback_msg.feedback
        )

        self.get_logger().info(
            f'[{step_id}] '
            f'{robot_name}: '
            f'state={feedback.state}, '
            f'progress={feedback.progress:.2f}, '
            f'detail={feedback.detail}'
        )

    # =========================================================
    # Excavator feedback
    # =========================================================

    def excavator_feedback_callback(
        self,
        robot_name,
        step_id,
        joint_names,
        feedback_msg,
    ):

        feedback = (
            feedback_msg.feedback
        )

        desired_deg = [
            round(
                math.degrees(
                    value
                ),
                1,
            )
            for value in (
                feedback.desired.positions
            )
        ]

        actual_deg = [
            round(
                math.degrees(
                    value
                ),
                1,
            )
            for value in (
                feedback.actual.positions
            )
        ]

        desired_text = ', '.join(
            (
                f'{joint}={value:+.1f}'
            )
            for (
                joint,
                value,
            ) in zip(
                joint_names,
                desired_deg,
            )
        )

        actual_text = ', '.join(
            (
                f'{joint}={value:+.1f}'
            )
            for (
                joint,
                value,
            ) in zip(
                joint_names,
                actual_deg,
            )
        )

        self.get_logger().info(
            f'[{step_id}] '
            f'{robot_name}: '
            f'desired=[{desired_text}] deg, '
            f'actual=[{actual_text}] deg'
        )

    # =========================================================
    # TASK - ExecuteRobotTask
    # =========================================================

    def execute_task_step(
        self,
        step,
        step_number,
        total_steps,
        context_label='STEP',
    ):

        step_id = str(
            step.get(
                'id',
                f'step_{step_number}',
            )
        )

        robot_name = str(
            step.get(
                'robot',
                '',
            )
        ).strip().strip('/')

        task_type = str(
            step.get(
                'task_type',
                '',
            )
        ).strip().lower()

        task_file = str(
            step.get(
                'task_file',
                '',
            )
        ).strip()

        # -----------------------------------------------------
        # Validation
        # -----------------------------------------------------

        if not robot_name:

            raise RuntimeError(
                f'{step_id}: robot is missing.'
            )

        if not task_type:

            raise RuntimeError(
                f'{step_id}: task_type is missing.'
            )

        if not task_file:

            raise RuntimeError(
                f'{step_id}: task_file is missing.'
            )

        if (
            self.allowed_trucks
            and robot_name not in self.allowed_trucks
        ):

            raise RuntimeError(
                f'{step_id}: robot "{robot_name}" '
                'is not included in the '
                'launch trucks parameter.'
            )

        # -----------------------------------------------------
        # Action client
        # -----------------------------------------------------

        client = self.get_action_client(
            robot_name
        )

        action_name = (
            f'/{robot_name}/execute_robot_task'
        )

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            f'{context_label}: TASK'
        )

        self.get_logger().info(
            f'id={step_id}'
        )

        self.get_logger().info(
            f'robot={robot_name}'
        )

        self.get_logger().info(
            f'task_type={task_type}'
        )

        self.get_logger().info(
            f'task_file={task_file}'
        )

        # -----------------------------------------------------
        # Wait for Action Server
        # -----------------------------------------------------

        self.get_logger().info(
            'Waiting for Action Server: '
            f'{action_name}'
        )

        while rclpy.ok():

            if client.wait_for_server(
                timeout_sec=1.0
            ):
                break

            self.get_logger().warn(
                'Action Server not ready yet: '
                f'{action_name}'
            )

        if not rclpy.ok():
            return False

        # -----------------------------------------------------
        # Goal
        # -----------------------------------------------------

        goal = (
            ExecuteRobotTask.Goal()
        )

        goal.robot_name = (
            robot_name
        )

        goal.task_type = (
            task_type
        )

        goal.task_file = (
            task_file
        )

        # -----------------------------------------------------
        # Send Goal
        # -----------------------------------------------------

        self.get_logger().info(
            f'{step_id}: Sending goal to '
            f'{robot_name}'
        )

        send_goal_future = (
            client.send_goal_async(
                goal,
                feedback_callback=lambda msg: (
                    self.feedback_callback(
                        robot_name,
                        step_id,
                        msg,
                    )
                ),
            )
        )

        while (
            rclpy.ok()
            and not send_goal_future.done()
        ):

            time.sleep(
                0.05
            )

        if not rclpy.ok():
            return False

        goal_handle = (
            send_goal_future.result()
        )

        if goal_handle is None:

            self.get_logger().error(
                f'{step_id}: '
                'No goal handle returned.'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                f'{step_id}: '
                f'Goal rejected by {robot_name}.'
            )

            return False

        # -----------------------------------------------------
        # Register active goal
        # -----------------------------------------------------

        with self.active_goal_handles_lock:

            self.active_goal_handles[
                step_id
            ] = {
                'robot_name': (
                    robot_name
                ),
                'goal_handle': (
                    goal_handle
                ),
            }

        self.get_logger().info(
            f'{step_id}: Goal accepted.'
        )

        # -----------------------------------------------------
        # Wait for result
        # -----------------------------------------------------

        result_future = (
            goal_handle.get_result_async()
        )

        while (
            rclpy.ok()
            and not result_future.done()
        ):

            time.sleep(
                0.05
            )

        if not rclpy.ok():
            return False

        wrapped_result = (
            result_future.result()
        )

        # -----------------------------------------------------
        # Remove active goal
        # -----------------------------------------------------

        with self.active_goal_handles_lock:

            self.active_goal_handles.pop(
                step_id,
                None,
            )

        if wrapped_result is None:

            self.get_logger().error(
                f'{step_id}: '
                'No Action result returned.'
            )

            return False

        result = (
            wrapped_result.result
        )

        # -----------------------------------------------------
        # Evaluate result
        # -----------------------------------------------------

        if result.success:

            self.get_logger().info(
                f'{step_id}: SUCCESS'
            )

            self.get_logger().info(
                result.message
            )

            return True

        self.get_logger().error(
            f'{step_id}: FAILED'
        )

        self.get_logger().error(
            result.message
        )

        return False

    # =========================================================
    # EXCAVATOR TRAJECTORY
    # =========================================================

    def execute_excavator_trajectory_step(
        self,
        step,
        step_number,
        total_steps,
        context_label='STEP',
    ):

        step_id = str(
            step.get(
                'id',
                f'excavator_{step_number}',
            )
        )

        robot_name = str(
            step.get(
                'robot',
                'excavator1',
            )
        ).strip().strip('/')

        task_file = str(
            step.get(
                'task_file',
                '',
            )
        ).strip()

        explicit_action_name = str(
            step.get(
                'action_name',
                '',
            )
        ).strip()

        if explicit_action_name:

            action_name = (
                explicit_action_name
            )

        else:

            action_name = (
                get_excavator_action_name(
                    robot_name
                )
            )

        try:

            seconds_per_waypoint = float(
                step.get(
                    'seconds_per_waypoint',
                    3.0,
                )
            )

        except (
            TypeError,
            ValueError,
        ):

            raise RuntimeError(
                f'{step_id}: '
                'seconds_per_waypoint '
                'must be numeric.'
            )

        # -----------------------------------------------------
        # Validation
        # -----------------------------------------------------

        if not robot_name:

            raise RuntimeError(
                f'{step_id}: robot is missing.'
            )

        if not task_file:

            raise RuntimeError(
                f'{step_id}: task_file is missing.'
            )

        if not action_name:

            raise RuntimeError(
                f'{step_id}: action_name is empty.'
            )

        if seconds_per_waypoint <= 0.0:

            raise RuntimeError(
                f'{step_id}: '
                'seconds_per_waypoint must '
                'be greater than zero.'
            )

        if self.is_robot_busy(
            robot_name
        ):

            raise RuntimeError(
                f'{step_id}: robot "{robot_name}" '
                'already has an active task.'
            )

        # -----------------------------------------------------
        # Resolve and load trajectory
        # -----------------------------------------------------

        trajectory_path = (
            resolve_excavator_trajectory_yaml(
                task_file
            )
        )

        if trajectory_path is None:

            raise RuntimeError(
                f'{step_id}: excavator trajectory '
                f'could not be found: '
                f'{task_file}'
            )

        trajectory = (
            load_excavator_trajectory(
                trajectory_path
            )
        )

        ros_joint_names = [
            EXCAVATOR_JOINT_NAMES[
                joint_name
            ]
            for joint_name in (
                trajectory['joints']
            )
        ]

        # -----------------------------------------------------
        # Logging
        # -----------------------------------------------------

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            f'{context_label}: '
            'EXCAVATOR_TRAJECTORY'
        )

        self.get_logger().info(
            f'id={step_id}'
        )

        self.get_logger().info(
            f'robot={robot_name}'
        )

        self.get_logger().info(
            f'task_file={task_file}'
        )

        self.get_logger().info(
            f'trajectory='
            f'{trajectory["trajectory_name"]}'
        )

        self.get_logger().info(
            'joints='
            + ', '.join(
                trajectory['joints']
            )
        )

        self.get_logger().info(
            f'waypoints='
            f'{len(trajectory["waypoints"])}'
        )

        self.get_logger().info(
            'seconds_per_waypoint='
            f'{seconds_per_waypoint:.2f}'
        )

        self.get_logger().info(
            f'action={action_name}'
        )

        # -----------------------------------------------------
        # Create Action goal
        # -----------------------------------------------------

        goal = (
            FollowJointTrajectory.Goal()
        )

        goal.trajectory.joint_names = (
            ros_joint_names
        )

        for (
            index,
            waypoint,
        ) in enumerate(
            trajectory['waypoints']
        ):

            point = (
                JointTrajectoryPoint()
            )

            point.positions = [
                math.radians(
                    position_deg
                )
                for position_deg in (
                    waypoint[
                        'positions_deg'
                    ]
                )
            ]

            if 'swing' in trajectory['joints']:
                point.velocities = [
                    float(waypoint['swing_direction']) if joint == 'swing' else 0.0
                    for joint in trajectory['joints']
                ]

            waypoint_time = (
                (index + 1)
                * seconds_per_waypoint
            )

            sec, nanosec = (
                seconds_to_duration(
                    waypoint_time
                )
            )

            point.time_from_start.sec = (
                sec
            )

            point.time_from_start.nanosec = (
                nanosec
            )

            goal.trajectory.points.append(
                point
            )

        # -----------------------------------------------------
        # Action client
        # -----------------------------------------------------

        client = (
            self.get_excavator_action_client(
                action_name
            )
        )

        self.get_logger().info(
            'Waiting for Excavator '
            'Action Server: '
            f'{action_name}'
        )

        while rclpy.ok():

            if client.wait_for_server(
                timeout_sec=1.0
            ):

                break

            self.get_logger().warn(
                'Excavator Action Server '
                'not ready yet: '
                f'{action_name}'
            )

        if not rclpy.ok():
            return False

        # -----------------------------------------------------
        # Send goal
        # -----------------------------------------------------

        self.get_logger().info(
            f'{step_id}: '
            'Sending excavator trajectory.'
        )

        send_goal_future = (
            client.send_goal_async(
                goal,
                feedback_callback=lambda msg: (
                    self.excavator_feedback_callback(
                        robot_name,
                        step_id,
                        trajectory['joints'],
                        msg,
                    )
                ),
            )
        )

        while (
            rclpy.ok()
            and not send_goal_future.done()
        ):

            time.sleep(
                0.05
            )

        if not rclpy.ok():
            return False

        goal_handle = (
            send_goal_future.result()
        )

        if goal_handle is None:

            self.get_logger().error(
                f'{step_id}: '
                'No excavator goal handle '
                'returned.'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                f'{step_id}: '
                'Excavator trajectory '
                'REJECTED.'
            )

            return False

        # -----------------------------------------------------
        # Register active goal
        # -----------------------------------------------------

        with self.active_goal_handles_lock:

            self.active_goal_handles[
                step_id
            ] = {
                'robot_name': (
                    robot_name
                ),
                'goal_handle': (
                    goal_handle
                ),
            }

        self.get_logger().info(
            f'{step_id}: '
            'Excavator trajectory ACCEPTED.'
        )

        # -----------------------------------------------------
        # Wait for result
        # -----------------------------------------------------

        result_future = (
            goal_handle.get_result_async()
        )

        while (
            rclpy.ok()
            and not result_future.done()
        ):

            time.sleep(
                0.05
            )

        if not rclpy.ok():
            return False

        wrapped_result = (
            result_future.result()
        )

        # -----------------------------------------------------
        # Remove active goal
        # -----------------------------------------------------

        with self.active_goal_handles_lock:

            self.active_goal_handles.pop(
                step_id,
                None,
            )

        if wrapped_result is None:

            self.get_logger().error(
                f'{step_id}: '
                'No excavator Action result '
                'returned.'
            )

            return False

        result = (
            wrapped_result.result
        )

        # -----------------------------------------------------
        # Evaluate result
        # -----------------------------------------------------

        if (
            wrapped_result.status
            == GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().info(
                f'{step_id}: '
                'EXCAVATOR SUCCESS'
            )

            if result.error_string:

                self.get_logger().info(
                    result.error_string
                )

            return True

        self.get_logger().error(
            f'{step_id}: '
            'EXCAVATOR FAILED'
        )

        self.get_logger().error(
            f'status='
            f'{wrapped_result.status}, '
            f'error_code='
            f'{result.error_code}, '
            f'message='
            f'{result.error_string}'
        )

        return False

    # =========================================================
    # WAIT
    # =========================================================

    def execute_wait_step(
        self,
        step,
        step_number,
        total_steps,
        context_label='STEP',
    ):

        step_id = str(
            step.get(
                'id',
                f'step_{step_number}',
            )
        )

        try:

            duration = float(
                step.get(
                    'duration',
                    0.0,
                )
            )

        except (
            TypeError,
            ValueError,
        ):

            raise RuntimeError(
                f'{step_id}: '
                'duration must be numeric.'
            )

        if duration < 0.0:

            raise RuntimeError(
                f'{step_id}: '
                'duration cannot be negative.'
            )

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            f'{context_label}: WAIT'
        )

        self.get_logger().info(
            f'id={step_id}'
        )

        self.get_logger().info(
            f'duration={duration:.2f} sec'
        )

        start_time = (
            time.monotonic()
        )

        while rclpy.ok():

            elapsed = (
                time.monotonic()
                - start_time
            )

            remaining = (
                duration
                - elapsed
            )

            if remaining <= 0.0:
                break

            time.sleep(
                min(
                    0.1,
                    remaining,
                )
            )

        if not rclpy.ok():
            return False

        self.get_logger().info(
            f'{step_id}: WAIT COMPLETE'
        )

        return True

    # =========================================================
    # Generic publisher helper
    # =========================================================

    def get_topic_publisher(
        self,
        topic_name,
        msg_class,
        msg_type_name,
    ):

        key = (
            topic_name,
            msg_type_name,
        )

        with self.topic_publishers_lock:

            if (
                key
                not in self.topic_publishers
            ):

                self.topic_publishers[
                    key
                ] = self.create_publisher(
                    msg_class,
                    topic_name,
                    10,
                )

            return (
                self.topic_publishers[
                    key
                ]
            )

    # =========================================================
    # Extract robot name from topic
    #
    # /truck1/cmd_vel -> truck1
    # /excavator1/cmd_vel -> excavator1
    # =========================================================

    def get_robot_name_from_topic(
        self,
        topic_name,
    ):

        parts = [
            part
            for part in (
                topic_name.split('/')
            )
            if part
        ]

        if len(parts) < 2:
            return None

        return parts[0]

    # =========================================================
    # TOPIC PUBLISH
    # =========================================================

    def execute_topic_publish_step(
        self,
        step,
        step_number,
        total_steps,
        context_label='STEP',
    ):

        step_id = str(
            step.get(
                'id',
                f'topic_publish_{step_number}',
            )
        )

        topic_name = str(
            step.get(
                'topic',
                '',
            )
        ).strip()

        msg_type = str(
            step.get(
                'msg_type',
                '',
            )
        ).strip()

        message_data = step.get(
            'message',
            {},
        )

        try:

            duration = float(
                step.get(
                    'duration',
                    0.0,
                )
            )

        except (
            TypeError,
            ValueError,
        ):

            raise RuntimeError(
                f'{step_id}: '
                'duration must be numeric.'
            )

        try:

            rate_hz = float(
                step.get(
                    'rate_hz',
                    10.0,
                )
            )

        except (
            TypeError,
            ValueError,
        ):

            raise RuntimeError(
                f'{step_id}: '
                'rate_hz must be numeric.'
            )

        if not topic_name:

            raise RuntimeError(
                f'{step_id}: topic is missing.'
            )

        if duration < 0.0:

            raise RuntimeError(
                f'{step_id}: '
                'duration cannot be negative.'
            )

        if rate_hz <= 0.0:

            raise RuntimeError(
                f'{step_id}: '
                'rate_hz must be greater '
                'than zero.'
            )

        if not isinstance(
            message_data,
            dict,
        ):

            raise RuntimeError(
                f'{step_id}: '
                'message must be a YAML mapping.'
            )

        robot_name = (
            self.get_robot_name_from_topic(
                topic_name
            )
        )

        if (
            robot_name is not None
            and self.is_robot_busy(
                robot_name
            )
        ):

            raise RuntimeError(
                f'{step_id}: robot '
                f'"{robot_name}" currently has '
                'an active Action task. '
                'Direct topic command rejected.'
            )

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            f'{context_label}: TOPIC_PUBLISH'
        )

        self.get_logger().info(
            f'id={step_id}'
        )

        self.get_logger().info(
            f'topic={topic_name}'
        )

        self.get_logger().info(
            f'msg_type={msg_type}'
        )

        self.get_logger().info(
            f'duration={duration:.2f} sec'
        )

        # =====================================================
        # std_msgs/String
        # =====================================================

        if msg_type in (
            'std_msgs/String',
            'std_msgs/msg/String',
        ):

            msg = String()

            msg.data = str(
                message_data.get(
                    'data',
                    '',
                )
            )

            publisher = (
                self.get_topic_publisher(
                    topic_name,
                    String,
                    'std_msgs/msg/String',
                )
            )

            publisher.publish(
                msg
            )

            self.get_logger().info(
                f'{step_id}: Published '
                f'String(data="{msg.data}")'
            )

            return True

        # =====================================================
        # geometry_msgs/Twist
        # =====================================================

        if msg_type in (
            'geometry_msgs/Twist',
            'geometry_msgs/msg/Twist',
        ):

            linear = (
                message_data.get(
                    'linear',
                    {},
                )
            )

            angular = (
                message_data.get(
                    'angular',
                    {},
                )
            )

            if not isinstance(
                linear,
                dict,
            ):

                raise RuntimeError(
                    f'{step_id}: '
                    'message.linear must '
                    'be a mapping.'
                )

            if not isinstance(
                angular,
                dict,
            ):

                raise RuntimeError(
                    f'{step_id}: '
                    'message.angular must '
                    'be a mapping.'
                )

            msg = Twist()

            msg.linear.x = float(
                linear.get(
                    'x',
                    0.0,
                )
            )

            msg.linear.y = float(
                linear.get(
                    'y',
                    0.0,
                )
            )

            msg.linear.z = float(
                linear.get(
                    'z',
                    0.0,
                )
            )

            msg.angular.x = float(
                angular.get(
                    'x',
                    0.0,
                )
            )

            msg.angular.y = float(
                angular.get(
                    'y',
                    0.0,
                )
            )

            msg.angular.z = float(
                angular.get(
                    'z',
                    0.0,
                )
            )

            publisher = (
                self.get_topic_publisher(
                    topic_name,
                    Twist,
                    'geometry_msgs/msg/Twist',
                )
            )

            if duration <= 0.0:

                publisher.publish(
                    msg
                )

                self.get_logger().info(
                    f'{step_id}: '
                    'Twist published once.'
                )

                return True

            period = (
                1.0
                / rate_hz
            )

            start_time = (
                time.monotonic()
            )

            try:

                while rclpy.ok():

                    elapsed = (
                        time.monotonic()
                        - start_time
                    )

                    if elapsed >= duration:
                        break

                    publisher.publish(
                        msg
                    )

                    time.sleep(
                        period
                    )

            finally:

                publisher.publish(
                    Twist()
                )

                self.get_logger().info(
                    f'{step_id}: '
                    'Zero Twist published.'
                )

            if not rclpy.ok():
                return False

            self.get_logger().info(
                f'{step_id}: '
                'TOPIC_PUBLISH COMPLETE'
            )

            return True

        raise RuntimeError(
            f'{step_id}: unsupported '
            f'msg_type "{msg_type}". '
            'Supported types are currently '
            'std_msgs/String and '
            'geometry_msgs/Twist.'
        )

    # =========================================================
    # PARALLEL worker
    # =========================================================

    def parallel_worker(
        self,
        child_step,
        child_number,
        total_children,
        parent_id,
        results,
        results_lock,
    ):

        child_id = str(
            child_step.get(
                'id',
                (
                    f'{parent_id}_child_'
                    f'{child_number}'
                ),
            )
        )

        try:

            success = (
                self.execute_step(
                    child_step,
                    child_number,
                    total_children,
                    context_prefix=(
                        f'PARALLEL '
                        f'{child_number}/'
                        f'{total_children} '
                    ),
                )
            )

        except Exception as exc:

            self.get_logger().error(
                f'{child_id}: '
                'PARALLEL CHILD ERROR: '
                f'{exc}'
            )

            success = False

        with results_lock:

            results[
                child_id
            ] = success

    # =========================================================
    # PARALLEL
    # =========================================================

    def execute_parallel_step(
        self,
        step,
        step_number,
        total_steps,
    ):

        parent_id = str(
            step.get(
                'id',
                f'parallel_{step_number}',
            )
        )

        children = step.get(
            'tasks',
            None,
        )

        if children is None:

            children = step.get(
                'steps',
                [],
            )

        if not isinstance(
            children,
            list,
        ):

            raise RuntimeError(
                f'{parent_id}: '
                'parallel tasks must be a list.'
            )

        if not children:

            raise RuntimeError(
                f'{parent_id}: '
                'parallel block contains '
                'no tasks.'
            )

        # -----------------------------------------------------
        # Prevent two Action tasks from using same robot
        # -----------------------------------------------------

        robots_in_parallel = []

        for child in children:

            if not isinstance(
                child,
                dict,
            ):
                continue

            child_type = str(
                child.get(
                    'type',
                    'task',
                )
            ).strip().lower()

            if child_type not in (
                'task',
                'excavator_trajectory',
            ):
                continue

            robot_name = str(
                child.get(
                    'robot',
                    '',
                )
            ).strip().strip('/')

            if not robot_name:
                continue

            if (
                robot_name
                in robots_in_parallel
            ):

                raise RuntimeError(
                    f'{parent_id}: robot '
                    f'"{robot_name}" appears '
                    'more than once as an '
                    'Action task in the same '
                    'parallel block.'
                )

            robots_in_parallel.append(
                robot_name
            )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'STEP {step_number}/{total_steps}: '
            'PARALLEL'
        )

        self.get_logger().info(
            f'id={parent_id}'
        )

        self.get_logger().info(
            f'children={len(children)}'
        )

        self.get_logger().info(
            'PARALLEL START'
        )

        self.get_logger().info(
            '========================================'
        )

        results = {}

        results_lock = (
            threading.Lock()
        )

        threads = []

        total_children = len(
            children
        )

        for (
            index,
            child_step,
        ) in enumerate(
            children
        ):

            thread = threading.Thread(
                target=self.parallel_worker,
                args=(
                    child_step,
                    index + 1,
                    total_children,
                    parent_id,
                    results,
                    results_lock,
                ),
                daemon=True,
            )

            threads.append(
                thread
            )

        for thread in threads:
            thread.start()

        for thread in threads:

            while (
                rclpy.ok()
                and thread.is_alive()
            ):

                thread.join(
                    timeout=0.1
                )

        if not rclpy.ok():
            return False

        all_success = (
            len(results)
            == total_children
            and all(
                results.values()
            )
        )

        self.get_logger().info(
            '========================================'
        )

        if all_success:

            self.get_logger().info(
                f'{parent_id}: '
                'PARALLEL COMPLETE'
            )

            for (
                child_id,
                success,
            ) in results.items():

                self.get_logger().info(
                    f'  {child_id}: '
                    f'{"SUCCESS" if success else "FAILED"}'
                )

            self.get_logger().info(
                '========================================'
            )

            return True

        self.get_logger().error(
            f'{parent_id}: PARALLEL FAILED'
        )

        for (
            child_id,
            success,
        ) in results.items():

            self.get_logger().error(
                f'  {child_id}: '
                f'{"SUCCESS" if success else "FAILED"}'
            )

        self.get_logger().error(
            '========================================'
        )

        return False

    # =========================================================
    # CONDITION helpers
    # =========================================================

    def update_condition_state(
        self,
        value,
        condition_state,
    ):

        value_string = str(
            value
        )

        with condition_state[
            'lock'
        ]:

            condition_state[
                'last_value'
            ] = value_string

            if (
                value_string
                == condition_state[
                    'expected_value'
                ]
            ):

                condition_state[
                    'matched'
                ].set()

    # =========================================================
    # CONDITION callback: std_msgs/String
    # =========================================================

    def condition_string_callback(
        self,
        msg,
        condition_state,
    ):

        self.update_condition_state(
            msg.data,
            condition_state,
        )

    # =========================================================
    # CONDITION callback: RobotStatus
    # =========================================================

    def condition_robot_status_callback(
        self,
        msg,
        condition_state,
    ):

        field_name = (
            condition_state[
                'field'
            ]
        )

        if not hasattr(
            msg,
            field_name,
        ):

            if not condition_state[
                'field_error_logged'
            ]:

                condition_state[
                    'field_error_logged'
                ] = True

                self.get_logger().error(
                    'RobotStatus does not contain '
                    f'field "{field_name}".'
                )

            return

        field_value = getattr(
            msg,
            field_name,
        )

        self.update_condition_state(
            field_value,
            condition_state,
        )

    # =========================================================
    # CONDITION branch
    # =========================================================

    def execute_branch(
        self,
        branch_steps,
        parent_id,
        branch_name,
    ):

        if branch_steps is None:
            return True

        if not isinstance(
            branch_steps,
            list,
        ):

            raise RuntimeError(
                f'{parent_id}: '
                f'{branch_name} branch '
                'must be a list.'
            )

        if not branch_steps:

            self.get_logger().info(
                f'{parent_id}: '
                f'{branch_name.upper()} '
                'branch is empty.'
            )

            return True

        self.get_logger().info(
            f'{parent_id}: Executing '
            f'{branch_name.upper()} branch.'
        )

        total_branch_steps = len(
            branch_steps
        )

        for (
            index,
            branch_step,
        ) in enumerate(
            branch_steps
        ):

            if not rclpy.ok():
                return False

            success = (
                self.execute_step(
                    branch_step,
                    index + 1,
                    total_branch_steps,
                    context_prefix=(
                        f'{branch_name.upper()} '
                    ),
                )
            )

            if not success:

                self.get_logger().error(
                    f'{parent_id}: '
                    f'{branch_name.upper()} '
                    'branch failed.'
                )

                return False

        return True

    # =========================================================
    # CONDITION
    # =========================================================

    def execute_condition_step(
        self,
        step,
        step_number,
        total_steps,
    ):

        step_id = str(
            step.get(
                'id',
                f'condition_{step_number}',
            )
        )

        condition = step.get(
            'condition',
            {},
        )

        if not isinstance(
            condition,
            dict,
        ):

            raise RuntimeError(
                f'{step_id}: '
                'condition must be a mapping.'
            )

        source = str(
            condition.get(
                'source',
                '',
            )
        ).strip().lower()

        topic_name = str(
            condition.get(
                'topic',
                '',
            )
        ).strip()

        msg_type = str(
            condition.get(
                'msg_type',
                '',
            )
        ).strip()

        field_name = str(
            condition.get(
                'field',
                '',
            )
        ).strip()

        expected_value = str(
            condition.get(
                'equals',
                '',
            )
        )

        try:

            timeout = float(
                condition.get(
                    'timeout',
                    30.0,
                )
            )

        except (
            TypeError,
            ValueError,
        ):

            raise RuntimeError(
                f'{step_id}: '
                'condition timeout '
                'must be numeric.'
            )

        if source != 'topic':

            raise RuntimeError(
                f'{step_id}: unsupported '
                f'condition source "{source}". '
                'Currently supported: topic.'
            )

        if not topic_name:

            raise RuntimeError(
                f'{step_id}: '
                'condition topic is missing.'
            )

        if timeout < 0.0:

            raise RuntimeError(
                f'{step_id}: '
                'timeout cannot be negative.'
            )

        is_string_condition = (
            msg_type in (
                'std_msgs/String',
                'std_msgs/msg/String',
            )
        )

        is_robot_status_condition = (
            msg_type in (
                (
                    'construction_site_interfaces/'
                    'RobotStatus'
                ),
                (
                    'construction_site_interfaces/'
                    'msg/RobotStatus'
                ),
            )
        )

        if (
            not is_string_condition
            and not is_robot_status_condition
        ):

            raise RuntimeError(
                f'{step_id}: unsupported '
                'condition msg_type '
                f'"{msg_type}". '
                'Supported condition types '
                'are std_msgs/String and '
                'construction_site_interfaces/'
                'msg/RobotStatus.'
            )

        if is_robot_status_condition:

            if not field_name:

                raise RuntimeError(
                    f'{step_id}: '
                    'RobotStatus condition '
                    'requires "field".'
                )

            valid_fields = (
                'robot_name',
                'state',
                'detail',
            )

            if (
                field_name
                not in valid_fields
            ):

                raise RuntimeError(
                    f'{step_id}: invalid '
                    'RobotStatus field '
                    f'"{field_name}". '
                    'Supported fields: '
                    'robot_name, state, detail.'
                )

        if is_string_condition:

            field_name = 'data'

        condition_state = {
            'expected_value': (
                expected_value
            ),
            'last_value': None,
            'matched': threading.Event(),
            'lock': threading.Lock(),
            'field': field_name,
            'field_error_logged': False,
        }

        if is_string_condition:

            subscription = (
                self.create_subscription(
                    String,
                    topic_name,
                    lambda msg: (
                        self.condition_string_callback(
                            msg,
                            condition_state,
                        )
                    ),
                    10,
                )
            )

        else:

            status_qos = QoSProfile(
                history=(
                    HistoryPolicy.KEEP_LAST
                ),
                depth=1,
                reliability=(
                    ReliabilityPolicy.RELIABLE
                ),
                durability=(
                    DurabilityPolicy.TRANSIENT_LOCAL
                ),
            )

            subscription = (
                self.create_subscription(
                    RobotStatus,
                    topic_name,
                    lambda msg: (
                        self.condition_robot_status_callback(
                            msg,
                            condition_state,
                        )
                    ),
                    status_qos,
                )
            )

        with self.condition_subscriptions_lock:

            self.condition_subscriptions.append(
                subscription
            )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'STEP {step_number}/{total_steps}: '
            'CONDITION'
        )

        self.get_logger().info(
            f'id={step_id}'
        )

        self.get_logger().info(
            f'source={source}'
        )

        self.get_logger().info(
            f'topic={topic_name}'
        )

        self.get_logger().info(
            f'msg_type={msg_type}'
        )

        self.get_logger().info(
            f'field={field_name}'
        )

        self.get_logger().info(
            f'equals="{expected_value}"'
        )

        self.get_logger().info(
            f'timeout={timeout:.2f} sec'
        )

        self.get_logger().info(
            'CONDITION WAITING'
        )

        self.get_logger().info(
            '========================================'
        )

        start_time = (
            time.monotonic()
        )

        matched = False

        while rclpy.ok():

            if condition_state[
                'matched'
            ].is_set():

                matched = True
                break

            elapsed = (
                time.monotonic()
                - start_time
            )

            if elapsed >= timeout:
                break

            time.sleep(
                0.05
            )

        try:

            self.destroy_subscription(
                subscription
            )

        finally:

            with (
                self.condition_subscriptions_lock
            ):

                if (
                    subscription
                    in self.condition_subscriptions
                ):

                    self.condition_subscriptions.remove(
                        subscription
                    )

        if not rclpy.ok():
            return False

        if matched:

            with condition_state[
                'lock'
            ]:

                last_value = (
                    condition_state[
                        'last_value'
                    ]
                )

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                f'{step_id}: '
                'CONDITION SATISFIED'
            )

            self.get_logger().info(
                f'{field_name}='
                f'"{last_value}"'
            )

            self.get_logger().info(
                'Branch: THEN'
            )

            self.get_logger().info(
                '========================================'
            )

            return self.execute_branch(
                step.get(
                    'then',
                    [],
                ),
                step_id,
                'then',
            )

        with condition_state[
            'lock'
        ]:

            last_value = (
                condition_state[
                    'last_value'
                ]
            )

        self.get_logger().warn(
            '========================================'
        )

        self.get_logger().warn(
            f'{step_id}: CONDITION TIMEOUT'
        )

        self.get_logger().warn(
            f'field={field_name}, '
            f'expected="{expected_value}", '
            f'last_received="{last_value}"'
        )

        self.get_logger().warn(
            'Branch: ELSE'
        )

        self.get_logger().warn(
            '========================================'
        )

        return self.execute_branch(
            step.get(
                'else',
                [],
            ),
            step_id,
            'else',
        )

    # =========================================================
    # Dispatcher
    # =========================================================

    def execute_step(
        self,
        step,
        step_number,
        total_steps,
        context_prefix='',
    ):

        if not isinstance(
            step,
            dict,
        ):

            raise RuntimeError(
                f'Step {step_number} '
                'must be a YAML mapping.'
            )

        step_type = str(
            step.get(
                'type',
                'task',
            )
        ).strip().lower()

        context_label = (
            f'{context_prefix}STEP'
        ).strip()

        # -----------------------------------------------------
        # TASK
        # -----------------------------------------------------

        if step_type == 'task':

            return self.execute_task_step(
                step,
                step_number,
                total_steps,
                context_label=(
                    context_label
                ),
            )

        # -----------------------------------------------------
        # EXCAVATOR TRAJECTORY
        # -----------------------------------------------------

        if (
            step_type
            == 'excavator_trajectory'
        ):

            return (
                self.execute_excavator_trajectory_step(
                    step,
                    step_number,
                    total_steps,
                    context_label=(
                        context_label
                    ),
                )
            )

        # -----------------------------------------------------
        # WAIT
        # -----------------------------------------------------

        if step_type == 'wait':

            return self.execute_wait_step(
                step,
                step_number,
                total_steps,
                context_label=(
                    context_label
                ),
            )

        # -----------------------------------------------------
        # TOPIC PUBLISH
        # -----------------------------------------------------

        if (
            step_type
            == 'topic_publish'
        ):

            return (
                self.execute_topic_publish_step(
                    step,
                    step_number,
                    total_steps,
                    context_label=(
                        context_label
                    ),
                )
            )

        # -----------------------------------------------------
        # PARALLEL
        # -----------------------------------------------------

        if step_type == 'parallel':

            return self.execute_parallel_step(
                step,
                step_number,
                total_steps,
            )

        # -----------------------------------------------------
        # CONDITION
        # -----------------------------------------------------

        if step_type == 'condition':

            return self.execute_condition_step(
                step,
                step_number,
                total_steps,
            )

        raise RuntimeError(
            f'Step {step_number}: '
            'unsupported step type '
            f'"{step_type}".'
        )

    # =========================================================
    # Scenario
    # =========================================================

    def execute_scenario(self):

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'SCENARIO START'
        )

        self.get_logger().info(
            f'Name: {self.scenario_title}'
        )

        self.get_logger().info(
            f'Steps: {len(self.steps)}'
        )

        self.get_logger().info(
            '========================================'
        )

        total_steps = len(
            self.steps
        )

        try:

            for (
                index,
                step,
            ) in enumerate(
                self.steps
            ):

                if not rclpy.ok():
                    return

                self.current_step_index = (
                    index
                )

                success = (
                    self.execute_step(
                        step,
                        index + 1,
                        total_steps,
                    )
                )

                if not success:

                    self.get_logger().error(
                        '========================================'
                    )

                    self.get_logger().error(
                        'SCENARIO ABORTED'
                    )

                    self.get_logger().error(
                        'Failed at step '
                        f'{index + 1}/'
                        f'{total_steps}'
                    )

                    self.get_logger().error(
                        '========================================'
                    )

                    self.scenario_finished = True

                    return

            self.scenario_finished = True

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'SCENARIO COMPLETE'
            )

            self.get_logger().info(
                f'Name: '
                f'{self.scenario_title}'
            )

            self.get_logger().info(
                'Completed steps: '
                f'{total_steps}'
            )

            self.get_logger().info(
                '========================================'
            )

        except Exception as exc:

            self.scenario_finished = True

            self.get_logger().error(
                '========================================'
            )

            self.get_logger().error(
                'SCENARIO ERROR'
            )

            self.get_logger().error(
                str(exc)
            )

            self.get_logger().error(
                '========================================'
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        ScenarioManager()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()