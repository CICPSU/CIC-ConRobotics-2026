#!/usr/bin/env python3

import os
import threading

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from construction_site_interfaces.action import ExecuteRobotTask


def resolve_scenario_yaml(scenario_name):
    """
    Resolve scenario YAML.

    Search order:

    1. Explicit filesystem path.

    2. Source package scenarios directory.

    3. Installed package share/scenarios directory.
    """

    if not scenario_name:
        return None

    # ---------------------------------------------------------
    # 1. Explicit path
    # ---------------------------------------------------------

    explicit_path = os.path.abspath(
        os.path.expanduser(scenario_name)
    )

    if os.path.isfile(explicit_path):
        return explicit_path

    # ---------------------------------------------------------
    # 2. Source package scenarios directory
    #
    # Works well with --symlink-install.
    # ---------------------------------------------------------

    python_file = os.path.realpath(__file__)

    python_package_dir = os.path.dirname(
        python_file
    )

    package_root = os.path.dirname(
        python_package_dir
    )

    source_scenario_path = os.path.join(
        package_root,
        'scenarios',
        scenario_name,
    )

    if os.path.isfile(source_scenario_path):
        return source_scenario_path

    # ---------------------------------------------------------
    # 3. Installed package share directory
    # ---------------------------------------------------------

    try:

        package_share = get_package_share_directory(
            'construction_site_control'
        )

        installed_scenario_path = os.path.join(
            package_share,
            'scenarios',
            scenario_name,
        )

        if os.path.isfile(installed_scenario_path):
            return installed_scenario_path

    except Exception:
        pass

    return None


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
        # Resolve and load scenario
        # -----------------------------------------------------

        self.scenario_path = resolve_scenario_yaml(
            self.scenario_name
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

            data = yaml.safe_load(f) or {}

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
        # Action clients
        # -----------------------------------------------------

        self.action_clients = {}

        # -----------------------------------------------------
        # Scenario state
        # -----------------------------------------------------

        self.current_step_index = 0

        self.scenario_started = False
        self.scenario_finished = False

        self.current_goal_handle = None

        self.execution_thread = None

        # -----------------------------------------------------
        # Start scenario shortly after launch
        # -----------------------------------------------------

        self.start_timer = self.create_timer(
            1.0,
            self.start_once,
        )

        self.get_logger().info(
            'Scenario Manager initialized. '
            f'scenario={self.scenario_title}, '
            f'file={self.scenario_path}, '
            f'steps={len(self.steps)}, '
            f'allowed_trucks={self.allowed_trucks}'
        )

    # =========================================================
    # Scenario startup
    # =========================================================

    def start_once(self):

        if self.scenario_started:
            return

        self.scenario_started = True

        self.start_timer.cancel()

        self.execution_thread = threading.Thread(
            target=self.execute_scenario,
            daemon=True,
        )

        self.execution_thread.start()

    # =========================================================
    # Action client helpers
    # =========================================================

    def get_action_client(
        self,
        robot_name,
    ):

        if robot_name not in self.action_clients:

            action_name = (
                f'/{robot_name}/execute_robot_task'
            )

            client = ActionClient(
                self,
                ExecuteRobotTask,
                action_name,
            )

            self.action_clients[
                robot_name
            ] = client

        return self.action_clients[
            robot_name
        ]

    # =========================================================
    # Feedback callback
    # =========================================================

    def feedback_callback(
        self,
        robot_name,
        step_id,
        feedback_msg,
    ):

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'[{step_id}] '
            f'{robot_name}: '
            f'state={feedback.state}, '
            f'progress={feedback.progress:.2f}, '
            f'detail={feedback.detail}'
        )

    # =========================================================
    # Single task execution
    # =========================================================

    def execute_step(
        self,
        step,
        step_number,
        total_steps,
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
        # Validate step
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
            and robot_name
            not in self.allowed_trucks
        ):

            raise RuntimeError(
                f'{step_id}: robot "{robot_name}" '
                'is not included in the '
                'launch trucks parameter.'
            )

        # -----------------------------------------------------
        # Create / get Action Client
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
            f'STEP {step_number}/{total_steps}'
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

        self.get_logger().info(
            f'action={action_name}'
        )

        # -----------------------------------------------------
        # Wait for Action Server
        # -----------------------------------------------------

        self.get_logger().info(
            f'Waiting for Action Server: '
            f'{action_name}'
        )

        while rclpy.ok():

            if client.wait_for_server(
                timeout_sec=1.0
            ):
                break

            self.get_logger().warn(
                f'Action Server not ready yet: '
                f'{action_name}'
            )

        if not rclpy.ok():
            return False

        # -----------------------------------------------------
        # Build Goal
        # -----------------------------------------------------

        goal = ExecuteRobotTask.Goal()

        goal.robot_name = robot_name
        goal.task_type = task_type
        goal.task_file = task_file

        # -----------------------------------------------------
        # Send Goal
        # -----------------------------------------------------

        self.get_logger().info(
            f'Sending goal to {robot_name}'
        )

        send_goal_future = client.send_goal_async(
            goal,
            feedback_callback=lambda msg: (
                self.feedback_callback(
                    robot_name,
                    step_id,
                    msg,
                )
            ),
        )

        # -----------------------------------------------------
        # Wait for goal acceptance
        #
        # This scenario runs in a worker thread, while the
        # normal ROS executor continues spinning in main().
        # -----------------------------------------------------

        while (
            rclpy.ok()
            and not send_goal_future.done()
        ):
            threading.Event().wait(
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

        self.current_goal_handle = (
            goal_handle
        )

        self.get_logger().info(
            f'{step_id}: Goal accepted.'
        )

        # -----------------------------------------------------
        # Wait for Result
        # -----------------------------------------------------

        result_future = (
            goal_handle.get_result_async()
        )

        while (
            rclpy.ok()
            and not result_future.done()
        ):
            threading.Event().wait(
                0.05
            )

        if not rclpy.ok():
            return False

        wrapped_result = (
            result_future.result()
        )

        self.current_goal_handle = None

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
        # Evaluate Result
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
    # Scenario execution
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

            for index, step in enumerate(
                self.steps
            ):

                if not rclpy.ok():
                    return

                self.current_step_index = index

                success = self.execute_step(
                    step,
                    index + 1,
                    total_steps,
                )

                if not success:

                    self.get_logger().error(
                        '========================================'
                    )

                    self.get_logger().error(
                        'SCENARIO ABORTED'
                    )

                    self.get_logger().error(
                        f'Failed at step '
                        f'{index + 1}/{total_steps}'
                    )

                    self.get_logger().error(
                        '========================================'
                    )

                    self.scenario_finished = True

                    return

            # -------------------------------------------------
            # Scenario complete
            # -------------------------------------------------

            self.scenario_finished = True

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'SCENARIO COMPLETE'
            )

            self.get_logger().info(
                f'Name: {self.scenario_title}'
            )

            self.get_logger().info(
                f'Completed steps: {total_steps}'
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

    node = ScenarioManager()

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