#!/usr/bin/env python3
"""
motor_drive_node.py

PWM motor controller + encoder wheel-state publisher
for one dump truck.

Features
--------
- Truck-specific ROS topics.
- Two encoder direction modes:
    * quadrature
    * commanded
- Optional pigpio glitch filtering.
- Per-truck encoder inversion.
- Optional encoder LEFT/RIGHT swap.

Important note about swap_encoders
----------------------------------
Some trucks may have their physical encoder GPIO channels
connected opposite to the logical robot LEFT / RIGHT wheels.

For those trucks:

    swap_encoders = True

must affect BOTH:

1. Which encoder count is published as logical LEFT / RIGHT.
2. Which logical commanded wheel direction is used to assign
   the sign of each physical encoder callback in "commanded" mode.

Without both mappings, forward motion can look correct while
direction changes / fine steering corrections produce incorrect
odometry signs.
"""

import time

import pigpio
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


# =============================================================
# Motor PWM Pins
# BCM numbering
# =============================================================

LEFT_MOTOR_PIN = 25
RIGHT_MOTOR_PIN = 5


# =============================================================
# Motor Electrical Inversion
# =============================================================

LEFT_MOTOR_INVERT = False
RIGHT_MOTOR_INVERT = False


# =============================================================
# Encoder Pins
# BCM numbering
#
# These names represent the PHYSICAL GPIO channels.
# They may not correspond to logical robot LEFT / RIGHT.
# swap_encoders handles that difference.
# =============================================================

RIGHT_ENC_A = 18
RIGHT_ENC_B = 27

LEFT_ENC_A = 19
LEFT_ENC_B = 20


# =============================================================
# ESC PWM
# =============================================================

NEUTRAL = 1500
MAX_FORWARD = 2000
MAX_REVERSE = 1000


# =============================================================
# Drive Tuning
# =============================================================

TURN_GAIN = 0.5

MIN_EFFECTIVE_CMD = 0.18

CMD_TIMEOUT_SEC = 0.5

DIRECTION_CMD_THRESHOLD = 0.03


# =============================================================
# Helper Functions
# =============================================================

def clamp(
    value,
    minimum,
    maximum,
):
    return max(
        min(
            value,
            maximum,
        ),
        minimum,
    )


def apply_deadband_boost(
    command,
):

    if (
        0.0
        < command
        < MIN_EFFECTIVE_CMD
    ):
        return MIN_EFFECTIVE_CMD

    if (
        -MIN_EFFECTIVE_CMD
        < command
        < 0.0
    ):
        return -MIN_EFFECTIVE_CMD

    return command


def command_to_pulse(
    command,
):

    command = clamp(
        command,
        -1.0,
        1.0,
    )

    if abs(command) > 1e-6:

        command = apply_deadband_boost(
            command
        )

    pulse = (
        NEUTRAL
        + command * 500.0
    )

    return int(
        clamp(
            pulse,
            MAX_REVERSE,
            MAX_FORWARD,
        )
    )


# =============================================================
# Differential Drive Node
# =============================================================

class DifferentialDriveNode(Node):

    def __init__(self):

        super().__init__(
            'dumptruck_differential_drive'
        )

        # =====================================================
        # ROS Parameters
        # =====================================================

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel',
        )

        self.declare_parameter(
            'wheel_states_topic',
            '/wheel_states',
        )

        # -----------------------------------------------------
        # Encoder behavior
        # -----------------------------------------------------

        self.declare_parameter(
            'encoder_direction_mode',
            'quadrature',
        )

        self.declare_parameter(
            'encoder_glitch_filter_us',
            0,
        )

        self.declare_parameter(
            'left_encoder_invert',
            False,
        )

        self.declare_parameter(
            'right_encoder_invert',
            False,
        )

        self.declare_parameter(
            'swap_encoders',
            False,
        )

        # =====================================================
        # Read Parameters
        # =====================================================

        self.cmd_vel_topic = str(
            self.get_parameter(
                'cmd_vel_topic'
            ).value
        )

        self.wheel_states_topic = str(
            self.get_parameter(
                'wheel_states_topic'
            ).value
        )

        self.encoder_direction_mode = str(
            self.get_parameter(
                'encoder_direction_mode'
            ).value
        ).strip().lower()

        self.encoder_glitch_filter_us = int(
            self.get_parameter(
                'encoder_glitch_filter_us'
            ).value
        )

        self.left_encoder_invert = bool(
            self.get_parameter(
                'left_encoder_invert'
            ).value
        )

        self.right_encoder_invert = bool(
            self.get_parameter(
                'right_encoder_invert'
            ).value
        )

        self.swap_encoders = bool(
            self.get_parameter(
                'swap_encoders'
            ).value
        )

        if self.encoder_direction_mode not in [
            'quadrature',
            'commanded',
        ]:

            self.get_logger().warn(
                'Unknown encoder_direction_mode='
                f'{self.encoder_direction_mode}. '
                'Using quadrature.'
            )

            self.encoder_direction_mode = (
                'quadrature'
            )

        # =====================================================
        # pigpio
        # =====================================================

        self.pi = pigpio.pi()

        if not self.pi.connected:

            self.get_logger().error(
                'Could not connect to pigpio daemon. '
                'Run: sudo pigpiod'
            )

            raise SystemExit(1)

        # =====================================================
        # Physical Encoder State
        #
        # These counts belong to the physical GPIO channels.
        #
        # If swap_encoders=True:
        #
        # physical LEFT encoder channel
        #     -> logical RIGHT wheel
        #
        # physical RIGHT encoder channel
        #     -> logical LEFT wheel
        # =====================================================

        self.left_ticks = 0
        self.right_ticks = 0

        # =====================================================
        # Previously Published Logical Encoder State
        # =====================================================

        self.prev_published_left_ticks = 0
        self.prev_published_right_ticks = 0

        self.prev_time = time.time()

        # =====================================================
        # Motor Command State
        # =====================================================

        self.left_cmd = 0.0
        self.right_cmd = 0.0

        self.last_cmd_time = time.time()

        # =====================================================
        # Logical Robot Wheel Directions
        #
        # These correspond to ROBOT LEFT / RIGHT wheels,
        # not physical encoder GPIO channels.
        #
        # Used only in encoder_direction_mode="commanded".
        # =====================================================

        self.left_encoder_direction = 1
        self.right_encoder_direction = 1

        # =====================================================
        # GPIO Setup
        # =====================================================

        self.setup_encoder_gpio()

        # =====================================================
        # ROS Interfaces
        # =====================================================

        self.subscription = (
            self.create_subscription(
                Twist,
                self.cmd_vel_topic,
                self.cmd_vel_callback,
                10,
            )
        )

        self.wheel_pub = (
            self.create_publisher(
                JointState,
                self.wheel_states_topic,
                10,
            )
        )

        # =====================================================
        # Encoder Callbacks
        # =====================================================

        self.left_cb = (
            self.pi.callback(
                LEFT_ENC_A,
                pigpio.RISING_EDGE,
                self.left_encoder_callback,
            )
        )

        self.right_cb = (
            self.pi.callback(
                RIGHT_ENC_A,
                pigpio.RISING_EDGE,
                self.right_encoder_callback,
            )
        )

        # =====================================================
        # Timers
        # =====================================================

        self.create_timer(
            0.02,
            self.publish_wheel_states,
        )

        self.create_timer(
            0.02,
            self.update_pwm,
        )

        # =====================================================
        # Start Neutral
        # =====================================================

        self.set_neutral()

        # =====================================================
        # Startup Log
        # =====================================================

        self.get_logger().info(
            'Motor controller running. '
            f'cmd_vel={self.cmd_vel_topic}, '
            f'wheel_states={self.wheel_states_topic}, '
            f'encoder_direction_mode='
            f'{self.encoder_direction_mode}, '
            f'encoder_glitch_filter_us='
            f'{self.encoder_glitch_filter_us}, '
            f'left_encoder_invert='
            f'{self.left_encoder_invert}, '
            f'right_encoder_invert='
            f'{self.right_encoder_invert}, '
            f'swap_encoders='
            f'{self.swap_encoders}'
        )

    # =========================================================
    # GPIO Setup
    # =========================================================

    def setup_encoder_gpio(
        self,
    ):

        for pin in [
            LEFT_ENC_A,
            LEFT_ENC_B,
            RIGHT_ENC_A,
            RIGHT_ENC_B,
        ]:

            self.pi.set_mode(
                pin,
                pigpio.INPUT,
            )

            self.pi.set_pull_up_down(
                pin,
                pigpio.PUD_UP,
            )

        if (
            self.encoder_glitch_filter_us
            > 0
        ):

            self.pi.set_glitch_filter(
                LEFT_ENC_A,
                self.encoder_glitch_filter_us,
            )

            self.pi.set_glitch_filter(
                RIGHT_ENC_A,
                self.encoder_glitch_filter_us,
            )

    # =========================================================
    # Neutral
    # =========================================================

    def set_neutral(
        self,
    ):

        self.pi.set_servo_pulsewidth(
            LEFT_MOTOR_PIN,
            NEUTRAL,
        )

        self.pi.set_servo_pulsewidth(
            RIGHT_MOTOR_PIN,
            NEUTRAL,
        )

    # =========================================================
    # Encoder Callbacks
    # =========================================================

    def left_encoder_callback(
        self,
        gpio,
        level,
        tick,
    ):

        if level != 1:
            return

        # -----------------------------------------------------
        # Commanded direction mode
        #
        # IMPORTANT:
        #
        # This callback belongs to the physical LEFT encoder
        # GPIO channel.
        #
        # If encoders are swapped, this physical channel
        # represents the logical RIGHT wheel.
        # -----------------------------------------------------

        if (
            self.encoder_direction_mode
            == 'commanded'
        ):

            if self.swap_encoders:

                delta = (
                    self.right_encoder_direction
                )

            else:

                delta = (
                    self.left_encoder_direction
                )

        # -----------------------------------------------------
        # Quadrature direction mode
        #
        # In quadrature mode, direction comes directly from
        # this physical encoder's B channel.
        # -----------------------------------------------------

        else:

            channel_b = (
                self.pi.read(
                    LEFT_ENC_B
                )
            )

            delta = (
                1
                if channel_b == 0
                else -1
            )

        # -----------------------------------------------------
        # Physical channel inversion
        # -----------------------------------------------------

        if self.left_encoder_invert:
            delta *= -1

        self.left_ticks += delta

    def right_encoder_callback(
        self,
        gpio,
        level,
        tick,
    ):

        if level != 1:
            return

        # -----------------------------------------------------
        # Commanded direction mode
        #
        # This callback belongs to the physical RIGHT encoder
        # GPIO channel.
        #
        # If encoders are swapped, this physical channel
        # represents the logical LEFT wheel.
        # -----------------------------------------------------

        if (
            self.encoder_direction_mode
            == 'commanded'
        ):

            if self.swap_encoders:

                delta = (
                    self.left_encoder_direction
                )

            else:

                delta = (
                    self.right_encoder_direction
                )

        # -----------------------------------------------------
        # Quadrature direction mode
        # -----------------------------------------------------

        else:

            channel_b = (
                self.pi.read(
                    RIGHT_ENC_B
                )
            )

            delta = (
                1
                if channel_b == 0
                else -1
            )

        # -----------------------------------------------------
        # Physical channel inversion
        # -----------------------------------------------------

        if self.right_encoder_invert:
            delta *= -1

        self.right_ticks += delta

    # =========================================================
    # cmd_vel Callback
    # =========================================================

    def cmd_vel_callback(
        self,
        msg,
    ):

        self.last_cmd_time = (
            time.time()
        )

        linear = clamp(
            msg.linear.x,
            -1.0,
            1.0,
        )

        angular = clamp(
            msg.angular.z,
            -1.0,
            1.0,
        )

        # -----------------------------------------------------
        # Differential Drive Mixing
        #
        # ROS convention:
        #
        # angular > 0
        #     left turn
        #
        # angular < 0
        #     right turn
        # -----------------------------------------------------

        left_robot_cmd = clamp(
            linear
            - TURN_GAIN * angular,
            -1.0,
            1.0,
        )

        right_robot_cmd = clamp(
            linear
            + TURN_GAIN * angular,
            -1.0,
            1.0,
        )

        # -----------------------------------------------------
        # Logical Wheel Encoder Direction
        #
        # Keep previous direction near zero so coasting encoder
        # pulses do not randomly switch sign.
        # -----------------------------------------------------

        if (
            left_robot_cmd
            > DIRECTION_CMD_THRESHOLD
        ):

            self.left_encoder_direction = 1

        elif (
            left_robot_cmd
            < -DIRECTION_CMD_THRESHOLD
        ):

            self.left_encoder_direction = -1

        if (
            right_robot_cmd
            > DIRECTION_CMD_THRESHOLD
        ):

            self.right_encoder_direction = 1

        elif (
            right_robot_cmd
            < -DIRECTION_CMD_THRESHOLD
        ):

            self.right_encoder_direction = -1

        # -----------------------------------------------------
        # Robot-frame wheel command
        # -> physical motor command
        #
        # Encoder swap does NOT affect motor outputs.
        #
        # swap_encoders describes encoder wiring only.
        # -----------------------------------------------------

        left_motor_cmd = (
            left_robot_cmd
        )

        right_motor_cmd = (
            right_robot_cmd
        )

        if LEFT_MOTOR_INVERT:
            left_motor_cmd *= -1.0

        if RIGHT_MOTOR_INVERT:
            right_motor_cmd *= -1.0

        self.left_cmd = (
            left_motor_cmd
        )

        self.right_cmd = (
            right_motor_cmd
        )

    # =========================================================
    # PWM Update
    # =========================================================

    def update_pwm(
        self,
    ):

        if (
            time.time()
            - self.last_cmd_time
            > CMD_TIMEOUT_SEC
        ):

            self.left_cmd = 0.0
            self.right_cmd = 0.0

        self.pi.set_servo_pulsewidth(
            LEFT_MOTOR_PIN,
            command_to_pulse(
                self.left_cmd
            ),
        )

        self.pi.set_servo_pulsewidth(
            RIGHT_MOTOR_PIN,
            command_to_pulse(
                self.right_cmd
            ),
        )

    # =========================================================
    # Wheel State Publisher
    # =========================================================

    def publish_wheel_states(
        self,
    ):

        now = time.time()

        dt = (
            now
            - self.prev_time
        )

        if dt <= 0.0:
            return

        # -----------------------------------------------------
        # Physical encoder GPIO counts
        # -----------------------------------------------------

        physical_left_ticks = (
            self.left_ticks
        )

        physical_right_ticks = (
            self.right_ticks
        )

        # -----------------------------------------------------
        # Map Physical Encoder Channels
        # -> Logical Robot Wheels
        # -----------------------------------------------------

        if self.swap_encoders:

            published_left_ticks = (
                physical_right_ticks
            )

            published_right_ticks = (
                physical_left_ticks
            )

        else:

            published_left_ticks = (
                physical_left_ticks
            )

            published_right_ticks = (
                physical_right_ticks
            )

        # -----------------------------------------------------
        # Logical Wheel Tick Deltas
        # -----------------------------------------------------

        left_delta = (
            published_left_ticks
            - self.prev_published_left_ticks
        )

        right_delta = (
            published_right_ticks
            - self.prev_published_right_ticks
        )

        # -----------------------------------------------------
        # Logical Wheel Velocity Estimates
        # -----------------------------------------------------

        left_rate = (
            left_delta / dt
        )

        right_rate = (
            right_delta / dt
        )

        # -----------------------------------------------------
        # JointState Message
        # -----------------------------------------------------

        msg = JointState()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.name = [
            'left_wheel',
            'right_wheel',
        ]

        msg.position = [
            float(
                published_left_ticks
            ),
            float(
                published_right_ticks
            ),
        ]

        msg.velocity = [
            float(
                left_rate
            ),
            float(
                right_rate
            ),
        ]

        self.wheel_pub.publish(
            msg
        )

        # -----------------------------------------------------
        # Save Logical Published State
        # -----------------------------------------------------

        self.prev_published_left_ticks = (
            published_left_ticks
        )

        self.prev_published_right_ticks = (
            published_right_ticks
        )

        self.prev_time = now

    # =========================================================
    # Cleanup
    # =========================================================

    def destroy_node(
        self,
    ):

        try:

            self.set_neutral()

            if hasattr(
                self,
                'left_cb',
            ):

                self.left_cb.cancel()

            if hasattr(
                self,
                'right_cb',
            ):

                self.right_cb.cancel()

            self.pi.set_glitch_filter(
                LEFT_ENC_A,
                0,
            )

            self.pi.set_glitch_filter(
                RIGHT_ENC_A,
                0,
            )

            self.pi.stop()

        except Exception:
            pass

        super().destroy_node()


# =============================================================
# Main
# =============================================================

def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = (
        DifferentialDriveNode()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():

            node.destroy_node()

            rclpy.shutdown()


if __name__ == '__main__':
    main()