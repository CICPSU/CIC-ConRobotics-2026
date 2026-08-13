#!/usr/bin/env python3
"""
motor_drive_node.py

PWM motor controller + encoder wheel state publisher for one dump truck.

Two encoder direction modes are supported:

1. quadrature
   - Original behavior.
   - Direction is determined by reading encoder channel B
     whenever channel A rises.
   - Recommended for trucks whose A/B encoder signals are stable.

2. commanded
   - Direction is inferred from the commanded wheel direction.
   - Only encoder channel A is used for pulse counting.
   - Useful when channel B is electrically unstable/noisy.
   - Recommended for Truck 4 / Truck 5.

Truck-specific behavior can be selected from the hardware YAML:

motor_drive_node:
  ros__parameters:
    encoder_direction_mode: commanded
    encoder_glitch_filter_us: 200
"""

import time

import pigpio
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ============================================================
# Motor PWM Pins
# ============================================================

LEFT_MOTOR_PIN = 25
RIGHT_MOTOR_PIN = 5

LEFT_MOTOR_INVERT = False
RIGHT_MOTOR_INVERT = False


# ============================================================
# Encoder Pins
# ============================================================

RIGHT_ENC_A = 18
RIGHT_ENC_B = 27

LEFT_ENC_A = 19
LEFT_ENC_B = 20


# ============================================================
# Encoder Calibration
# ============================================================

LEFT_ENCODER_INVERT = False
RIGHT_ENCODER_INVERT = False


# ============================================================
# ESC / Servo PWM
# ============================================================

NEUTRAL = 1500
MAX_FORWARD = 2000
MAX_REVERSE = 1000


# ============================================================
# Drive Tuning
# ============================================================

TURN_GAIN = 0.5
MIN_EFFECTIVE_CMD = 0.18
CMD_TIMEOUT_SEC = 0.5

# When using encoder_direction_mode="commanded",
# wheel commands smaller than this are ignored when changing
# the remembered encoder direction.
DIRECTION_CMD_THRESHOLD = 0.03


# ============================================================
# Helper Functions
# ============================================================

def clamp(value, min_value, max_value):
    return max(
        min(value, max_value),
        min_value,
    )


def apply_deadband_boost(command):
    if 0.0 < command < MIN_EFFECTIVE_CMD:
        return MIN_EFFECTIVE_CMD

    if -MIN_EFFECTIVE_CMD < command < 0.0:
        return -MIN_EFFECTIVE_CMD

    return command


def command_to_pulse(command):
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


# ============================================================
# Differential Drive Node
# ============================================================

class DifferentialDriveNode(Node):

    def __init__(self):

        super().__init__(
            'dumptruck_differential_drive'
        )

        # ----------------------------------------------------
        # ROS parameters
        # ----------------------------------------------------

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel',
        )

        self.declare_parameter(
            'wheel_states_topic',
            '/wheel_states',
        )

        self.declare_parameter(
            'node_name_suffix',
            '',
        )

        # Encoder direction mode:
        #
        #   quadrature
        #       Use encoder B phase.
        #
        #   commanded
        #       Use commanded wheel direction.
        #
        self.declare_parameter(
            'encoder_direction_mode',
            'quadrature',
        )

        # pigpio glitch filter in microseconds.
        #
        # 0 disables the filter.
        #
        # Truck 4 / 5:
        #   recommended starting value = 200
        #
        self.declare_parameter(
            'encoder_glitch_filter_us',
            0,
        )

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

        self.node_name_suffix = str(
            self.get_parameter(
                'node_name_suffix'
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

        # ----------------------------------------------------
        # Validate encoder mode
        # ----------------------------------------------------

        valid_modes = [
            'quadrature',
            'commanded',
        ]

        if (
            self.encoder_direction_mode
            not in valid_modes
        ):
            raise RuntimeError(
                'Invalid encoder_direction_mode: '
                f'{self.encoder_direction_mode}. '
                'Use "quadrature" or "commanded".'
            )

        if self.encoder_glitch_filter_us < 0:
            raise RuntimeError(
                'encoder_glitch_filter_us '
                'must be >= 0.'
            )

        # ----------------------------------------------------
        # pigpio
        # ----------------------------------------------------

        self.pi = pigpio.pi()

        if not self.pi.connected:
            self.get_logger().error(
                'Could not connect to pigpio daemon. '
                'Run: sudo pigpiod'
            )
            raise SystemExit(1)

        # ----------------------------------------------------
        # Encoder state
        # ----------------------------------------------------

        self.left_ticks = 0
        self.right_ticks = 0

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.prev_time = time.time()

        # ----------------------------------------------------
        # Motor command state
        # ----------------------------------------------------

        self.left_cmd = 0.0
        self.right_cmd = 0.0

        self.last_cmd_time = time.time()

        # ----------------------------------------------------
        # Command-based encoder direction
        #
        # Used only when:
        #
        #   encoder_direction_mode == "commanded"
        #
        # Keep the last known direction while stopping or
        # coasting.
        # ----------------------------------------------------

        self.left_encoder_direction = 1
        self.right_encoder_direction = 1

        # ----------------------------------------------------
        # GPIO setup
        # ----------------------------------------------------

        self.setup_encoder_gpio()

        # ----------------------------------------------------
        # ROS interfaces
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Encoder callbacks
        # ----------------------------------------------------

        self.left_cb = self.pi.callback(
            LEFT_ENC_A,
            pigpio.RISING_EDGE,
            self.left_encoder_callback,
        )

        self.right_cb = self.pi.callback(
            RIGHT_ENC_A,
            pigpio.RISING_EDGE,
            self.right_encoder_callback,
        )

        # ----------------------------------------------------
        # Timers
        # ----------------------------------------------------

        self.create_timer(
            0.02,
            self.publish_wheel_states,
        )

        self.create_timer(
            0.02,
            self.update_pwm,
        )

        # ----------------------------------------------------
        # Initial motor state
        # ----------------------------------------------------

        self.set_neutral()

        # ----------------------------------------------------
        # Startup log
        # ----------------------------------------------------

        self.get_logger().info(
            'Motor controller running. '
            f'cmd_vel={self.cmd_vel_topic}, '
            f'wheel_states={self.wheel_states_topic}, '
            f'encoder_direction_mode='
            f'{self.encoder_direction_mode}, '
            f'encoder_glitch_filter_us='
            f'{self.encoder_glitch_filter_us}'
        )

    # ========================================================
    # GPIO Setup
    # ========================================================

    def setup_encoder_gpio(self):

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

        # ----------------------------------------------------
        # Optional glitch filtering
        #
        # Applied to channel A because A generates the
        # callbacks/pulse counts.
        # ----------------------------------------------------

        self.pi.set_glitch_filter(
            LEFT_ENC_A,
            self.encoder_glitch_filter_us,
        )

        self.pi.set_glitch_filter(
            RIGHT_ENC_A,
            self.encoder_glitch_filter_us,
        )

    # ========================================================
    # Motor Neutral
    # ========================================================

    def set_neutral(self):

        self.pi.set_servo_pulsewidth(
            LEFT_MOTOR_PIN,
            NEUTRAL,
        )

        self.pi.set_servo_pulsewidth(
            RIGHT_MOTOR_PIN,
            NEUTRAL,
        )

    # ========================================================
    # Left Encoder
    # ========================================================

    def left_encoder_callback(
        self,
        gpio,
        level,
        tick,
    ):

        if level != 1:
            return

        # ----------------------------------------------------
        # Mode 1:
        # Quadrature direction detection
        # ----------------------------------------------------

        if (
            self.encoder_direction_mode
            == 'quadrature'
        ):

            b = self.pi.read(
                LEFT_ENC_B
            )

            delta = (
                1
                if b == 0
                else -1
            )

        # ----------------------------------------------------
        # Mode 2:
        # Commanded direction
        # ----------------------------------------------------

        else:

            delta = (
                self.left_encoder_direction
            )

        # ----------------------------------------------------
        # Optional encoder inversion
        # ----------------------------------------------------

        if LEFT_ENCODER_INVERT:
            delta *= -1

        self.left_ticks += delta

    # ========================================================
    # Right Encoder
    # ========================================================

    def right_encoder_callback(
        self,
        gpio,
        level,
        tick,
    ):

        if level != 1:
            return

        # ----------------------------------------------------
        # Mode 1:
        # Quadrature direction detection
        # ----------------------------------------------------

        if (
            self.encoder_direction_mode
            == 'quadrature'
        ):

            b = self.pi.read(
                RIGHT_ENC_B
            )

            delta = (
                1
                if b == 0
                else -1
            )

        # ----------------------------------------------------
        # Mode 2:
        # Commanded direction
        # ----------------------------------------------------

        else:

            delta = (
                self.right_encoder_direction
            )

        # ----------------------------------------------------
        # Optional encoder inversion
        # ----------------------------------------------------

        if RIGHT_ENCODER_INVERT:
            delta *= -1

        self.right_ticks += delta

    # ========================================================
    # cmd_vel Callback
    # ========================================================

    def cmd_vel_callback(
        self,
        msg,
    ):

        self.last_cmd_time = time.time()

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

        # ----------------------------------------------------
        # Robot-frame wheel commands
        #
        # These are calculated BEFORE any electrical motor
        # inversion.
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Command-based encoder direction
        #
        # Only relevant when encoder_direction_mode is
        # "commanded".
        #
        # Small near-zero commands do NOT change the stored
        # direction. This prevents direction flipping while
        # the truck is stopping/coasting.
        # ----------------------------------------------------

        if (
            self.encoder_direction_mode
            == 'commanded'
        ):

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

        # ----------------------------------------------------
        # Electrical motor commands
        # ----------------------------------------------------

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

    # ========================================================
    # PWM Update
    # ========================================================

    def update_pwm(self):

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

    # ========================================================
    # Wheel State Publisher
    # ========================================================

    def publish_wheel_states(self):

        now = time.time()

        dt = (
            now
            - self.prev_time
        )

        if dt <= 0.0:
            return

        left_delta = (
            self.left_ticks
            - self.prev_left_ticks
        )

        right_delta = (
            self.right_ticks
            - self.prev_right_ticks
        )

        left_rate = (
            left_delta
            / dt
        )

        right_rate = (
            right_delta
            / dt
        )

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
                self.left_ticks
            ),
            float(
                self.right_ticks
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

        self.prev_left_ticks = (
            self.left_ticks
        )

        self.prev_right_ticks = (
            self.right_ticks
        )

        self.prev_time = now

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

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

            # Disable glitch filters before closing pigpio.
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


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = DifferentialDriveNode()

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