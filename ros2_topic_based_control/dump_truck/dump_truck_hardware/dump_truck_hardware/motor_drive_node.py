#!/usr/bin/env python3
"""
motor_drive_node.py

PWM motor controller + encoder wheel-state publisher for one dump truck.

Features
--------
- Truck-specific ROS topics.
- Encoder direction can be assigned from commanded wheel direction.
- pigpio glitch filtering.
- Optional encoder left/right swap for trucks whose physical encoder
  wiring / assembly is reversed.

The important distinction is:

    physical_left_ticks
    physical_right_ticks

are counted internally according to the GPIO assignments.

When swap_encoders=True, they are exchanged ONLY when publishing
JointState so downstream odometry receives:

    msg.position[0] = actual robot LEFT wheel
    msg.position[1] = actual robot RIGHT wheel
"""

import time

import pigpio
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


# =============================================================
# Motor PWM pins
# BCM numbering
# =============================================================

LEFT_MOTOR_PIN = 25
RIGHT_MOTOR_PIN = 5


# =============================================================
# Motor electrical inversion
# =============================================================

LEFT_MOTOR_INVERT = False
RIGHT_MOTOR_INVERT = False


# =============================================================
# Encoder pins
# BCM numbering
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
# Drive tuning
# =============================================================

TURN_GAIN = 0.5

MIN_EFFECTIVE_CMD = 0.18

CMD_TIMEOUT_SEC = 0.5

DIRECTION_CMD_THRESHOLD = 0.03


# =============================================================
# Helper functions
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
# Motor Drive Node
# =============================================================

class DifferentialDriveNode(Node):

    def __init__(self):

        super().__init__(
            'dumptruck_differential_drive'
        )

        # =====================================================
        # ROS parameters
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
        # Read parameters
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
        # Encoder state
        #
        # These represent encoder counts according to the
        # PHYSICAL GPIO channels.
        #
        # They may later be swapped before publishing.
        # =====================================================

        self.left_ticks = 0
        self.right_ticks = 0

        self.prev_published_left_ticks = 0
        self.prev_published_right_ticks = 0

        self.prev_time = time.time()

        # =====================================================
        # Motor command state
        # =====================================================

        self.left_cmd = 0.0
        self.right_cmd = 0.0

        self.last_cmd_time = time.time()

        # =====================================================
        # Commanded encoder direction
        #
        # Used only when:
        #
        # encoder_direction_mode == commanded
        #
        # Direction is expressed in the ROBOT FRAME before
        # electrical motor inversion.
        # =====================================================

        self.left_encoder_direction = 1
        self.right_encoder_direction = 1

        # =====================================================
        # GPIO setup
        # =====================================================

        self.setup_encoder_gpio()

        # =====================================================
        # ROS interfaces
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
        # Encoder callbacks
        # =====================================================

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
        # Start neutral
        # =====================================================

        self.set_neutral()

        # =====================================================
        # Startup log
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
    # GPIO
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

        if self.encoder_glitch_filter_us > 0:

            self.pi.set_glitch_filter(
                LEFT_ENC_A,
                self.encoder_glitch_filter_us,
            )

            self.pi.set_glitch_filter(
                RIGHT_ENC_A,
                self.encoder_glitch_filter_us,
            )

    # =========================================================
    # Motor neutral
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
    # Encoder callbacks
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
        # Command-derived encoder direction
        # -----------------------------------------------------

        if (
            self.encoder_direction_mode
            == 'commanded'
        ):

            delta = (
                self.left_encoder_direction
            )

        # -----------------------------------------------------
        # Quadrature-derived direction
        # -----------------------------------------------------

        else:

            channel_b = self.pi.read(
                LEFT_ENC_B
            )

            delta = (
                1
                if channel_b == 0
                else -1
            )

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
        # Command-derived encoder direction
        # -----------------------------------------------------

        if (
            self.encoder_direction_mode
            == 'commanded'
        ):

            delta = (
                self.right_encoder_direction
            )

        # -----------------------------------------------------
        # Quadrature-derived direction
        # -----------------------------------------------------

        else:

            channel_b = self.pi.read(
                RIGHT_ENC_B
            )

            delta = (
                1
                if channel_b == 0
                else -1
            )

        if self.right_encoder_invert:
            delta *= -1

        self.right_ticks += delta

    # =========================================================
    # cmd_vel
    # =========================================================

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

        # -----------------------------------------------------
        # Differential-drive mixing
        #
        # ROS convention:
        #
        # angular > 0
        #     turn left
        #
        # angular < 0
        #     turn right
        #
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
        # Encoder direction in robot frame
        #
        # Keep previous sign close to zero so encoder coasting
        # does not randomly change sign.
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
        # Convert robot-frame commands to electrical motor
        # commands.
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
    # PWM update
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
    # Wheel state publisher
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
        # Optional physical encoder swap
        #
        # This corrects trucks where the physical encoder
        # channels are wired / assembled opposite to the
        # logical robot LEFT / RIGHT wheel names.
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
        # Published tick deltas
        #
        # Important:
        # Calculate velocity AFTER the optional swap so
        # position and velocity refer to the same logical wheel.
        # -----------------------------------------------------

        left_delta = (
            published_left_ticks
            - self.prev_published_left_ticks
        )

        right_delta = (
            published_right_ticks
            - self.prev_published_right_ticks
        )

        left_rate = (
            left_delta / dt
        )

        right_rate = (
            right_delta / dt
        )

        # -----------------------------------------------------
        # JointState
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
        # Save published state
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