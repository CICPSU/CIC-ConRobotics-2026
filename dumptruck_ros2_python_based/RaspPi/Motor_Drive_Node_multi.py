#!/usr/bin/env python3
"""
Motor_Drive_Node_multi.py

PWM motor controller + encoder wheel state publisher for one dump truck.

Use one copy of this process per physical truck/Pi.
Topics are parameterized so multiple trucks can share the same ROS_DOMAIN_ID:

  /truck1/cmd_vel      -> input
  /truck1/wheel_states -> output

Example:
  python3 Motor_Drive_Node_multi.py --ros-args \
    -p cmd_vel_topic:=/truck1/cmd_vel \
    -p wheel_states_topic:=/truck1/wheel_states \
    -p node_name_suffix:=truck1
"""

import time
import pigpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# ============================
# Motor PWM Pins
# ============================
LEFT_MOTOR_PIN = 25
RIGHT_MOTOR_PIN = 5

LEFT_MOTOR_INVERT = False
RIGHT_MOTOR_INVERT = False

# ============================
# Encoder Pins
# ============================
RIGHT_ENC_A = 18
RIGHT_ENC_B = 27
LEFT_ENC_A = 19
LEFT_ENC_B = 20

LEFT_ENCODER_INVERT = False
RIGHT_ENCODER_INVERT = False

# ============================
# ESC / Servo PWM
# ============================
NEUTRAL = 1500
MAX_FORWARD = 2000
MAX_REVERSE = 1000

# ============================
# Drive Tuning
# ============================
TURN_GAIN = 0.5
MIN_EFFECTIVE_CMD = 0.18
CMD_TIMEOUT_SEC = 0.5


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def apply_deadband_boost(cmd):
    if 0.0 < cmd < MIN_EFFECTIVE_CMD:
        return MIN_EFFECTIVE_CMD
    if -MIN_EFFECTIVE_CMD < cmd < 0.0:
        return -MIN_EFFECTIVE_CMD
    return cmd


def command_to_pulse(cmd):
    cmd = clamp(cmd, -1.0, 1.0)
    if abs(cmd) > 1e-6:
        cmd = apply_deadband_boost(cmd)
    pulse = NEUTRAL + cmd * 500.0
    return int(clamp(pulse, MAX_REVERSE, MAX_FORWARD))


class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('dumptruck_differential_drive')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_states_topic', '/wheel_states')
        self.declare_parameter('node_name_suffix', '')

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.wheel_states_topic = str(self.get_parameter('wheel_states_topic').value)
        self.node_name_suffix = str(self.get_parameter('node_name_suffix').value)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon. Run: sudo pigpiod")
            raise SystemExit(1)

        self.setup_encoder_gpio()

        self.subscription = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )
        self.wheel_pub = self.create_publisher(JointState, self.wheel_states_topic, 10)

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = time.time()

        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.last_cmd_time = time.time()

        self.left_cb = self.pi.callback(LEFT_ENC_A, pigpio.RISING_EDGE, self.left_encoder_callback)
        self.right_cb = self.pi.callback(RIGHT_ENC_A, pigpio.RISING_EDGE, self.right_encoder_callback)

        self.create_timer(0.02, self.publish_wheel_states)
        self.create_timer(0.02, self.update_pwm)

        self.get_logger().info(
            f"Motor controller running. cmd_vel={self.cmd_vel_topic}, wheel_states={self.wheel_states_topic}"
        )

    def setup_encoder_gpio(self):
        for pin in [LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)

    def set_neutral(self):
        self.pi.set_servo_pulsewidth(LEFT_MOTOR_PIN, NEUTRAL)
        self.pi.set_servo_pulsewidth(RIGHT_MOTOR_PIN, NEUTRAL)

    def left_encoder_callback(self, gpio, level, tick):
        if level == 2:
            return
        b = self.pi.read(LEFT_ENC_B)
        delta = 1 if b == 0 else -1
        if LEFT_ENCODER_INVERT:
            delta *= -1
        self.left_ticks += delta

    def right_encoder_callback(self, gpio, level, tick):
        if level == 2:
            return
        b = self.pi.read(RIGHT_ENC_B)
        delta = 1 if b == 0 else -1
        if RIGHT_ENCODER_INVERT:
            delta *= -1
        self.right_ticks += delta

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        linear = clamp(msg.linear.x, -1.0, 1.0)
        angular = clamp(msg.angular.z, -1.0, 1.0)

        left = clamp(linear - TURN_GAIN * angular, -1.0, 1.0)
        right = clamp(linear + TURN_GAIN * angular, -1.0, 1.0)

        if LEFT_MOTOR_INVERT:
            left *= -1.0
        if RIGHT_MOTOR_INVERT:
            right *= -1.0

        self.left_cmd = left
        self.right_cmd = right

    def update_pwm(self):
        if time.time() - self.last_cmd_time > CMD_TIMEOUT_SEC:
            self.left_cmd = 0.0
            self.right_cmd = 0.0

        self.pi.set_servo_pulsewidth(LEFT_MOTOR_PIN, command_to_pulse(self.left_cmd))
        self.pi.set_servo_pulsewidth(RIGHT_MOTOR_PIN, command_to_pulse(self.right_cmd))

    def publish_wheel_states(self):
        now = time.time()
        dt = now - self.prev_time
        if dt <= 0.0:
            return

        left_delta = self.left_ticks - self.prev_left_ticks
        right_delta = self.right_ticks - self.prev_right_ticks

        left_rate = left_delta / dt
        right_rate = right_delta / dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel', 'right_wheel']
        msg.position = [float(self.left_ticks), float(self.right_ticks)]
        msg.velocity = [float(left_rate), float(right_rate)]

        self.wheel_pub.publish(msg)

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_time = now

    def destroy_node(self):
        try:
            self.set_neutral()
            if hasattr(self, 'left_cb'):
                self.left_cb.cancel()
            if hasattr(self, 'right_cb'):
                self.right_cb.cancel()
            self.pi.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()
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
