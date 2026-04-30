#!/usr/bin/env python3
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

# Flip these if one side moves backward when it should move forward
LEFT_MOTOR_INVERT = False
RIGHT_MOTOR_INVERT = False

# ============================
# Encoder Pins
# ============================
RIGHT_ENC_A = 18
RIGHT_ENC_B = 27
LEFT_ENC_A = 19
LEFT_ENC_B = 20

# Flip these if forward motion makes tick counts go down
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

pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpio daemon")
    raise SystemExit(1)


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

        self.setup_encoder_gpio()

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.wheel_pub = self.create_publisher(JointState, '/wheel_states', 10)

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = time.time()

        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.last_cmd_time = time.time()

        self.left_cb = pi.callback(LEFT_ENC_A, pigpio.RISING_EDGE, self.left_encoder_callback)
        self.right_cb = pi.callback(RIGHT_ENC_A, pigpio.RISING_EDGE, self.right_encoder_callback)

        self.create_timer(0.02, self.publish_wheel_states)
        self.create_timer(0.02, self.update_pwm)

        self.get_logger().info("Motor controller running (PWM + encoders + /wheel_states)")

    def setup_encoder_gpio(self):
        pi.set_mode(LEFT_ENC_A, pigpio.INPUT)
        pi.set_mode(LEFT_ENC_B, pigpio.INPUT)
        pi.set_mode(RIGHT_ENC_A, pigpio.INPUT)
        pi.set_mode(RIGHT_ENC_B, pigpio.INPUT)

        pi.set_pull_up_down(LEFT_ENC_A, pigpio.PUD_UP)
        pi.set_pull_up_down(LEFT_ENC_B, pigpio.PUD_UP)
        pi.set_pull_up_down(RIGHT_ENC_A, pigpio.PUD_UP)
        pi.set_pull_up_down(RIGHT_ENC_B, pigpio.PUD_UP)

    def set_neutral(self):
        pi.set_servo_pulsewidth(LEFT_MOTOR_PIN, NEUTRAL)
        pi.set_servo_pulsewidth(RIGHT_MOTOR_PIN, NEUTRAL)

    def left_encoder_callback(self, gpio, level, tick):
        if level == 2:
            return
        b = pi.read(LEFT_ENC_B)
        delta = 1 if b == 0 else -1
        if LEFT_ENCODER_INVERT:
            delta *= -1
        self.left_ticks += delta

    def right_encoder_callback(self, gpio, level, tick):
        if level == 2:
            return
        b = pi.read(RIGHT_ENC_B)
        delta = 1 if b == 0 else -1
        if RIGHT_ENCODER_INVERT:
            delta *= -1
        self.right_ticks += delta

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        linear = clamp(msg.linear.x, -1.0, 1.0)
        angular = clamp(msg.angular.z, -1.0, 1.0)

        left = linear - TURN_GAIN * angular
        right = linear + TURN_GAIN * angular

        left = clamp(left, -1.0, 1.0)
        right = clamp(right, -1.0, 1.0)

        if LEFT_MOTOR_INVERT:
            left *= -1.0
        if RIGHT_MOTOR_INVERT:
            right *= -1.0

        self.left_cmd = left
        self.right_cmd = right

        self.get_logger().info(
            f"cmd_vel: lin={linear:.2f}, ang={angular:.2f} | "
            f"mixed: left={self.left_cmd:.2f}, right={self.right_cmd:.2f}",
            throttle_duration_sec=0.3
        )

    def update_pwm(self):
        if time.time() - self.last_cmd_time > CMD_TIMEOUT_SEC:
            left_pulse = NEUTRAL
            right_pulse = NEUTRAL
        else:
            left_pulse = command_to_pulse(self.left_cmd)
            right_pulse = command_to_pulse(self.right_cmd)

        pi.set_servo_pulsewidth(LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(RIGHT_MOTOR_PIN, right_pulse)

    def publish_wheel_states(self):
        now = time.time()
        dt = now - self.prev_time
        if dt <= 0.0:
            return

        dl = self.left_ticks - self.prev_left_ticks
        dr = self.right_ticks - self.prev_right_ticks

        vel_l = dl / dt
        vel_r = dr / dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel', 'right_wheel']
        msg.position = [float(self.left_ticks), float(self.right_ticks)]
        msg.velocity = [float(vel_l), float(vel_r)]

        self.wheel_pub.publish(msg)

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_time = now

        self.get_logger().info(
            f"wheel_states: ticks L={self.left_ticks} R={self.right_ticks} | "
            f"vel L={vel_l:.2f} R={vel_r:.2f}",
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_neutral()
        time.sleep(0.5)
        pi.set_servo_pulsewidth(LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(RIGHT_MOTOR_PIN, 0)

        if hasattr(node, 'left_cb') and node.left_cb is not None:
            node.left_cb.cancel()
        if hasattr(node, 'right_cb') and node.right_cb is not None:
            node.right_cb.cancel()

        pi.stop()

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
