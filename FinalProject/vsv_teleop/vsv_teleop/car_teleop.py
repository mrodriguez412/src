#!/usr/bin/env python3
"""Car-like joystick teleop for the VSV truck (Ackermann steering).

The stock teleop maps the stick to an angular rate. vsv_driver clamps the
turning radius v/w to min_radius, so any steering above ~20 % already gives
full lock. Here the stick sets the *curvature*, like a steering wheel:

    v     = max_speed * stick_y           (rate limited by max_accel)
    kappa = stick_x / min_radius          (rate limited by max_steer_rate)
    w     = v * kappa

Half stick is half lock at any speed, and reversing steers like a car.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def shape(value, deadzone, expo):
    """Deadzone, then expo curve: fine control near centre, full range at the end."""
    a = abs(value)
    if a < deadzone:
        return 0.0
    a = (a - deadzone) / (1.0 - deadzone)
    return math.copysign(min(1.0, a ** expo), value)


def approach(current, target, max_step):
    return current + max(-max_step, min(max_step, target - current))


class CarTeleop(Node):
    def __init__(self):
        super().__init__('car_teleop')
        p = self.declare_parameter
        self.axis_speed = p('axis_speed', 1).value
        self.axis_steer = p('axis_steer', 0).value
        self.max_speed = p('max_speed', 1.5).value          # m/s
        self.min_radius = p('min_radius', 5.0).value        # m, same as vsv_driver
        self.max_accel = p('max_accel', 1.0).value          # m/s^2
        self.max_steer_rate = p('max_steer_rate', 0.3).value  # 1/m per s
        self.deadzone = p('deadzone', 0.1).value
        self.speed_expo = p('speed_expo', 1.5).value
        self.steer_expo = p('steer_expo', 1.3).value
        self.invert_steer = p('invert_steer', False).value
        self.timeout = p('timeout', 0.5).value
        self.dt = 0.05

        self.joy = None
        self.last_joy = -1e9
        self.v = 0.0
        self.kappa = 0.0
        self.active = False
        self.create_subscription(Joy, '/joy', self.joy_cb, 1)
        self.pub = self.create_publisher(Twist, '~/twistCommand', 1)
        self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info('Car teleop: speed axis %d, steer axis %d, max %.1f m/s, '
                               'min radius %.1f m' % (self.axis_speed, self.axis_steer,
                                                      self.max_speed, self.min_radius))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def joy_cb(self, msg):
        if len(msg.axes) > max(self.axis_speed, self.axis_steer):
            self.joy = msg
            self.last_joy = self.now()

    def timer_cb(self):
        if self.joy is not None and self.now() - self.last_joy < self.timeout:
            if not self.active:
                self.get_logger().info('Accepting joystick commands')
                self.active = True
            v_target = self.max_speed * shape(self.joy.axes[self.axis_speed],
                                              self.deadzone, self.speed_expo)
            steer = shape(self.joy.axes[self.axis_steer], self.deadzone, self.steer_expo)
            k_target = (-steer if self.invert_steer else steer) / self.min_radius
        else:
            if self.active:
                self.get_logger().info('Joystick timeout: stopping')
                self.active = False
            v_target, k_target = 0.0, self.kappa  # brake, keep the wheels where they are
        self.v = approach(self.v, v_target, self.max_accel * self.dt)
        self.kappa = approach(self.kappa, k_target, self.max_steer_rate * self.dt)
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = self.v * self.kappa
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CarTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
