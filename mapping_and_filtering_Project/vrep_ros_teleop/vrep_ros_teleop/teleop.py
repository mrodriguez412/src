#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__("joystick_teleop")

        self.timeout = True
        self.last_joy = -1e10
        self.joy_value = None
        self.declare_parameter('~/axis_linear_x', 1)
        self.declare_parameter('~/axis_linear_y', 2)
        self.declare_parameter('~/axis_angular', 0)
        self.declare_parameter('~/scale_linear_x', 1.0)
        self.declare_parameter('~/scale_linear_y', 0.0)
        self.declare_parameter('~/scale_angular', 1.0)
        self.declare_parameter('~/timeout', 0.5)

        self.axis_linear_x = self.get_parameter('~/axis_linear_x').get_parameter_value().integer_value
        self.axis_linear_y = self.get_parameter('~/axis_linear_y').get_parameter_value().integer_value
        self.axis_angular = self.get_parameter('~/axis_angular').get_parameter_value().integer_value
        self.scale_linear_x = self.get_parameter('~/scale_linear_x').get_parameter_value().double_value
        self.scale_linear_y = self.get_parameter('~/scale_linear_y').get_parameter_value().double_value
        self.scale_angular = self.get_parameter('~/scale_angular').get_parameter_value().double_value
        self.timeout_delay = self.get_parameter('~/timeout').get_parameter_value().double_value

        self.get_logger().info("Axis %d %d %d SCale %f %f %f" %(self.axis_linear_x,self.axis_linear_y,self.axis_angular,self.scale_linear_x,self.scale_linear_y,self.scale_angular))

        self.sub = self.create_subscription(Joy,'/joy', self.joy_cb,1)
        self.pub = self.create_publisher(Twist,'~/twistCommand', 1)

        self.timer = self.create_timer(0.1,self.timer_cb)

    def joy_cb(self,value):
        now = self.get_clock().now().nanoseconds/1e9
        if len(value.axes) > max([self.axis_linear_x,self.axis_linear_y,self.axis_angular]):
            self.last_joy = now
            self.joy_value = value
        else:
            self.joy_value = None

    def timer_cb(self):
        twist = Twist()
        now = self.get_clock().now().nanoseconds/1e9
        if (self.timeout_delay < 0) or ((now - self.last_joy) < self.timeout_delay): 
            if self.timeout:
                self.timeout = False
                self.get_logger().info("Accepting joystick commands")
            twist.linear.x = self.joy_value.axes[self.axis_linear_x] * self.scale_linear_x
            twist.linear.y = self.joy_value.axes[self.axis_linear_y] * self.scale_linear_y
            twist.angular.z = self.joy_value.axes[self.axis_angular] * self.scale_angular
            if twist.linear.x < 0:
                twist.angular.z = - twist.angular.z
        else:
            if not self.timeout:
                self.timeout = True
                self.get_logger().info("Timeout: ignoring joystick commands")
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    joystick_teleop = JoystickTeleop()

    rclpy.spin(joystick_teleop)

    joystick_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
