#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32
from math import pi


class TeleopIK(Node):
    def __init__(self):
        super().__init__("joystick_teleop")
        self.last_joy = -1e10
        self.joy_value = None
        self.state = "Ready"
        self.timeout = False

        self.declare_parameter("~/axis_arm_x",3)
        self.declare_parameter("~/axis_arm_y",6)
        self.declare_parameter("~/axis_arm_z",4)
        self.declare_parameter("~/arm_velocity",0.1)
        self.declare_parameter("~/home_button",1)
        self.declare_parameter("~/ready_button",0)
        self.declare_parameter("~/move_button",2)

        self.axis_arm_x = self.get_parameter("~/axis_arm_x").get_parameter_value().integer_value
        self.axis_arm_y = self.get_parameter("~/axis_arm_y").get_parameter_value().integer_value
        self.axis_arm_z = self.get_parameter("~/axis_arm_z").get_parameter_value().integer_value
        self.arm_step = self.get_parameter("~/arm_velocity").get_parameter_value().double_value
        self.home_button = self.get_parameter("~/home_button").get_parameter_value().integer_value
        self.ready_button = self.get_parameter("~/ready_button").get_parameter_value().integer_value
        self.move_button = self.get_parameter("~/move_button").get_parameter_value().integer_value

        self.sub = self.create_subscription(Joy,'/joy', self.joy_cb,1)
        self.twist_pub = self.create_publisher(Twist,'~/twist_command', 1)
        self.pose_pub = self.create_publisher(Point,'~/position_command', 1)
        self.tool_pub = self.create_publisher(Float32,'~/tool_command', 1)

        self.timer = self.create_timer(0.1,self.timer_cb)

    def joy_cb(self,value):
        now = self.get_clock().now().nanoseconds/1e9
        self.last_joy = now
        self.joy_value = value
        self.joy_value.axes = tuple(list(self.joy_value.axes) + [0.0]*10)

    def timer_cb(self):
        twist = Twist()
        now = self.get_clock().now().nanoseconds/1e9
        if (now - self.last_joy) < 0.5: 
            if self.timeout:
                self.timeout = False
                self.get_logger().info("Teleop Geom: Accepting joystick commands")
            if (self.state == "Homing") or (self.joy_value.buttons[self.home_button]):
                if self.state != "Homing":
                    self.get_logger().info("Homing");
                self.state = "Homing"
                tool = Float32()
                tool.data = 0.0
                point = Point()
                point.x = 0.8
                point.y = 0.0
                point.z = 0.3
                self.tool_pub.publish(tool)
                self.pose_pub.publish(point)
            if (self.state == "Ready") or (self.joy_value.buttons[self.ready_button]):
                if self.state != "Ready":
                    self.get_logger().info("Getting ready");
                self.state = "Ready"
                tool = Float32()
                tool.data = -pi/2
                point = Point()
                point.x = 2.0
                point.y = 0.0
                point.z = -0.5
                self.tool_pub.publish(tool)
                self.pose_pub.publish(point)
            if (self.state == "Default") or (self.joy_value.buttons[self.move_button]):
                self.state = "Default"
                twist = Twist()
                twist.linear.x = -self.joy_value.axes[self.axis_arm_x]*self.arm_step
                twist.linear.y = self.joy_value.axes[self.axis_arm_y]*self.arm_step
                twist.linear.z = self.joy_value.axes[self.axis_arm_z]*self.arm_step
                tool = Float32()
                tool.data = -pi/2
                self.tool_pub.publish(tool)
                self.twist_pub.publish(twist)
        else:
            if not self.timeout:
                self.timeout = True
                self.get_logger().info("Teleop Geom: Timeout: ignoring joystick commands")
                twist = Twist()
                self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    joystick_teleop = TeleopIK()

    rclpy.spin(joystick_teleop)

    joystick_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
