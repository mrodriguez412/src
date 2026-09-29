#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from topic_tools_interfaces.srv import MuxSelect

class MuxTeleop(Node):
    def __init__(self):
        super().__init__("mux_teleop")
        self.mux_proxy = None
        self.joystick_button = 0
        self.joystick_topic = "/joy/topic"
        self.auto_button = 1
        self.auto_topic = "/auto/topic"
        self.last_joy = -1e10

        self.declare_parameter('~/joystick_button', self.joystick_button)
        self.declare_parameter('~/joystick_topic', self.joystick_topic)
        self.declare_parameter('~/auto_button', self.auto_button)
        self.declare_parameter('~/auto_topic', self.auto_topic)

        self.joystick_button = self.get_parameter('~/joystick_button').get_parameter_value().integer_value
        self.joystick_topic = self.get_parameter('~/joystick_topic').get_parameter_value().string_value
        self.auto_button = self.get_parameter('~/auto_button').get_parameter_value().integer_value
        self.auto_topic = self.get_parameter('~/auto_topic').get_parameter_value().string_value

        self.client_futures=None
        self.timer = self.create_timer(0.5,self.timer_cb)
        self.jsub = self.create_subscription(Joy,'joy', self.joy_cb,1)


        self.req = None
        self.mux_proxy = self.create_client(MuxSelect,'select')
        while not self.mux_proxy.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def timer_cb(self):
        if (self.client_futures is None) and (self.req is not None):
            self.get_logger().info("Selecting %s" % self.req.topic)
            self.client_futures = self.mux_proxy.call_async(self.req)
            self.req = None

        if (self.client_futures is not None) and (self.client_futures.done()):
            print("received service result: {}".format(self.client_futures.result()))
            self.client_futures = None

    def send_request(self, topic):
        self.req = MuxSelect.Request()
        self.req.topic = topic

    def joy_cb(self,value):
        now = self.get_clock().now().nanoseconds/1e9
        if len(value.buttons) > max([self.joystick_button,self.auto_button]):
            self.last_joy = now
            try:
                if value.buttons[self.joystick_button]:
                    self.send_request(self.joystick_topic)
                elif value.buttons[self.auto_button]:
                    self.send_request(self.auto_topic)
            except Exception as e:
                self.get_logger().error("Service call failed: %s"%e)


def main(args=None):
    rclpy.init(args=args)

    mux_teleop = MuxTeleop()

    rclpy.spin(mux_teleop)

    mux_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


