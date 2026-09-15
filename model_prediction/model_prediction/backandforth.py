#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sin,cos,pi
from geometry_msgs.msg import Twist

class BackAndForth(Node):
    def __init__(self):
        super().__init__("back_and_forth")
        self.pub = self.create_publisher(Twist,"/vrep/twistCommand",1)

        self.t0 = self.get_clock().now().nanoseconds/1e9
        self.timer = self.create_timer(0.1,self.timer_cb)

    def timer_cb(self):
        t = self.get_clock().now().nanoseconds/1e9
        twist=Twist()
        if cos((t-self.t0)*pi/3) >= 0:
            twist.linear.x=0.5
        else:
            twist.linear.x=-0.5
        self.pub.publish(twist);


def main(args=None):
    rclpy.init(args=args)

    driver = BackAndForth()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
