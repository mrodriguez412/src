#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time,Duration
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,Pose
from math import atan2, hypot, pi, cos, sin
import message_filters
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs.tf2_geometry_msgs

from vrep_vsv_driver.vsv_kinematics import *


class VSVDriver(Node):
    def __init__(self):
        super().__init__("vsv_driver")
        
        self.declare_parameter("~/min_radius",5.0)
        self.declare_parameter("~/check_timeout",True)

        self.min_radius = self.get_parameter("~/min_radius").get_parameter_value().double_value
        self.check_timeout = self.get_parameter("~/check_timeout").get_parameter_value().bool_value


        self.get_logger().info("Starting vsv driver for vsv")
        self.last_cmd = -1e10
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.steering_pub={}
        self.drive_pub={}
        self.ready = False

        self.kinematics = VSVKinematics(self.min_radius)
        self.get_logger().info("Waiting for initial transforms")
        self.radius={}
        for k in prefix:
            try:
                self.waittf('VSV/ground','VSV/%sDrive'%k,60.)
                t = self.tf_buffer.lookup_transform('VSV/ground','VSV/%sDrive'% k, Time())
                self.radius[k] = t.transform.translation.z
                self.get_logger().info("Got transform for " + k)
            except TransformException as e:
                self.get_logger().error("TF exception: " + repr(e))

        self.twist_sub = self.create_subscription(Twist,'~/twistCommand', self.twist_cb,1)
        # print "Initialising wheel data structure"
        for k in prefix:
            if k in steer_prefix:
                self.steering_pub[k] = self.create_publisher(Float64,"/VSV/%sSteer/command" % k, 1)
            self.drive_pub[k] = self.create_publisher(Float64,"/VSV/%sDrive/command" % k, 1)

        self.timeout = True
        self.ready = True
        self.get_logger().info("VSV Driver: We're ready")

    def waittf(self,tf_from,tf_to,duration):
        t0 = self.get_clock().now().nanoseconds/1e9
        self.get_logger().info("waiting...")
        last_res = ""
        while rclpy.ok():
            rclpy.spin_once(self,timeout_sec=0.1)
            try:
                res = self.tf_buffer.can_transform(tf_from,tf_to,Time(),Duration(seconds=1.),True)
                if res[0]:
                    return True
                if not (res[1] == last_res):
                    self.get_logger().warn(res[1])
                    last_res = res[1]
            except TransformExpection as e:
                self.get_logger().warn(f"{e}")
                self.get_logger().info("Continuing")
            now = self.get_clock().now().nanoseconds/1e9
            if now - t0 > duration:
                return False





    def twist_cb(self,twist):
        if not self.ready:
            return
        # print "Got twist: " + str(twist)
        now = self.get_clock().now().nanoseconds/1e9
        self.last_cmd = now
        # Get the pose of all drives
        drive_cfg={}
        for k in prefix:
            res = self.tf_buffer.can_transform('VSV/ground','VSV/%sDrive'% k, 
                    Time(), Duration(seconds=1.0),True)
            t = self.tf_buffer.lookup_transform('VSV/ground','VSV/%sDrive'% k, rclpy.time.Time())
            drive_cfg[k] = DriveConfiguration(self.radius[k],
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)
        # Now compute for each drive, its rotation speed and steering angle
        motors = self.kinematics.twist_to_motors(twist,drive_cfg)
        self.publish(motors)

    def publish(self, motor):
        for k in prefix:
            f = Float64()
            f.data = motor.drive[k]
            self.drive_pub[k].publish(f)
            if k in steer_prefix:
                f.data = motor.steering[k]
                self.steering_pub[k].publish(f)
            

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds/1e9
        if self.check_timeout:
            if (now - self.last_cmd) < 0.5: 
                if timeout:
                    timeout = False
                    self.get_logger().info("Accepting joystick commands")
            else:
                if not timeout:
                    timeout = True
                    self.get_logger().info("Timeout: ignoring joystick commands")
                motors = VSVMotors()
                self.publish(motors)

def main(args=None):
    rclpy.init(args=args)

    driver = VSVDriver()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
