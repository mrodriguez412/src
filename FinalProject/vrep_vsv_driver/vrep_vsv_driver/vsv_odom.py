#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time,Duration
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,PoseStamped, Quaternion
from math import atan2, hypot, pi, cos, sin
import message_filters
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from vrep_vsv_driver.vsv_kinematics import *


class VSVDriver(Node):
    def __init__(self,name):
        super().__init__("vsv_driver")

        self.get_logger().info("Starting vsv odometry")
        self.last_cmd = -1e10
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ready = False
        self.connected = False
        self.steering_sub={}
        self.drive_sub={}
        self.odom_pub=self.create_publisher("~odom",PoseStamped, queue_size=1)

        self.kinematics = VSVKinematics()

        self.get_logger().info("Waiting for initial transforms")
        self.radius={}
        for k in prefix:
            try:
                self.waittf('VSV/ground','VSV/%sDrive'%k,60.)
                t = self.tf_buffer.lookup_transform('VSV/ground','VSV/%sDrive'% k, Time())
                self.radius[k] = t.transformation.translation.z
                self.get_logger().info("Got transform for " + k)
            except TransformException as e:
                self.get_logger().error("TF exception: " + repr(e))

        # print "Initialising wheel data structure"
        for k in prefix:
            if k in steer_prefix:
                self.steering_sub[k] = message_filters.Subscriber("/%s/%sSteer/state" % (self.name,k), JointState)
            self.drive_sub[k] = message_filters.Subscriber("/%s/%sDrive/state" % (self.name,k), JointState)
            # print "Initialised wheel " + k
        self.ts = message_filters.TimeSynchronizer(self.steering_sub.values()+self.drive_sub.values(), 10)
        self.ts.registerCallback(self.sync_odo_cb)
        self.get_logger().info("VSV Odom: We're ready")


    def sync_odo_cb(self,*args):
        self.connected = True
        if not self.ready:
            return
        if len(args)!=6:
            rospy.logerr("Invalid number of argument in OdoCallback")
            return
        steering_val = [s.position[0] for s in args[0:2]]
        drive_val = [s.position[0] for s in args[2:6]]
        motors = VSVMotors()
        motors.steering = dict(zip(self.steering_sub.keys(),steering_val))
        motors.drive = dict(zip(self.drive_sub.keys(),drive_val))
        self.odo_cb(args[0].header.stamp,motors)

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = cos(ai)
        si = sin(ai)
        cj = cos(aj)
        sj = sin(aj)
        ck = cos(ak)
        sk = sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def odo_cb(self,timestamp,motors):
        # Get the pose of all drives
        drive_cfg={}
        for k in prefix:
            t = self.tf_buffer.lookup_transform('VSV/ground','VSV/%sDrive'% k, rclpy.Time())
            drive_cfg[k] = DriveConfiguration(self.radius[k],
                    t.transformation.translation.x,
                    t.transformation.translation.y,
                    t.transformation.translation.z)
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #    return
        X = self.kinematics.integrate_odometry(motors, drive_cfg)
        pose = PoseStamped()
        pose.header.stamp = timestamp
        pose.header.frame_id = "world"
        pose.pose.position.x = X[0,0]
        pose.pose.position.y = X[1,0]
        Q = self.quaternion_from_euler(0, 0, X[2,0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        self.odom_pub.publish(pose)
        # finally store the value of the motor state



def main(args=None):
    rclpy.init(args=args)

    driver = VSVDriver()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
