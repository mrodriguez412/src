#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,PoseStamped, Quaternion
from math import atan2, hypot, pi, cos, sin
import message_filters

from vrep_vsv_driver.vsv_kinematics import *


class VSVDriver(Node):
    def __init__(self):
        super().__init__("vsv_driver")

        self.joint_sub={}
        self.command_pub={}
        self.joint_names = [ "ArmPan",  "ArmTilt",  "ArmFold",  "ArmExtend",  "ToolRotate"]
        self.min_value = {"ArmPan":-pi/2, "ArmTilt":-pi/6, "ArmFold":-pi/2,
                "ArmExtend":0.0, "ToolRotate":-pi/2}
        self.max_value = {"ArmPan":pi/2, "ArmTilt":pi/2, "ArmFold":pi/2,
                "ArmExtend":1.0, "ToolRotate":pi/2}
        self.joint_pub=self.create_publisher(JointState,"/VSV/aggregated/state",1)
        self.command_sub=self.create_subscription(JointState,"/VSV/aggregated/command",self.sync_joint_cmd,1)

        # print "Initialising wheel data structure"
        for k in self.joint_names:
            self.command_pub[k] = self.create_publisher(Float64,"/VSV/%s/command" % k , 1)
            self.joint_sub[k] = message_filters.Subscriber(self, JointState,"/VSV/%s/state" % k)
            # print "Initialised wheel " + k
        self.ts = message_filters.TimeSynchronizer(self.joint_sub.values(), 10)
        self.ts.registerCallback(self.sync_joint_cb)

        self.get_logger().info("VSV Arm: We're ready")


    def sync_joint_cb(self,*args):
        js = JointState()
        js.header = args[0].header
        for j in args:
            js.name+=j.name
            js.position+=j.position
            js.velocity+=j.velocity
            js.effort+=j.effort
        self.joint_pub.publish(js)

    def sat(self,name,value):
        return max(self.min_value[name],min(self.max_value[name],value))

    def sync_joint_cmd(self,js):
        for (name,value) in zip(js.name,js.position):
            c = Float64()
            c.data = self.sat(name,value)
            self.command_pub[name].publish(c)



def main(args=None):
    rclpy.init(args=args)

    driver = VSVDriver()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
