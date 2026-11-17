#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, Vector3, Point
from std_msgs.msg import Float32
from math import pi,sqrt
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs.tf2_geometry_msgs
import tf2_kdl

from vrep_vsv_driver.arm_ik import *

def sat(val, center, step):
    if val > center + step:
        return center + step
    elif val < center - step:
        return center - step
    else:
        return val

def angle(val, modulo=2*pi):
    return ((val+2*modulo+modulo/2)%modulo)-modulo/2


class VSVArmIK(Node):
    def __init__(self):
        super().__init__("vsv_arm_ik")

        self.ready = False
        self.last_twist = -1e10
        self.twist_value = None
        self.joint_state = None
        self.tool_orientation = -pi/2
        self.dt = 0.1

        self.declare_parameter("~/max_velocity",0.1)
        self.max_velocity = self.get_parameter("~/max_velocity").get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        try:
            if not self.waittf('VSV/ArmPan','VSV/ArmTilt', 60.0):
                raise TransformException
            t = self.tf_buffer.lookup_transform('VSV/ArmPan','VSV/ArmTilt', Time())
            self.zoffset = t.transform.translation.z
            self.get_logger().info("Got transform for zoffset")
        except TransformException as ex:
            self.get_logger().error("TF exception: " + repr(ex))

        
        self.jsub = self.create_subscription(JointState,'~/joint_state', self.joint_cb, 1)
        self.get_logger().info("Waiting for first joint state")
        while self.joint_state is None:
            rclpy.spin_once(self,timeout_sec=0.5)

        u = self.joint_state.position[self.index["ArmExtend"]]
        self.get_logger().info("Collecting system length")
        # Prepare the kinematic solver. The assumption is that
        # extend is at zero
        try:
            if not self.waittf('VSV/ArmTilt','VSV/ArmFold', 60.0):
                raise TransformException
            t1 = self.tf_buffer.lookup_transform('VSV/ArmTilt','VSV/ArmFold', Time())
        except TransformException as ex:
            self.get_logger().error("TF exception: " + repr(ex))
        (x1,y1,z1) = (t1.transform.translation.x,t1.transform.translation.y,t1.transform.translation.z)
        print (x1,y1,z1)
        l1 = sqrt(x1*x1+y1*y1+z1*z1)

        try:
            if not self.waittf('VSV/ArmFold','VSV/ToolRotate', 60.0):
                raise TransformException
            t2 = self.tf_buffer.lookup_transform('VSV/ArmFold','VSV/ToolRotate', Time())
        except TransformException as ex:
            self.get_logger().error("TF exception: " + repr(ex))
        (x2,y2,z2) = (t2.transform.translation.x,t2.transform.translation.y,t2.transform.translation.z)

        print (x2,y2,z2)
        l2 = sqrt(x2*x2+y2*y2+z2*z2) - u
        self.get_logger().info("Creating kinematic solver: l1=%.3f l2=%.3f"%(l1,l2))
        self.arm_ik = ArmIK(self,l1,l2)

        self.command = JointState()
        self.command.header = self.joint_state.header
        self.command.name = self.joint_state.name
        self.command.velocity = [0.0] * len(self.joint_state.name)
        self.command.effort = [0.0] * len(self.joint_state.name)
        self.command.position = [x for x in self.joint_state.position]

        try:
            if not self.waittf('VSV/ArmTilt','VSV/ToolRotate', 60.0):
                raise TransformException
            t3 = self.tf_buffer.lookup_transform('VSV/ArmTilt','VSV/ToolRotate', Time())
        except TransformException as ex:
            self.get_logger().error("TF exception: " + repr(ex))
        (a,b,c) = (t3.transform.translation.x,t3.transform.translation.y,t3.transform.translation.z)
        self.x = -b; self.y = a; self.z = c;
        self.state = "Idle"

        self.osub = self.create_subscription(Float32,'~/tool_orientation', self.tool_cb, 1)
        self.tsub = self.create_subscription(Twist,'~/twist', self.twist_cb, 1)
        self.psub = self.create_subscription(Point,'~/position', self.point_cb, 1)
        self.joint = self.create_publisher(JointState,'~/joint_command', 1)

        self.timeout = True
        self.ready = True
        self.state = "Ready"
        self.timer = self.create_timer(self.dt,self.timer_cb)

    def waittf(self,tf_from,tf_to,duration):
        t0 = self.get_clock().now().nanoseconds/1e9
        self.get_logger().info("waiting...")
        last_res = ""
        while rclpy.ok():
            rclpy.spin_once(self,timeout_sec=0.1)
            try:
                res = self.tf_buffer.can_transform(tf_from,tf_to,Time(),Duration(seconds=0.1),True)
                if res[0]:
                    return True
                if not (res[1] == last_res):
                    self.get_logger().warn(res[1])
                    last_res = res[1]
            except TransformExpection as e:
                self.get_logger().warn(f"{e}")
            now = self.get_clock().now().nanoseconds/1e9
            if now - t0 > duration:
                return False


    def tool_cb(self,value):
        self.tool_orientation = value.data

    def joint_cb(self,value):
        self.joint_state = value
        self.index = dict(zip(value.name,range(0,len(value.name))))


    def point_cb(self,value):
        if not self.joint_state or not self.ready:
            # Ignoring joystick while we don't have the joint_state
            return
        if self.state != "Position":
            self.get_logger().info("Processing GOTO %.2f %.2f %.2f request"%(value.x,value.y,value.z))
        value.z -= self.zoffset
        S = self.arm_ik.ik_xyz(value.x,value.y,value.z,0.5)
        if S:
            self.x = value.x; self.y = value.y; self.z = value.z
            (theta0,theta1,theta2,u) = S;
            # print S
            self.state = "Position"
            self.command.position[self.index["ArmPan"]] = theta0
            self.command.position[self.index["ArmTilt"]] = pi/2-theta1
            self.command.position[self.index["ArmFold"]] = -pi/2-theta2
            self.command.position[self.index["ArmExtend"]] = u
        else:
            self.get_logger().info("No Solution")

    def twist_cb(self,value):
        now = self.get_clock().now().nanoseconds/1e9
        if not self.joint_state or not self.ready:
            # Ignoring joystick while we don't have the joint_state
            return
        if self.state != "Velocity":
            self.get_logger().info("Entering velocity mode")
        self.state = "Velocity"
        self.last_twist = now
        self.twist_value = value

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds/1e9
        if self.state == "Velocity":
            if (now - self.last_twist) < 0.5: 
                if self.timeout:
                    self.timeout = False
                    self.get_logger().info("Accepting twist commands")

                dx = sat(self.twist_value.linear.x,0,self.max_velocity) * self.dt
                dy = sat(self.twist_value.linear.y,0,self.max_velocity) * self.dt
                dz = sat(self.twist_value.linear.z,0,self.max_velocity) * self.dt
                new_x = max(self.x + dx,0.8)
                new_y = self.y + dy
                new_z = min(max(self.z + dz,-1.0),0.7)
                # self.get_logger().info("Trying to move to %.2f %.2f %.2f from %.2f %.2f %.2f" \
                #         % (new_x,new_y,new_z, self.x,self.y,self.z))
                S = self.arm_ik.ik_xyz(new_x,new_y,new_z,0.5)
                if S:
                    self.x = new_x; self.y = new_y; self.z = new_z
                    (theta0,theta1,theta2,u) = S;
                    # print S
                    self.command.position[self.index["ArmPan"]] = theta0
                    self.command.position[self.index["ArmTilt"]] = pi/2-theta1
                    self.command.position[self.index["ArmFold"]] = -pi/2-theta2
                    self.command.position[self.index["ArmExtend"]] = u
                else:
                    # self.get_logger().info("No Solution")
                    pass
            else:
                if not self.timeout:
                    self.timeout = True
                    self.get_logger().info("Timeout: ignoring twist commands")
        state = self.joint_state.position[self.index["ToolRotate"]]
        t = self.tf_buffer.lookup_transform('VSV/Tool','VSV/ArmPan', Time())
        F = tf2_kdl.transform_to_kdl(t)
        euler = F.M.GetRPY()
        command = state+angle(euler[0]-self.tool_orientation)
        # print (euler[0], self.tool_orientation, command, state)
        # self.command.position[self.index["ToolRotate"]] += 0.1*error
        self.command.position[self.index["ToolRotate"]] = command
        self.joint.publish(self.command)



def main(args=None):
    rclpy.init(args=args)

    driver = VSVArmIK()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
