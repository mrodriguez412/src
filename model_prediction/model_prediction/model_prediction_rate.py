#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

class ModelPrediction(Node):
    def __init__(self):
        super().__init__("model_prediction")
        self.declare_parameter("~/rate",10.0)
        self.declare_parameter("~/inverse_coef_list",True)
        self.declare_parameter("~/command_field","")
        self.declare_parameter("~/command_type","")
        self.declare_parameter("~/command_coef_csv","")
        self.declare_parameter("~/state_field","")
        self.declare_parameter("~/state_type","")
        self.declare_parameter("~/state_coef_csv","")

        self.rate_param = self.get_parameter("~/rate").get_parameter_value().double_value
        self.inverse_param = self.get_parameter("~/inverse_coef_list").get_parameter_value().bool_value
        self.command_field = self.get_parameter("~/command_field").get_parameter_value().string_value
        self.command_type = self.get_parameter("~/command_type").get_parameter_value().string_value
        self.command_coef_csv = self.get_parameter("~/command_coef_csv").get_parameter_value().string_value
        self.state_field = self.get_parameter("~/state_field").get_parameter_value().string_value
        self.state_type = self.get_parameter("~/state_type").get_parameter_value().string_value
        self.state_coef_csv = self.get_parameter("~/state_coef_csv").get_parameter_value().string_value

        self.command_type = self.command_type.split("/")
        if len(self.command_type)!=2:
            self.get_logger().error("Invalid command type. Use the pkg/msg syntax")
            raise ImportError
        self.state_type = self.state_type.split("/")
        if len(self.state_type)!=2:
            self.get_logger().error("Invalid state type. Use the pkg/msg syntax")
            raise ImportError
        exec ("from %s.msg import %s" % (self.command_type[0],self.command_type[1]))
        exec ("from %s.msg import %s" % (self.state_type[0],self.state_type[1]))
        
        self.command_coef = [float(x) for x in self.command_coef_csv.split(",") if len(x)>0]
        self.state_coef = [float(x) for x in self.state_coef_csv.split(",") if len(x)>0]

        if len(self.command_coef) == 0:
            self.command_coef=[0.0]
        if len(self.state_coef) == 0:
            self.state_coef=[0.0]
        if self.inverse_param:
            self.command_coef.reverse()
            self.state_coef.reverse()
            print(self.command_coef)
            print(self.state_coef)
        self.command = None
        self.state = None

        exec ("self.state_sub = self.create_subscription(%s, '~/state', self.state_cb, 1)" % self.state_type[1])
        exec ("self.command_sub = self.create_subscription(%s, '~/command', self.command_cb, 1)" % self.command_type[1])
        self.fpub = self.create_publisher(Float64, "~/prediction",1)
        self.tpub = self.create_publisher(TwistStamped, "~/twist_prediction",1)

        
        self.x = []
        self.u = []
        self.timer = self.create_timer(1./self.rate_param,self.timer_cb)


    def command_cb(self,msg):
        # print("Commmand: "+str(msg)+" -> "+self.command_field)
        if len(self.command_field):
            value = eval("float(msg.%s)" % self.command_field)
        else:
            value = float(msg)
        # self.get_logger().info("Command value: "+str(value))
        self.command = value

    def state_cb(self,msg):
        # print("State: "+str(msg)+" -> "+self.state_field)
        if len(self.state_field):
            value = eval("float(msg.%s)" % self.state_field)
        else:
            value = float(msg)
        # self.get_logger().info("State value: "+str(value))
        self.state = value

    def timer_cb(self):
        # self.get_logger().info("X: "+str(self.x))
        # self.get_logger().info("U: "+str(self.u))
        if not self.state is None:
            self.x.append(self.state)
            if len(self.x) > len(self.state_coef):
                self.x = self.x[-len(self.state_coef):]
        if not self.command is None:
            self.u.append(self.command)
            if len(self.u) > len(self.command_coef):
                self.u = self.u[-len(self.command_coef):]
        if len(self.x)==len(self.state_coef) and len(self.u)==len(self.command_coef):
            # print("State")
            # print([(a,b) for a,b in zip(self.state_coef,self.x)])
            # print("Command")
            # print([(a,b) for a,b in zip(self.command_coef,self.u)])
        
            pred = sum([-a*xi for a,xi in zip(self.state_coef,self.x)]) \
                    + sum([b*ui for b,ui in zip(self.command_coef,self.u)])
            f = Float64()
            f.data = pred
            self.fpub.publish(f)

            t = TwistStamped()
            t.header.frame_id="n/a"
            t.header.stamp = self.get_clock().now().to_msg()
            t.twist.linear.x = pred
            self.tpub.publish(t)


def main(args=None):
    rclpy.init(args=args)

    driver = ModelPrediction()

    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

