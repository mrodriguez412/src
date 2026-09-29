#!/usr/bin/env python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)


try:
    for angle in [0.0,pi/2,pi,3*pi/2,0.0]:
        tc.Wait(duration=3.0)
        tc.SetHeading(target=angle)

except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))



tc.get_logger().info("Mission completed")


