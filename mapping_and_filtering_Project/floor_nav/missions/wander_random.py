#!/usr/bin/env python3
# ROS specific imports
import sys
import rclpy
from numpy import pi
import numpy.random
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)


scale=2.0
vel=0.5

while True:
    tc.Constant(linear=0.1,angular=0.0,duration=30.0)
    tc.SetHeading(target=numpy.random.uniform(-pi,pi,1)[0],relative=True,max_angular_velocity=0.3,angle_threshold=0.1,k_theta=3.0)

tc.get_logger().info("Mission completed")
