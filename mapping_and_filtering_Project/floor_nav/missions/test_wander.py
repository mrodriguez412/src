#!/usr/bin/env python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)


scale=2.0
vel=0.5

while True:
    try:
        tc.Wander(max_linear_speed=0.5, safety_range=0.8, dont_care_range=2.5, max_angular_speed=0.5,multiplier=0.3,front_sector=False,angular_range=pi/6, task_timeout=60.0)
    except TaskException as e:
        pass
    for i in range(3):
        tc.SetHeading(target=2*pi/3,relative=True,max_angular_velocity=3.0,angle_threshold=0.1,k_theta=3.0)


tc.get_logger().info("Mission completed")
