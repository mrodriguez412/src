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

try:
    tc.GoTo(goal_x=-scale,goal_y=-scale,max_velocity=vel,k_v=0.0)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=-scale,goal_y=scale,max_velocity=vel,k_v=0.1)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=scale,goal_y=scale,max_velocity=vel,k_v=0.1)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=scale,goal_y=-scale,max_velocity=vel,k_v=0.1)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=-scale,goal_y=-scale,max_velocity=vel,k_v=0.1)

except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))


tc.get_logger().info("Mission completed")
