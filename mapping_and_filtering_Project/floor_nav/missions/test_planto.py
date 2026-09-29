#!/usr/bin/env python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)


scale=2.0

try:
    tc.PlanTo(goal_x=-scale,goal_y=-scale,goal_theta=0.0)
    tc.Wait(duration=1.0)
    tc.PlanTo(goal_x=-scale,goal_y=scale,goal_theta=0.0)
    tc.Wait(duration=1.0)
    tc.PlanTo(goal_x=scale,goal_y=scale,goal_theta=0.0)
    tc.Wait(duration=1.0)
    tc.PlanTo(goal_x=scale,goal_y=-scale,goal_theta=0.0)
    tc.Wait(duration=1.0)
    tc.PlanTo(goal_x=-scale,goal_y=-scale,goal_theta=0.0)

except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))


tc.get_logger().info("Mission completed")
