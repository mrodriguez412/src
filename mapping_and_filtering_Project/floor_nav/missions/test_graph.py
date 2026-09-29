#!/usr/bin/env python3
# ROS specific imports
import sys
import os
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *
import igraph as ig
from ament_index_python.packages import get_package_share_directory

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)
tc.declare_parameter('~/graph_file',
        os.path.join( get_package_share_directory('floor_graph'), 'graphs', 'graph.picklez'))
graph_file = tc.get_parameter('~/graph_file').get_parameter_value().string_value


g = ig.Graph.Read_Picklez(graph_file)

try:
    tc.Wait(duration=1.0)
    # Hand-made hamiltonian path
    node_seq = [2, 1, 4, 3, 5, 6, 0, 9, 8, 10, 7, 11]
    for node in node_seq:
        tc.GoTo(goal_x=g.vs[node]["x"],goal_y=g.vs[node]["y"], max_velocity=0.5, k_v=1.0, max_angular_velocity=1.0)
    tc.Wait(duration=1.0)

except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))


tc.get_logger().info("Mission completed")
