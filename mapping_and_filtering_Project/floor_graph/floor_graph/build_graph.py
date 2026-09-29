#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros

import igraph as ig
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import Point

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class GraphBuilder(Node):
    def __init__(self):
        super().__init__('graph_publisher')
        self.declare_parameter('~/graph_file', "graph.pickles")
        self.declare_parameter('~/graph_plot', "graph.png")
        self.graph_file = self.get_parameter('~/graph_file').get_parameter_value().string_value
        self.graph_plot = self.get_parameter('~/graph_plot').get_parameter_value().string_value
        self.saved = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        if self.saved:
            return
        g = ig.Graph();
        g.add_vertices(12)
        g.add_edges([(0,i) for i in [2,6,9]])
        g.add_edges([(1,i) for i in [2,3,4]])
        g.add_edges([(2,i) for i in [3,6]])
        g.add_edges([(3,i) for i in [4,5]])
        g.add_edges([(5,i) for i in [6]])
        # g.add_edges([(6,i) for i in []])
        g.add_edges([(7,i) for i in [10,11]])
        g.add_edges([(8,i) for i in [9, 10]])
        g.add_edges([(9,i) for i in [10]])
        g.add_edges([(10,i) for i in [11]])
        # 11, 10 and 4 do not connect to a higher-index vertex

        vx=[0.0] * len(g.vs)
        vy=[0.0] * len(g.vs)
        label=[""] * len(g.vs)
        for i,v in enumerate(g.vs):
            try:
                t = self.tf_buffer.lookup_transform( "world", "Node%d"%v.index, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform Node{v.index} to world: {ex}')
                return
            vx[i] = t.transform.translation.x
            vy[i] = t.transform.translation.y
            label[i] = "Node%d" % v.index
        g.vs["x"] = vx
        g.vs["y"] = vy
        g.vs["label"] = label

        layout = g.layout("auto")
        ig.plot(g,self.graph_plot, layout = layout, label_dist=[100.]*len(g.vs))
        g.write_picklez(self.graph_file)
        self.get_logger().info("Saved graph")
        self.saved = True



def main():
    rclpy.init()
    node = GraphBuilder()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
