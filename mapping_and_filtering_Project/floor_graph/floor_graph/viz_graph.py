#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import igraph as ig
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import Point

class GraphPublisher(Node):
    def __init__(self):
        super().__init__('graph_publisher')
        self.declare_parameter('~/timer_period', 0.5)
        self.declare_parameter('~/graph_file', "graph.pickles")

        self.timer_period = self.get_parameter('~/timer_period').get_parameter_value().double_value
        self.graph_file = self.get_parameter('~/graph_file').get_parameter_value().string_value

        self.publisher = self.create_publisher(MarkerArray, '~/viz', 1)
        self.ma = MarkerArray()
        g = ig.Graph.Read_Picklez(self.graph_file)
        id = 0
        now = self.get_clock().now()
        for v in g.vs:
            marker = Marker()
            marker.header.stamp = now.to_msg()
            marker.header.frame_id = "/world"
            marker.ns = "graph_nodes"
            marker.id = id; id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = v["x"]
            marker.pose.position.y = v["y"]
            marker.pose.position.z = -0.05
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            self.ma.markers.append(marker)
        for e in g.es:
            v0 = g.vs[e.source]
            v1 = g.vs[e.target]
            print (e.source,e.target)
            marker = Marker()
            marker.header.stamp = now.to_msg()
            marker.header.frame_id = "/world"
            marker.ns = "graph_edges"
            marker.id = id; id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = -0.05
            marker.scale.x = 0.05
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.points = [self.point(v0["x"],v0["y"],0.0),self.point(v1["x"],v1["y"],0.0)];
            self.ma.markers.append(marker)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def point(self,x,y,z):
        P=Point()
        P.x=x
        P.y=y
        P.z=z
        return P

    def timer_callback(self):
        now = self.get_clock().now()
        for m in self.ma.markers:
            m.header.stamp = now.to_msg()
        self.publisher.publish(self.ma)



def main():
    rclpy.init()
    node = GraphPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
