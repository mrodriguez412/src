#!/usr/bin/env python3
"""Evaluation only. Never feed this into the detector.

The scene publishes the true treasure poses as TF frames world -> Treasure<i>.
This node:
  * reports the error of every detection against the closest true treasure;
  * optionally logs (true distance tool->treasure, detector reading) to a CSV
    file, used by fit_sensor_model to calibrate the inverse sensor model.
"""
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TreasureEval(Node):
    def __init__(self):
        super().__init__('treasure_eval')
        self.world_frame = self.declare_parameter('world_frame', 'world').value
        self.tool_frame = self.declare_parameter('tool_frame', 'VSV/Tool').value
        self.n_treasures = self.declare_parameter('n_treasures', 5).value
        csv_file = self.declare_parameter('csv_file', '').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.truth = {}
        self.detections = []
        self.reported = {}

        self.writer = None
        if csv_file:
            self.csv = open(csv_file, 'w', newline='')
            self.writer = csv.writer(self.csv)
            self.writer.writerow(['t', 'tool_x', 'tool_y', 'tool_z', 'true_distance', 'signal'])
            self.get_logger().info('Logging calibration data to ' + csv_file)
            self.create_subscription(Float32, '/vrep/metalDetector', self.signal_cb, 10)

        self.create_subscription(PoseArray, '/treasure_detector/detections', self.det_cb, 1)
        self.create_timer(2.0, self.report_cb)

    def update_truth(self):
        for i in range(1, self.n_treasures + 1):
            if i in self.truth:
                continue  # treasures do not move
            try:
                t = self.tf_buffer.lookup_transform(self.world_frame, 'Treasure%d' % i, Time())
                tr = t.transform.translation
                self.truth[i] = (tr.x, tr.y, tr.z)
                self.get_logger().info('Ground truth Treasure%d at (%.2f, %.2f, %.2f)'
                                       % (i, tr.x, tr.y, tr.z))
            except TransformException:
                pass

    def signal_cb(self, msg):
        self.update_truth()
        if not self.truth:
            return
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, self.tool_frame, Time())
        except TransformException:
            return
        p = t.transform.translation
        d = min(math.dist((p.x, p.y, p.z), q) for q in self.truth.values())
        stamp = self.get_clock().now().nanoseconds / 1e9
        self.writer.writerow(['%.3f' % stamp, '%.3f' % p.x, '%.3f' % p.y, '%.3f' % p.z,
                              '%.4f' % d, '%.4f' % msg.data])

    def det_cb(self, msg):
        self.detections = [(p.position.x, p.position.y) for p in msg.poses]

    def report_cb(self):
        self.update_truth()
        if not self.truth:
            return
        for k, (x, y) in enumerate(self.detections, start=1):
            i, q = min(self.truth.items(), key=lambda kv: math.hypot(kv[1][0] - x, kv[1][1] - y))
            err = math.hypot(q[0] - x, q[1] - y)
            # only print when the estimate changed noticeably
            if k in self.reported and abs(self.reported[k] - err) < 0.01:
                continue
            self.reported[k] = err
            self.get_logger().info('Detection T%d -> Treasure%d: est (%.2f, %.2f) true '
                                   '(%.2f, %.2f) error %.3f m' % (k, i, x, y, q[0], q[1], err))

    def destroy_node(self):
        if self.writer is not None:
            self.csv.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TreasureEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
