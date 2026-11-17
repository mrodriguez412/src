#!/usr/bin/env python3
"""Fuse metal detector readings into treasure position estimates.

Subscribes:  /vrep/metalDetector (std_msgs/Float32), TF world -> VSV/Tool
Publishes:   ~/markers    (visualization_msgs/MarkerArray) detected treasures
             ~/detections (geometry_msgs/PoseArray)        detected treasures
             ~/heatmap    (nav_msgs/OccupancyGrid)          votes over the explored area
"""
import array
import math

import numpy as np
import rclpy
from scipy.ndimage import maximum_filter
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from treasure_detector.treasure_grid import MetalDetectorModel, TreasureGrid


class Detection:
    def __init__(self, ident, est, max_signal):
        self.id = ident
        self.update(est, max_signal)

    def update(self, est, max_signal):
        self.x, self.y = est['x'], est['y']
        self.cov = est['cov']
        self.llr = est['llr']
        self.ambiguous = est['ambiguous']
        self.rival = est['rival']
        self.max_signal = max_signal

    def sigma(self):
        return math.sqrt(max(np.linalg.eigvalsh(self.cov)))


class TreasureDetector(Node):
    def __init__(self):
        super().__init__('treasure_detector')
        p = self.declare_parameter
        self.world_frame = p('world_frame', 'world').value
        self.tool_frame = p('tool_frame', 'VSV/Tool').value
        # sensor model (defaults = values in the treasure_search.ttt script)
        model = MetalDetectorModel(
            sigma=p('model.sigma', 2.0).value,
            amplitude=p('model.amplitude', 1.0).value,
            noise_width=p('model.noise_width', 0.1).value,
            d_cap=p('model.d_cap', 5.0).value,
            blur=p('model.blur', 0.02).value,
            outlier=p('model.outlier', 0.01).value)
        self.grid = TreasureGrid(
            model,
            x_min=p('grid.x_min', -60.0).value, x_max=p('grid.x_max', 60.0).value,
            y_min=p('grid.y_min', -60.0).value, y_max=p('grid.y_max', 60.0).value,
            resolution=p('grid.resolution', 0.05).value,
            treasure_z=p('treasure_z', 0.0).value)
        # detection logic
        self.llr_threshold = p('llr_threshold', 30.0).value
        self.peak_signal = p('peak_signal_threshold', 0.6).value
        self.peak_signal_radius = p('peak_signal_radius', 3.0).value
        self.min_separation = p('min_separation', 6.0).value
        self.search_radius = p('search_radius', 5.0).value
        self.mode_radius = p('mode_radius', 0.75).value
        self.ambiguity_gap = p('ambiguity_gap', 3.0).value
        self.heat_k = max(1, int(round(p('heatmap_resolution', 0.25).value / self.grid.res)))
        heat_cell = self.grid.res * self.heat_k
        self.heat_window = 2 * int(p('heatmap_window', 10.0).value / heat_cell / 2) + 1
        self.marker_z = self.grid.treasure_z

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detections = []
        self.last_tool = None
        self.last_signal = 0.0
        self.n_tf_fail = 0

        self.create_subscription(Float32, '/vrep/metalDetector', self.signal_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 1)
        self.pose_pub = self.create_publisher(PoseArray, '~/detections', 1)
        # transient local so RViz's Map display (transient local by default) connects
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.heat_pub = self.create_publisher(OccupancyGrid, '~/heatmap', latched)
        self.create_timer(p('detect_period', 0.5).value, self.detect_cb)
        self.create_timer(p('heatmap_period', 1.0).value, self.heatmap_cb)
        self.get_logger().info(
            'Treasure detector ready: grid %dx%d @ %.2fm, update radius %.1fm'
            % (self.grid.nx, self.grid.ny, self.grid.res, self.grid.update_radius))

    # ------------------------------------------------------------ measurement
    def signal_cb(self, msg):
        # Float32 has no stamp. The sim publishes TF and the reading in the
        # same sensing step, so the latest transform is the matching one.
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, self.tool_frame, Time())
        except TransformException as e:
            self.n_tf_fail += 1
            if self.n_tf_fail % 100 == 1:
                self.get_logger().warn('No tool transform: %s' % e)
            return
        tr = t.transform.translation
        self.last_tool = (tr.x, tr.y, tr.z)
        self.last_signal = msg.data
        self.grid.add(tr.x, tr.y, tr.z, float(msg.data))

    # -------------------------------------------------------------- detection
    def detect_cb(self):
        if self.grid.n_updates == 0:
            return
        # 1. refine the existing detections with everything received so far
        for det in self.detections:
            est = self._estimate(det.x, det.y)
            det.update(est, self.grid.max_signal_near(est['x'], est['y'],
                                                      self.peak_signal_radius))
        # 2. look for new peaks away from the known treasures
        while True:
            exclude = [(d.x, d.y, self.min_separation) for d in self.detections]
            peak = self.grid.global_peak(exclude)
            if peak is None or peak[0] < self.llr_threshold:
                break
            _, px, py = peak
            max_sig = self.grid.max_signal_near(px, py, self.peak_signal_radius)
            if max_sig < self.peak_signal:
                break  # evidence exists, but no high detector peak yet
            det = Detection(len(self.detections) + 1, self._estimate(px, py), max_sig)
            self.detections.append(det)
            self.get_logger().info(
                'Treasure %d detected at (%.2f, %.2f) sigma=%.2fm LLR=%.0f peak=%.2f%s'
                % (det.id, det.x, det.y, det.sigma(), det.llr, max_sig,
                   ' [AMBIGUOUS: mirror at (%.2f, %.2f)]' % det.rival[:2]
                   if det.ambiguous else ''))
        self.publish_detections()

    def _estimate(self, x, y):
        return self.grid.estimate(x, y, self.search_radius, self.mode_radius,
                                  self.ambiguity_gap)

    # ----------------------------------------------------------------- output
    def publish_detections(self):
        now = self.get_clock().now().to_msg()
        pa = PoseArray()
        pa.header.frame_id = self.world_frame
        pa.header.stamp = now
        ma = MarkerArray()
        for det in self.detections:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = det.x, det.y, self.marker_z
            pose.orientation.w = 1.0
            pa.poses.append(pose)

            color = (1.0, 0.6, 0.0) if det.ambiguous else (0.0, 0.9, 0.1)
            # uncertainty ellipse: 2 sigma, never smaller than 30 cm so it stays visible
            evals, evecs = np.linalg.eigh(det.cov)
            yaw = math.atan2(evecs[1, 1], evecs[0, 1])
            m = self._marker(det.id * 3, Marker.CYLINDER, det.x, det.y,
                             self.marker_z + 0.05, color, 0.6)
            m.scale.x = max(4 * math.sqrt(evals[1]), 0.3)
            m.scale.y = max(4 * math.sqrt(evals[0]), 0.3)
            m.scale.z = 0.1
            m.pose.orientation.z = math.sin(yaw / 2)
            m.pose.orientation.w = math.cos(yaw / 2)
            ma.markers.append(m)

            m = self._marker(det.id * 3 + 1, Marker.SPHERE, det.x, det.y,
                             self.marker_z + 0.5, color, 1.0)
            m.scale.x = m.scale.y = m.scale.z = 0.4
            ma.markers.append(m)

            m = self._marker(det.id * 3 + 2, Marker.TEXT_VIEW_FACING, det.x, det.y,
                             self.marker_z + 1.3, (1.0, 1.0, 1.0), 1.0)
            m.scale.z = 0.5
            m.text = 'T%d (%.2f, %.2f) +/-%.2fm%s' % (
                det.id, det.x, det.y, det.sigma(), ' ?' if det.ambiguous else '')
            ma.markers.append(m)
        self.pose_pub.publish(pa)
        self.marker_pub.publish(ma)

    def _marker(self, ident, mtype, x, y, z, rgb, alpha):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'treasures'
        m.id = ident
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = alpha
        return m

    def heatmap_cb(self):
        """Votes over everything explored: 100 = strongest cell within
        heatmap_window, 0 = no evidence or evidence against, -1 = never sensed.

        The 5 cm grid is max-pooled into heatmap_resolution cells so that the
        whole lake stays light enough for RViz and peaks do not get averaged out.
        """
        if self.grid.bbox is None or self.heat_pub.get_subscription_count() == 0:
            return
        i0, i1, j0, j1 = self.grid.bbox
        k = self.heat_k
        sub = self.grid.llr[i0:i1, j0:j1]
        h, w = sub.shape
        sub = np.pad(sub, ((0, (-h) % k), (0, (-w) % k)))
        blocks = sub.reshape(sub.shape[0] // k, k, sub.shape[1] // k, k)
        peak = blocks.max(axis=(1, 3))
        seen = (blocks != 0.0).any(axis=(1, 3))
        # The LLR keeps growing with every reading, so a fixed scale saturates
        # every ring within a second. Scale each cell relative to the strongest
        # cell within heatmap_window instead: the best intersection is 100 and
        # a ring holding a quarter of that evidence is 25. The detection
        # threshold sets a floor so background noise is not magnified.
        local_max = maximum_filter(np.maximum(peak, 0.0), size=self.heat_window)
        scale = np.maximum(local_max, self.llr_threshold)
        data = np.clip(100.0 * peak / scale, 0, 100).astype(np.int8)
        data[~seen] = -1
        g = OccupancyGrid()
        g.header.frame_id = self.world_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = float(self.grid.res * k)
        g.info.height, g.info.width = data.shape
        g.info.origin.position.x = float(self.grid.x_min + j0 * self.grid.res)
        g.info.origin.position.y = float(self.grid.y_min + i0 * self.grid.res)
        g.info.origin.position.z = float(self.grid.treasure_z)
        g.info.origin.orientation.w = 1.0
        g.data = array.array('b', data.tobytes())
        self.heat_pub.publish(g)


def main(args=None):
    rclpy.init(args=args)
    node = TreasureDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
