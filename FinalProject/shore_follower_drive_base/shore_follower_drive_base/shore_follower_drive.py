#!/usr/bin/env python3

import pytorch_models_base.venv_hack
from pytorch_models_base.model import CNNModel

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import torch
import numpy as np

import cv2
from cv_bridge2 import CvBridge


# Label convention from shore_follower_observe.cpp:
#   0 if angular.z < -rotation_threshold, 2 if angular.z > rotation_threshold, 1 otherwise.
# Labels are defined by the SIGN of the recorded command, so we map them back to that sign
# (in ROS convention +z is a left turn, so 0 = turn right, 2 = turn left).
NEG_TURN = 0
STRAIGHT = 1
POS_TURN = 2


class ShoreFollowerDrive(Node):
    def __init__(self):
        super().__init__("shore_follower_drive")

        self.declare_parameter('~/model', "")
        self.declare_parameter('~/linear_vel', 0.3)
        self.declare_parameter('~/twist_factor', 0.4)
        self.declare_parameter('~/dropout', 0.2)
        self.declare_parameter('~/regression', False)
        self.model_ = self.get_parameter('~/model').get_parameter_value().string_value
        self.regression_ = self.get_parameter('~/regression').get_parameter_value().bool_value
        self.linear_vel_ = self.get_parameter('~/linear_vel').get_parameter_value().double_value
        self.twist_factor_ = self.get_parameter('~/twist_factor').get_parameter_value().double_value
        self.dropout = self.get_parameter('~/dropout').get_parameter_value().double_value

        self.load_model()

        self.br = CvBridge()
        self.twist_pub_ = self.create_publisher(Twist, "~/twist", 1)
        self.image_sub_ = self.create_subscription(Image, "~/image", self.image_callback, 1)

    def load_model(self):
        device = torch.device('cpu')
        self.get_logger().info("Loading pytorch model '" + self.model_ + "'")
        # Must match training: 3 input channels (BGR), 1 output for regression, 3 classes otherwise
        num_classes = 1 if self.regression_ else 3
        self.model = CNNModel(3, num_classes, self.dropout)
        # map_location: the checkpoint was saved from a CUDA model, this lets it load on a CPU-only PC
        self.model.load_state_dict(torch.load(self.model_, weights_only=True, map_location=device))
        self.model.eval()
        self.get_logger().info("Model loaded, ready to run")

    def image_callback(self, data):
        # Same preprocessing as training (reader.py): whole BGR image shrunk to 32x32 with INTER_AREA
        raw = self.br.imgmsg_to_cv2(data, "bgr8")
        processed_ = np.expand_dims(cv2.resize(raw, (32, 32), interpolation=cv2.INTER_AREA), axis=0)
        self.twist_pub_.publish(self.image_to_rot(processed_))

    def image_to_rot(self, img):
        # Reads the image, feed it to the network, get the predictions and act on it.
        out = Twist()
        # always move forward
        out.linear.x = self.linear_vel_
        # Runs the network: (1, H, W, C) -> (1, C, H, W), float in [0,1] like in training
        img = np.rollaxis(img, 3, 1)
        img = torch.from_numpy(img.astype(np.float32) / 255.)
        with torch.no_grad():
            res = self.model(img)
        if self.regression_:
            # Network directly predicts the rotation speed
            out.angular.z = float(res[0, 0])
        else:
            # Softmax gives the probability of each label. Instead of a hard argmax
            # (which makes the robot zig-zag), use the expected rotation:
            #   z = twist_factor * (P(positive turn) - P(negative turn))
            # It is +/- twist_factor when the network is sure, and smaller when it hesitates.
            p = torch.softmax(res, dim=1)[0].numpy()
            out.angular.z = float(self.twist_factor_ * (p[POS_TURN] - p[NEG_TURN]))
            self.get_logger().debug("P(neg, straight, pos) = %.2f %.2f %.2f -> z = %.2f"
                                    % (p[NEG_TURN], p[STRAIGHT], p[POS_TURN], out.angular.z))
        return out


def main(args=None):
    rclpy.init(args=args)

    drive = ShoreFollowerDrive()

    rclpy.spin(drive)

    drive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
