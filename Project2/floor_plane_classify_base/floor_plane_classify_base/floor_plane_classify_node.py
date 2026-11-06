#!/usr/bin/env python3

import pytorch_models_base.venv_hack
from pytorch_models_base.model import CNNModel

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import torch
import numpy as np

import cv2
# cv_bridge2 is a local version of cv_bridge compiled with numpy 2
from cv_bridge2 import CvBridge

# Label convention from floor_plane_record (labels.txt): 1 = traversable, 0 = untraversable
UNTRAVERSABLE = 0
TRAVERSABLE = 1


class FloorPlaneClassify(Node):
    def __init__(self):
        super().__init__("classify")

        self.declare_parameter('~/model', ".")
        self.declare_parameter('~/thumb_size', 32)
        self.declare_parameter('~/num_classes', 2)
        self.declare_parameter('~/dropout', 0.2)
        self.model_ = self.get_parameter('~/model').get_parameter_value().string_value
        self.ts_ = self.get_parameter('~/thumb_size').get_parameter_value().integer_value
        self.num_classes = self.get_parameter('~/num_classes').get_parameter_value().integer_value
        self.dropout = self.get_parameter('~/dropout').get_parameter_value().double_value

        self.load_model()

        self.br = CvBridge()
        self.image_pub_ = self.create_publisher(Image, "~/image_label", 1)
        self.image_sub_ = self.create_subscription(Image, "~/image", self.image_callback, 1)

    def load_model(self):
        self.get_logger().info("Loading pytorch model %s" % self.model_)
        # Must match the training setup: 3 input channels (BGR), num_classes outputs
        num_channels = 3
        self.model = CNNModel(num_channels, self.num_classes, self.dropout)
        # map_location: the checkpoint was saved from a CUDA model, this lets it load on a CPU-only PC
        self.model.load_state_dict(torch.load(self.model_, weights_only=True, map_location='cpu'))
        self.model.eval()
        self.get_logger().info("Model loaded, ready to run")

    def reshape_split(self, image, kernel_size):
        img_height, img_width, channels = image.shape
        tile_heigth, tile_width = kernel_size
        tiled_array = image.reshape(img_height // tile_heigth,
                                    tile_heigth,
                                    img_width // tile_width,
                                    tile_width,
                                    channels)
        tiled_array = tiled_array.swapaxes(1, 2)
        return tiled_array.reshape(-1, self.ts_, self.ts_, 3)

    def image_callback(self, data):
        # Gets the image, check that it is square and divisible by the thumbnail size
        assert(data.height == data.width)
        assert(data.height % self.ts_ == 0)
        # Convert to np array
        img = self.br.imgmsg_to_cv2(data, "bgr8")
        # Reshape the image as a batch of thumbnails (faster processing when using batches)
        batch = self.reshape_split(img, (self.ts_, self.ts_))
        # Calls the network
        checked = self.check_thumb(batch)
        # Transforms the array into low resolution image (one pixel per thumbnail)
        low_res = np.reshape(checked, [data.height // self.ts_, data.width // self.ts_, 3])
        # Upsamples the predictions so they have the same size as the input image
        classified = cv2.resize(low_res, (0, 0), fx=self.ts_, fy=self.ts_,
                                interpolation=cv2.INTER_NEAREST).astype(np.uint8)
        overlay = cv2.addWeighted(img, 0.5, classified, 0.5, 0)
        # Publish the result. cv_bridge2 crashes when asked for "bgr8" directly, so convert with
        # passthrough and then label it bgr8 so RViz / image_view display the colors correctly
        enc = self.br.cv2_to_imgmsg(overlay, "passthrough")
        enc.encoding = "bgr8"
        enc.header = data.header
        self.image_pub_.publish(enc)

    def check_thumb(self, batch):
        # Same preprocessing as training (reader.py): BGR, float in [0,1], no mean/std normalization
        batch = np.rollaxis(batch, 3, 1)  # (N, H, W, C) -> (N, C, H, W)
        batch = torch.from_numpy(batch.astype(np.float32) / 255.)
        with torch.no_grad():
            # Network outputs logits, softmax turns them into class probabilities
            prob = torch.softmax(self.model(batch), dim=1).numpy()
        p_trav = prob[:, TRAVERSABLE]
        # One BGR color per thumbnail: green = traversable, red = untraversable,
        # blended by the predicted probability (i.e. the danger level)
        array = np.zeros((prob.shape[0], 3), dtype=np.float32)
        array[:, 1] = 255. * p_trav          # G
        array[:, 2] = 255. * (1. - p_trav)   # R
        return array


def main(args=None):
    rclpy.init(args=args)

    classify = FloorPlaneClassify()

    rclpy.spin(classify)

    classify.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
