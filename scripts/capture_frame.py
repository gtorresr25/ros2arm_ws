#!/usr/bin/env python3
"""
capture_frame.py — Grab one synchronised RGB + depth frame from the Aurora 930.

Saves:
  <stem>.jpg        — BGR colour image
  <stem>_depth.png  — 16-bit depth image (millimetres, raw)

Usage:
  python3 scripts/capture_frame.py [output.jpg]

Requires the camera launched with depth_enable:=True (teleop_rviz.launch.py).
"""

import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class FrameCapture(Node):
    def __init__(self, rgb_path: str, depth_path: str):
        super().__init__('capture_frame')
        self._bridge = CvBridge()
        self._rgb_path   = rgb_path
        self._depth_path = depth_path
        self._rgb   = None
        self._depth = None

        self.create_subscription(Image, '/aurora/rgb/image_raw',   self._rgb_cb,   1)
        self.create_subscription(Image, '/aurora/depth/image_raw', self._depth_cb, 1)
        self.get_logger().info('Waiting for RGB + depth frames …')

    def _rgb_cb(self, msg: Image):
        if self._rgb is None:
            self._rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg: Image):
        if self._depth is None:
            # 16UC1 — depth in millimetres
            self._depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    @property
    def done(self) -> bool:
        return self._rgb is not None and self._depth is not None

    def save(self):
        cv2.imwrite(self._rgb_path, self._rgb)
        cv2.imwrite(self._depth_path, self._depth)
        self.get_logger().info(f'RGB   → {self._rgb_path}')
        self.get_logger().info(f'Depth → {self._depth_path}')


def main():
    rgb_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/cube_capture.jpg'
    stem, ext = os.path.splitext(rgb_path)
    depth_path = stem + '_depth.png'

    rclpy.init()
    node = FrameCapture(rgb_path, depth_path)
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
