#!/usr/bin/env python3
"""
crosshair_overlay.py

Subscribes to /aurora/rgb/image_raw, draws a bright green crosshair at
(3/4 * width, 3/4 * height), and republishes on /aurora/rgb/crosshair.

Run alongside the camera driver — this node sits between the driver and
any consumer (RViz, data recorder, ACT policy) that needs the crosshair.

  source ~/ros2arm_ws/install/setup.bash
  python3 scripts/crosshair_overlay.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

CROSSHAIR_COLOR = (0, 255, 0)   # bright green (BGR)
CROSSHAIR_ARM   = 25            # pixels each side of centre
CROSSHAIR_GAP   = 4             # pixel gap around centre point
THICKNESS       = 2


class CrosshairOverlay(Node):
    def __init__(self):
        super().__init__('crosshair_overlay')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/aurora/rgb/image_raw', self._cb, 10)
        self.pub = self.create_publisher(
            Image, '/aurora/rgb/crosshair', 10)

    def _cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w  = frame.shape[:2]

        cx = int(w * 6.4 / 10)
        cy = int(h * 8 / 10)

        # Horizontal arm
        cv2.line(frame,
                 (cx - CROSSHAIR_ARM, cy), (cx - CROSSHAIR_GAP, cy),
                 CROSSHAIR_COLOR, THICKNESS)
        cv2.line(frame,
                 (cx + CROSSHAIR_GAP, cy), (cx + CROSSHAIR_ARM, cy),
                 CROSSHAIR_COLOR, THICKNESS)

        # Vertical arm
        cv2.line(frame,
                 (cx, cy - CROSSHAIR_ARM), (cx, cy - CROSSHAIR_GAP),
                 CROSSHAIR_COLOR, THICKNESS)
        cv2.line(frame,
                 (cx, cy + CROSSHAIR_GAP), (cx, cy + CROSSHAIR_ARM),
                 CROSSHAIR_COLOR, THICKNESS)

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CrosshairOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
