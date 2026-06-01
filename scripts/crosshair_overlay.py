#!/usr/bin/env python3
"""
crosshair_overlay.py

Subscribes to /aurora/rgb/image_raw, draws a bright green crosshair and
ArUco marker annotations, then republishes on /aurora/rgb/crosshair.

Marker poses are published as geometry_msgs/PoseStamped in the URDF
'camera' frame (connected to base_link via robot_state_publisher):
    /aruco/marker_1   (ID 1, 100 mm)
    /aruco/marker_2   (ID 2,  25 mm)

RViz markers are published on /aruco/markers for visual representation.

  source ~/ros2arm_ws/install/setup.bash
  python3 scripts/crosshair_overlay.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from aruco_detect import detect_and_draw, MARKER_SIZES

CROSSHAIR_COLOR = (0, 255, 0)   # bright green (BGR)
CROSSHAIR_ARM   = 25            # pixels each side of centre
CROSSHAIR_GAP   = 4             # pixel gap around centre point
THICKNESS       = 2

# Colour per marker ID for RViz (r, g, b)
MARKER_COLORS = {
    1: (1.0, 0.2, 0.2),   # red
    2: (0.2, 0.6, 1.0),   # blue
}


class CrosshairOverlay(Node):
    def __init__(self):
        super().__init__('crosshair_overlay')
        self.bridge = CvBridge()
        self.K = None
        self.D = None

        self.sub_info = self.create_subscription(
            CameraInfo, '/aurora/rgb/camera_info', self._info_cb, 1)
        self.sub = self.create_subscription(
            Image, '/aurora/rgb/image_raw', self._cb, 10)
        self.pub = self.create_publisher(
            Image, '/aurora/rgb/crosshair', 10)

        # Pose per marker
        self.pose_pubs = {
            mid: self.create_publisher(PoseStamped, f'/aruco/marker_{mid}', 10)
            for mid in MARKER_SIZES
        }

        # RViz visual markers
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco/markers', 10)

    def _info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received')

    def _cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w  = frame.shape[:2]

        # ArUco detection (only if intrinsics are ready)
        if self.K is not None:
            detections = detect_and_draw(frame, self.K, self.D)
            marker_array = MarkerArray()

            for d in detections:
                mid = d['id']
                x, y, z, w_q = d['quat']

                # PoseStamped in URDF 'camera' frame
                pose_msg = PoseStamped()
                pose_msg.header.stamp    = msg.header.stamp
                pose_msg.header.frame_id = 'camera'
                pose_msg.pose.position.x = float(d['tvec'][0])
                pose_msg.pose.position.y = float(d['tvec'][1])
                pose_msg.pose.position.z = float(d['tvec'][2])
                pose_msg.pose.orientation.x = float(x)
                pose_msg.pose.orientation.y = float(y)
                pose_msg.pose.orientation.z = float(z)
                pose_msg.pose.orientation.w = float(w_q)
                self.pose_pubs[mid].publish(pose_msg)

                # RViz cube marker at marker centre
                size  = MARKER_SIZES[mid]
                color = MARKER_COLORS.get(mid, (1.0, 1.0, 0.0))
                vis = Marker()
                vis.header.stamp    = msg.header.stamp
                vis.header.frame_id = 'camera'
                vis.ns              = 'aruco'
                vis.id              = mid
                vis.type            = Marker.CUBE
                vis.action          = Marker.ADD
                vis.pose            = pose_msg.pose
                vis.scale.x         = size
                vis.scale.y         = size
                vis.scale.z         = 0.002          # thin slab
                vis.color.r         = color[0]
                vis.color.g         = color[1]
                vis.color.b         = color[2]
                vis.color.a         = 0.8
                vis.lifetime.sec    = 1              # auto-disappear if not detected
                marker_array.markers.append(vis)

                # Text label above the cube
                label = Marker()
                label.header        = vis.header
                label.ns            = 'aruco_labels'
                label.id            = mid
                label.type          = Marker.TEXT_VIEW_FACING
                label.action        = Marker.ADD
                label.pose          = pose_msg.pose
                label.pose.position.z += size        # float above cube
                label.scale.z       = 0.02           # text height metres
                label.color.r = label.color.g = label.color.b = 1.0
                label.color.a       = 1.0
                label.text          = f'ID {mid} ({int(size*1000)}mm)'
                label.lifetime.sec  = 1
                marker_array.markers.append(label)

            self.marker_pub.publish(marker_array)

        # Crosshair
        cx = int(w * 6.4 / 10)
        cy = int(h * 8 / 10)

        cv2.line(frame, (cx - CROSSHAIR_ARM, cy), (cx - CROSSHAIR_GAP, cy),
                 CROSSHAIR_COLOR, THICKNESS)
        cv2.line(frame, (cx + CROSSHAIR_GAP, cy), (cx + CROSSHAIR_ARM, cy),
                 CROSSHAIR_COLOR, THICKNESS)
        cv2.line(frame, (cx, cy - CROSSHAIR_ARM), (cx, cy - CROSSHAIR_GAP),
                 CROSSHAIR_COLOR, THICKNESS)
        cv2.line(frame, (cx, cy + CROSSHAIR_GAP), (cx, cy + CROSSHAIR_ARM),
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
