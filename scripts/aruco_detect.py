#!/usr/bin/env python3
"""
aruco_detect.py

Provides detect_and_draw(frame, K, D) for use by crosshair_overlay.py.
Can also be run standalone as a ROS2 node.

Markers:
    ID 1 → 100 mm side
    ID 2 →  25 mm side
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# Physical side lengths per marker ID (metres)
MARKER_SIZES = {
    1: 0.100,
    2: 0.025,
}

# Detector — OpenCV 4.6 API
_aruco_dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
_aruco_params = cv2.aruco.DetectorParameters_create()


def _obj_points(size):
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                    dtype=np.float64)


def _rvec_to_quat(rvec):
    """Rodrigues vector → quaternion (x, y, z, w)."""
    R, _ = cv2.Rodrigues(rvec)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def _draw_axes(frame, K, D, rvec, tvec, length):
    axes_3d = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]])
    pts, _ = cv2.projectPoints(axes_3d, rvec, tvec, K, D)
    pts = pts.reshape(-1, 2).astype(int)
    origin = tuple(pts[0])
    cv2.arrowedLine(frame, origin, tuple(pts[1]), (0, 0, 255), 2, tipLength=0.3)  # X red
    cv2.arrowedLine(frame, origin, tuple(pts[2]), (0, 255, 0), 2, tipLength=0.3)  # Y green
    cv2.arrowedLine(frame, origin, tuple(pts[3]), (255, 0, 0), 2, tipLength=0.3)  # Z blue


def detect_and_draw(frame, K, D):
    """
    Detect ArUco markers in frame, draw annotations, and return pose data.

    Args:
        frame: BGR numpy array (modified in place)
        K:     3x3 camera matrix (np.float64)
        D:     distortion coefficients (np.float64)

    Returns:
        detections: list of dicts, one per detected known marker:
            {
                'id':   int,
                'tvec': np.array([x, y, z]),   # metres, camera frame
                'quat': (x, y, z, w),          # orientation quaternion
            }
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, _aruco_dict, parameters=_aruco_params)

    detections = []

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            size = MARKER_SIZES.get(marker_id)
            if size is None:
                continue

            img_pts = corners[i][0].astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                _obj_points(size), img_pts, K, D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue

            tvec = tvec.flatten()
            rvec = rvec.flatten()
            quat = _rvec_to_quat(rvec)

            _draw_axes(frame, K, D, rvec, tvec, size * 0.5)

            detections.append({
                'id':   int(marker_id),
                'tvec': tvec,
                'quat': quat,
            })

    return detections


# ── Standalone node ───────────────────────────────────────────────────────────

class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__("aruco_detect")
        self.bridge = CvBridge()
        self.K = None
        self.D = None

        self.create_subscription(CameraInfo, "/aurora/rgb/camera_info", self._info_cb, 1)
        self.create_subscription(Image,      "/aurora/rgb/image_raw",   self._image_cb, 1)
        self.get_logger().info("Waiting for camera topics...")

    def _info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f"Intrinsics ready — "
                f"fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} "
                f"cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}"
            )

    def _image_cb(self, msg: Image):
        if self.K is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detections = detect_and_draw(frame, self.K, self.D)

        for d in detections:
            t = d['tvec']
            x, y, z, w = d['quat']
            self.get_logger().info(
                f"ID {d['id']} ({MARKER_SIZES[d['id']]*1000:.0f}mm) | "
                f"xyz=[{t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}] m | "
                f"quat=[{x:.3f}, {y:.3f}, {z:.3f}, {w:.3f}]"
            )

        if not detections:
            cv2.putText(frame, "No markers", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.imshow("ArUco", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ArucoDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
