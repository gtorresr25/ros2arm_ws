#!/usr/bin/env python3
"""
parallel_edges.py — Detect parallel edges of a cube from depth + RGB streams.

Strategy:
  1. Threshold depth to isolate the closest object (the cube).
  2. Extract its silhouette contour from the depth mask — immune to lighting/glare.
  3. Fit a polygon to that contour and find the two longest parallel sides.
  4. Back-project the grasp midpoint to 3D using camera intrinsics + depth.

Press Q to quit.

Run with:
    python3 scripts/parallel_edges.py

Requires teleop_rviz.launch.py (depth_enable=True) in another terminal.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from math import atan2, degrees, pi, cos, sin

# ── Tunable parameters ────────────────────────────────────────────────────────
CUBE_MARGIN_MM   = 30    # depth band above scene minimum (1 inch = 25.4mm + slack)
MIN_VALID_MM     = 100   # ignore depth readings below this (sensor noise floor)
MIN_CONTOUR_AREA = 300   # px² — blobs smaller than this are skipped
ANGLE_TOL_DEG    = 15.0  # max angular difference for two edges to be parallel
MIN_SEP_PX       = 10    # min perpendicular separation between parallel edges
BORDER_PX        = 40    # ignore this many pixels around the image edge
# ─────────────────────────────────────────────────────────────────────────────

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ParallelEdgeDetector(Node):

    def __init__(self):
        super().__init__('parallel_edges')
        self._fx = self._fy = self._cx = self._cy = None

        # Intrinsics — grabbed once from camera_info
        self.create_subscription(
            CameraInfo, '/aurora/ir/camera_info', self._info_cb, SENSOR_QOS)

        # Synchronised RGB + depth
        rgb_sub   = message_filters.Subscriber(
            self, Image, '/aurora/rgb/image_raw',   qos_profile=SENSOR_QOS)
        depth_sub = message_filters.Subscriber(
            self, Image, '/aurora/depth/image_raw', qos_profile=SENSOR_QOS)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._frame_cb)

        self.get_logger().info('parallel_edges ready — waiting for frames …')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo):
        if self._fx is None:
            self._fx = msg.k[0];  self._fy = msg.k[4]
            self._cx = msg.k[2];  self._cy = msg.k[5]
            self.get_logger().info(
                f'Intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
                f'cx={self._cx:.1f} cy={self._cy:.1f}')

    def _frame_cb(self, rgb_msg: Image, depth_msg: Image):
        # ── Decode ────────────────────────────────────────────────────────────
        rgb = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(
            rgb_msg.height, rgb_msg.width, 3).copy()
        depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
            depth_msg.height, depth_msg.width).copy()

        # ── Border mask ───────────────────────────────────────────────────────
        b = BORDER_PX
        depth[:b, :]  = 0;  depth[-b:, :]  = 0
        depth[:, :b]  = 0;  depth[:, -b:]  = 0

        # ── Depth gaps = cube silhouette ──────────────────────────────────────
        # Interior zero pixels are invalid depth readings — the cube's surface
        # doesn't return a valid reading, leaving a clean gap in the depth image.
        interior  = np.zeros(depth.shape, dtype=bool)
        interior[b:-b, b:-b] = True
        cube_mask = ((depth == 0) & interior).astype(np.uint8)

        # Morphological clean — remove speckle, close small gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        cube_mask = cv2.morphologyEx(cube_mask, cv2.MORPH_OPEN,  kernel)
        cube_mask = cv2.morphologyEx(cube_mask, cv2.MORPH_CLOSE, kernel)

        # ── Largest contour = cube silhouette ─────────────────────────────────
        contours, _ = cv2.findContours(
            cube_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._show(rgb, depth, cube_mask, None, None)
            return

        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
            self._show(rgb, depth, cube_mask, contour, None)
            return

        # ── Polygon approximation → edge list ─────────────────────────────────
        peri   = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.08 * peri, True)
        pts    = approx[:, 0, :]     # (N, 2)

        self.get_logger().info(f'Polygon vertices: {len(pts)}')
        if len(pts) < 3:
            self._show(rgb, depth, cube_mask, contour, None, approx)
            return

        edges = []
        n = len(pts)
        for i in range(n):
            p1 = pts[i];  p2 = pts[(i + 1) % n]
            dx, dy = float(p2[0] - p1[0]), float(p2[1] - p1[1])
            length = float(np.hypot(dx, dy))
            angle  = atan2(dy, dx) % pi
            mx, my = (p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0
            edges.append({'mx': mx, 'my': my, 'angle': angle,
                          'length': length, 'p1': p1, 'p2': p2})

        # ── Best parallel pair ────────────────────────────────────────────────
        angle_tol  = ANGLE_TOL_DEG * pi / 180.0
        best_score = -1.0
        best_pair  = None

        for i in range(len(edges)):
            e1 = edges[i]
            for j in range(i + 1, len(edges)):
                e2 = edges[j]

                da = abs(e1['angle'] - e2['angle'])
                if da > pi / 2:
                    da = pi - da
                if da > angle_tol:
                    continue

                mean_angle = (e1['angle'] + e2['angle']) / 2.0
                dx = e2['mx'] - e1['mx'];  dy = e2['my'] - e1['my']
                sep = abs(-dx * sin(mean_angle) + dy * cos(mean_angle))

                if sep < MIN_SEP_PX:
                    continue

                score = e1['length'] + e2['length']
                if score > best_score:
                    best_score = score
                    best_pair  = (i, j, mean_angle, sep)

        result = None
        if best_pair is not None:
            i, j, angle_rad, sep_px = best_pair
            e1, e2 = edges[i], edges[j]
            midpoint = ((e1['mx'] + e2['mx']) / 2.0,
                        (e1['my'] + e2['my']) / 2.0)

            # Z — sample valid depth in a ring just outside the gap (table surface)
            ring_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            ring_mask   = cv2.dilate(cube_mask, ring_kernel)
            ring        = (ring_mask > 0) & (cube_mask == 0) & interior & (depth > MIN_VALID_MM)
            z_mm = float(np.median(depth[ring])) if ring.any() else 0.0

            # 3D back-projection of midpoint using table Z
            mu = int(np.clip(midpoint[0], 0, depth.shape[1] - 1))
            mv = int(np.clip(midpoint[1], 0, depth.shape[0] - 1))
            fx = self._fx or 600.0;  fy = self._fy or 600.0
            cx = self._cx or depth.shape[1] / 2.0
            cy = self._cy or depth.shape[0] / 2.0
            z_m = z_mm / 1000.0
            x_m = (mu - cx) / fx * z_m
            y_m = (mv - cy) / fy * z_m

            result = {
                'midpoint_px': midpoint,
                'angle_rad':   angle_rad,
                'sep_px':      sep_px,
                'xyz_m':       (x_m, y_m, z_m),
                'e1': e1, 'e2': e2,
            }
            self.get_logger().info(
                f'Midpoint ({midpoint[0]:.0f}, {midpoint[1]:.0f}) px  '
                f'angle {degrees(angle_rad):.1f}°  sep {sep_px:.0f} px  '
                f'3D ({x_m*100:.1f}, {y_m*100:.1f}, {z_m*100:.1f}) cm')

        self._show(rgb, depth, cube_mask, contour, result, approx)

    # ── Display ───────────────────────────────────────────────────────────────

    def _show(self, rgb, depth, mask, contour, result, approx=None):
        vis = rgb.copy()

        if mask is not None:
            tint = vis.copy();  tint[mask > 0] = [0, 180, 0]
            vis = cv2.addWeighted(vis, 0.7, tint, 0.3, 0)
            cv2.putText(vis, f'gap: {int(mask.sum())} px2',
                        (10, vis.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if contour is not None:
            cv2.drawContours(vis, [contour], -1, (0, 200, 0), 1)
        if approx is not None:
            cv2.drawContours(vis, [approx],  -1, (0, 255, 255), 2)

        if result is not None:
            for e in (result['e1'], result['e2']):
                cv2.line(vis, tuple(e['p1']), tuple(e['p2']), (0, 255, 0), 3)

            mu = int(result['midpoint_px'][0]);  mv = int(result['midpoint_px'][1])
            cv2.circle(vis, (mu, mv), 6, (0, 0, 255), -1)

            a = result['angle_rad']
            cv2.arrowedLine(vis, (mu, mv),
                            (int(mu + 40 * cos(a + pi / 2)),
                             int(mv + 40 * sin(a + pi / 2))),
                            (255, 0, 0), 2)

            x_m, y_m, z_m = result['xyz_m']
            cv2.putText(vis,
                f'3D: ({x_m*100:.1f}, {y_m*100:.1f}, {z_m*100:.1f}) cm  '
                f'{degrees(result["angle_rad"]):.1f} deg',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Colorised depth
        d = depth.astype(np.float32);  v = d > 0
        if v.any():
            d_min, d_max = d[v].min(), d[v].max()
            if d_max > d_min:
                norm = np.where(v, (d - d_min) / (d_max - d_min), 0.0)
                dc   = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
                dc[~v] = 0
                cv2.imshow('depth', dc)

        cv2.imshow('parallel edges', vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ParallelEdgeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
