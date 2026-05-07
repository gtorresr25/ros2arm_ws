#!/usr/bin/env python3
"""
Floor filter debug script.

Subscribes to the Aurora RGB + depth streams, fits a floor plane using
least squares (SVD), masks pixels above the floor, and displays the result
with cv2.imshow.

Run with:
    python3 scripts/floor_filter.py

Press Q to quit.

Parameters (edit below):
    MAX_HEIGHT_M   — keep pixels this far above the floor (metres)
    SUBSAMPLE      — use 1-in-N depth pixels for plane fitting (speed vs accuracy)
"""

import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
import message_filters

# --- tunable parameters ---------------------------------------------------
MAX_HEIGHT_M  = 0.100   # 100 mm above floor = candidate zone
MIN_HEIGHT_M  = 0.003   # 100 mm above floor = candidate zone
SUBSAMPLE     = 16      # use every Nth pixel for plane fitting
PLANE_AVG_N   = 5      # number of frames to average the floor plane over
DILATE_PX        = 20   # pixels to expand the candidate mask outward
MIN_CONTOUR_AREA = 200  # px² — blobs smaller than this are ignored
GRAD_THRESHOLD_M  = 0.05  # m — row-to-row depth jump that signals wall/background
# --------------------------------------------------------------------------

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def fit_floor_plane(points: np.ndarray):
    """
    Fit a plane to (N, 3) XYZ points using SVD (least squares).
    Returns (normal, d) where normal is a unit vector and d is the offset,
    such that normal . p = d for points on the plane.
    """
    centroid = points.mean(axis=0)
    _, _, Vt = np.linalg.svd(points - centroid)
    normal = Vt[-1]
    # For a forward-angled camera, the floor normal points upward in camera
    # space — which means negative Y (Y axis points down in camera frame).
    # Ensure the normal points away from the floor toward the camera.
    if normal[1] > 0:
        normal = -normal
    d = float(normal @ centroid)
    return normal, d


class FloorFilter(Node):

    def __init__(self):
        super().__init__('floor_filter')
        self._fx = self._fy = self._cx = self._cy = None

        # Rolling buffer for plane averaging
        self._normal_history = []   # list of (normal, d) tuples
        self._avg_normal = None
        self._avg_d      = None

        # FPS tracking
        self._last_ts = None
        self._fps     = 0.0

        # Precomputed pixel direction grids (set once intrinsics are known)
        self._ray_x = None   # (H, W)  (u - cx) / fx
        self._ray_y = None   # (H, W)  (v - cy) / fy

        # Camera info — grab intrinsics once
        self.info_sub = self.create_subscription(
            CameraInfo, '/aurora/ir/camera_info',
            self._info_cb, SENSOR_QOS)

        # Synchronised RGB + depth
        rgb_sub   = message_filters.Subscriber(self, Image, '/aurora/rgb/image_raw',   qos_profile=SENSOR_QOS)
        depth_sub = message_filters.Subscriber(self, Image, '/aurora/depth/image_raw', qos_profile=SENSOR_QOS)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._frame_cb)

        self.get_logger().info('floor_filter ready — waiting for frames...')

    def _info_cb(self, msg: CameraInfo):
        if self._fx is None:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]
            # Precompute per-pixel ray directions — constant for this camera
            h, w = msg.height, msg.width
            uu, vv = np.meshgrid(np.arange(w, dtype=np.float32),
                                 np.arange(h, dtype=np.float32))
            self._ray_x = (uu - self._cx) / self._fx
            self._ray_y = (vv - self._cy) / self._fy
            self.get_logger().info(
                f'Intrinsics set — fx={self._fx:.1f} fy={self._fy:.1f} '
                f'cx={self._cx:.1f} cy={self._cy:.1f}')

    def _frame_cb(self, rgb_msg: Image, depth_msg: Image):
        # Use fallback intrinsics until camera_info arrives
        fx = self._fx or 600.0
        fy = self._fy or 600.0
        cx = self._cx or (depth_msg.width  / 2.0)
        cy = self._cy or (depth_msg.height / 2.0)

        # --- decode images ------------------------------------------------
        rgb = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(
            rgb_msg.height, rgb_msg.width, 3).copy()
        depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
            depth_msg.height, depth_msg.width).copy()

        h, w = depth.shape

        # --- convert depth to metres, mask out zeros ----------------------
        Z = depth.astype(np.float32) / 1000.0   # mm → metres
        valid = Z > 0.1

        # Use precomputed ray grids if available, else build fallback once
        if self._ray_x is None or self._ray_x.shape != (h, w):
            uu, vv = np.meshgrid(np.arange(w, dtype=np.float32),
                                 np.arange(h, dtype=np.float32))
            self._ray_x = (uu - cx) / fx
            self._ray_y = (vv - cy) / fy

        ray_x = self._ray_x
        ray_y = self._ray_y

        # --- depth gradient filter: find floor rows, exclude wall/bg ---------
        Z_masked = np.where(valid, Z, np.nan)
        with np.errstate(all='ignore'):
            row_median = np.nanmedian(Z_masked, axis=1)   # (H,)

        # Row-to-row gradient — find last big jump scanning from bottom up
        grad = np.abs(np.diff(row_median))
        floor_start_row = 0   # default: use all rows
        for i in range(len(grad) - 1, 0, -1):
            if np.isfinite(grad[i]) and grad[i] > GRAD_THRESHOLD_M:
                floor_start_row = i + 1
                break

        self._floor_start_row = floor_start_row   # store for plot

        # Build row mask — only rows at or below the detected floor start
        row_mask = np.zeros(h, dtype=bool)
        row_mask[floor_start_row:] = True
        fit_mask = valid & row_mask[:, np.newaxis]

        # --- subsample for plane fitting ----------------------------------
        mask_sub = fit_mask[::SUBSAMPLE, ::SUBSAMPLE]
        Zs = Z[::SUBSAMPLE, ::SUBSAMPLE][mask_sub]
        Xs = ray_x[::SUBSAMPLE, ::SUBSAMPLE][mask_sub] * Zs
        Ys = ray_y[::SUBSAMPLE, ::SUBSAMPLE][mask_sub] * Zs
        pts = np.stack([Xs, Ys, Zs], axis=1)

        if len(pts) < 50:
            self.get_logger().warn('Too few valid depth points — skipping frame')
            return

        # --- fit floor plane and average over last N frames ---------------
        normal, d = fit_floor_plane(pts)

        self._normal_history.append((normal, d))
        if len(self._normal_history) > PLANE_AVG_N:
            self._normal_history.pop(0)

        normals = np.array([n for n, _ in self._normal_history])
        ds      = np.array([d for _, d in self._normal_history])
        self._avg_normal = normals.mean(axis=0)
        self._avg_normal /= np.linalg.norm(self._avg_normal)   # re-normalise
        self._avg_d = float(ds.mean())

        normal = self._avg_normal
        d      = self._avg_d

        # --- height above floor for every pixel ---------------------------
        # Expand: normal·p - d = nx*(ray_x*Z) + ny*(ray_y*Z) + nz*Z - d
        #                      = (nx*ray_x + ny*ray_y + nz) * Z - d
        # Precompute the per-pixel scale factor (cheap, avoids (H,W,3) array)
        scale = ray_x * normal[0] + ray_y * normal[1] + normal[2]
        height = scale * Z - d                                    # (H, W)

        # --- candidate mask: above floor, below MAX_HEIGHT_M --------------
        candidate = valid & (height > MIN_HEIGHT_M) & (height < MAX_HEIGHT_M)

        # --- dilate mask to recover object edges lost to depth noise ------
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (DILATE_PX * 2 + 1, DILATE_PX * 2 + 1))
        candidate_dilated = cv2.dilate(
            candidate.astype(np.uint8), kernel, iterations=1).astype(bool)

        # --- FPS ----------------------------------------------------------
        now = time.monotonic()
        if self._last_ts is not None:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / (now - self._last_ts))
        self._last_ts = now

        # --- overlay on RGB -----------------------------------------------
        overlay = rgb.copy()
        overlay[candidate_dilated] = [0, 200, 0]    # green

        result = cv2.addWeighted(rgb, 0.5, overlay, 0.5, 0)

        # --- draw sample grid dots ----------------------------------------
        # yellow = used in plane fit,  red = valid depth but excluded (wall)
        rows_s = np.arange(0, h, SUBSAMPLE)
        cols_s = np.arange(0, w, SUBSAMPLE)
        for r in rows_s:
            for c in cols_s:
                if fit_mask[r, c]:
                    cv2.circle(result, (c, r), 2, (0, 220, 220), -1)   # yellow
                elif valid[r, c]:
                    cv2.circle(result, (c, r), 2, (0, 0, 180), -1)     # red

        # --- annotate -----------------------------------------------------
        cv2.putText(result,
                    f'floor normal: {normal.round(2)}  d={d:.3f}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(result,
                    f'candidates: {candidate_dilated.sum()}  FPS: {self._fps:.1f}',
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

        # --- colorised depth image ----------------------------------------
        depth_vis = np.zeros_like(Z, dtype=np.uint8)
        valid_z = Z[valid]
        if len(valid_z) > 0:
            z_min, z_max = float(valid_z.min()), float(valid_z.max())
            if z_max > z_min:
                depth_norm = np.where(valid, (Z - z_min) / (z_max - z_min), 0.0)
                depth_vis = (depth_norm * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        depth_color[~valid] = 0   # black for invalid pixels

        cv2.imshow('floor filter', result)
        cv2.imshow('depth image', depth_color)
        cv2.imshow('depth profile', self._draw_depth_profile(
            Z, valid, normal, d, getattr(self, '_floor_start_row', 0)))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Quit requested')
            rclpy.shutdown()

    def _draw_depth_profile(self, Z: np.ndarray, valid: np.ndarray,
                             normal: np.ndarray, d: float,
                             floor_start_row: int = 0):
        """
        Draw a side-view depth profile plot:
          X axis — image row (0 = top, H = bottom)
          Y axis — median depth across all valid columns at that row (metres)
        Overlays the plane-predicted depth as a red line and a green vertical
        line marking where the gradient filter cut off the wall/background.
        """
        PLOT_W, PLOT_H = 600, 400
        MARGIN = 40

        h = Z.shape[0]

        # Median depth per row (ignore invalid pixels)
        Z_masked = np.where(valid, Z, np.nan)
        with np.errstate(all='ignore'):
            row_depth = np.nanmedian(Z_masked, axis=1)   # (H,)

        # Plane-predicted depth per row — at image centre column (ray_x = 0)
        rows = np.arange(h, dtype=np.float32)
        ray_y_centre = (rows - (self._cy or h / 2.0)) / (self._fy or 600.0)
        # plane: nx*ray_x*Z + ny*ray_y*Z + nz*Z = d  →  Z = d / (nx*0 + ny*ray_y + nz)
        denom = normal[1] * ray_y_centre + normal[2]
        with np.errstate(divide='ignore', invalid='ignore'):
            plane_depth = np.where(np.abs(denom) > 1e-6, d / denom, np.nan)

        # Depth range for scaling
        valid_depths = row_depth[np.isfinite(row_depth)]
        if len(valid_depths) == 0:
            return np.zeros((PLOT_H, PLOT_W, 3), dtype=np.uint8)
        d_min = max(0.0, float(np.nanmin(valid_depths)) - 0.1)
        d_max = float(np.nanmax(valid_depths)) + 0.1

        plot = np.ones((PLOT_H, PLOT_W, 3), dtype=np.uint8) * 30   # dark bg

        def to_px(row_i, depth_val):
            px = int(MARGIN + (row_i / h) * (PLOT_W - 2 * MARGIN))
            py = int(MARGIN + (1.0 - (depth_val - d_min) / (d_max - d_min))
                     * (PLOT_H - 2 * MARGIN))
            return px, py

        # Draw grid lines
        for depth_tick in np.linspace(d_min, d_max, 5):
            _, py = to_px(0, depth_tick)
            cv2.line(plot, (MARGIN, py), (PLOT_W - MARGIN, py), (60, 60, 60), 1)
            cv2.putText(plot, f'{depth_tick:.2f}m', (2, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)

        # Draw measured depth (white)
        prev = None
        for i, dep in enumerate(row_depth):
            if not np.isfinite(dep):
                prev = None
                continue
            pt = to_px(i, dep)
            if prev:
                cv2.line(plot, prev, pt, (200, 200, 200), 1)
            prev = pt

        # Draw plane-predicted depth (red)
        prev = None
        for i, dep in enumerate(plane_depth):
            if not np.isfinite(dep) or dep < 0:
                prev = None
                continue
            pt = to_px(i, dep)
            if prev:
                cv2.line(plot, prev, pt, (60, 60, 220), 1)
            prev = pt

        # Draw gradient cutoff line (green vertical)
        if floor_start_row > 0:
            cut_px, _ = to_px(floor_start_row, d_min)
            cv2.line(plot, (cut_px, MARGIN), (cut_px, PLOT_H - MARGIN),
                     (0, 200, 0), 1)
            cv2.putText(plot, f'row {floor_start_row}', (cut_px + 3, MARGIN + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 0), 1)

        # Labels
        cv2.putText(plot,
                    'white=measured  blue=plane fit  green=wall cutoff',
                    (MARGIN, PLOT_H - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (160, 160, 160), 1)

        return plot


def main():
    rclpy.init()
    node = FloorFilter()
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
