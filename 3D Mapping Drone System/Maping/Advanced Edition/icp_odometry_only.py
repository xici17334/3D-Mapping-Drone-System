#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Ye Li 
# Description:
#   Robust 2D ICP-based laser odometry (ICP only, no IMU).
#   Subscribes: /scan (sensor_msgs/LaserScan)
#   Publishes : /odom_icp (nav_msgs/Odometry)
#   TF        : odom -> base_link
#
#   Key design points:
#     - Nearest-neighbor + weighted SVD (Huber weights) to suppress outliers
#     - Keyframe policy to reduce long-term drift
#     - Quality gates (min_pairs, median residual threshold)
#     - Step capping and EMA smoothing for visual stability
#     - Runtime reset via /icp_reset (std_srvs/Empty)

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Empty


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw (rad) to planar quaternion (z, w)."""
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def ranges_to_points(scan: LaserScan, max_range=20.0, downsample=1) -> np.ndarray:
    """
    Convert LaserScan to (x, y) points in the laser frame.
    - 3-point median filter for spike suppression
    - Downsampling controlled by 'downsample'
    - Filter points out of [range_min, min(range_max, max_range)]
    Returns float32 array Nx2 (possibly empty).
    """
    if scan.angle_increment == 0.0 or len(scan.ranges) == 0:
        return np.zeros((0, 2), np.float32)
    rmin = max(scan.range_min, 0.05)
    rmax = min(scan.range_max, max_range)
    step = max(1, int(downsample))
    pts, a = [], scan.angle_min
    rng = list(scan.ranges)
    n = len(rng)

    for i in range(0, n, step):
        j0 = max(0, i - 1)
        j1 = i
        j2 = min(n - 1, i + 1)
        r = sorted([rng[j0], rng[j1], rng[j2]])[1]
        if math.isfinite(r) and (rmin <= r <= rmax):
            pts.append((r * math.cos(a), r * math.sin(a)))
        a += scan.angle_increment * step

    if not pts:
        return np.zeros((0, 2), np.float32)
    return np.asarray(pts, np.float32)


def nearest_neighbors(A: np.ndarray, B: np.ndarray, max_dist=1.5):
    """
    Brute-force nearest-neighbor matching from A to B (sufficient for small point sets).
    Returns (A_sel, B_sel, residuals).
    """
    if len(A) == 0 or len(B) == 0:
        return (np.zeros((0, 2), np.float32),
                np.zeros((0, 2), np.float32),
                np.zeros((0,), np.float32))
    md2 = max_dist * max_dist
    A_sel, B_sel, R = [], [], []
    for a in A:
        d2 = np.sum((B - a) ** 2, axis=1)
        j = int(np.argmin(d2))
        if d2[j] <= md2:
            A_sel.append(a)
            B_sel.append(B[j])
            R.append(math.sqrt(float(d2[j])))
    if not A_sel:
        return (np.zeros((0, 2), np.float32),
                np.zeros((0, 2), np.float32),
                np.zeros((0,), np.float32))
    return (np.asarray(A_sel, np.float32),
            np.asarray(B_sel, np.float32),
            np.asarray(R, np.float32))


def best_fit_transform_w(A, B, w):
    """
    Weighted 2D rigid registration.
    Inputs:
      A, B: Nx2 matched points
      w   : Nx1 weights (e.g., Huber)
    Outputs:
      R(2x2), t(2), yaw
    """
    if A.shape[0] < 3 or B.shape[0] < 3:
        return np.eye(2, dtype=np.float32), np.zeros(2, dtype=np.float32), 0.0
    w = w.reshape(-1, 1)
    W = float(np.sum(w))
    ca = np.sum(w * A, axis=0) / max(W, 1e-6)
    cb = np.sum(w * B, axis=0) / max(W, 1e-6)
    A0, B0 = A - ca, B - cb
    H = (A0 * w).T @ B0
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = cb - (R @ ca)
    yaw = math.atan2(R[1, 0], R[0, 0])
    return R.astype(np.float32), t.astype(np.float32), yaw


def huber_weights(resid, delta):
    """Huber weights: 1 for small residuals; delta/r for large ones."""
    w = np.ones_like(resid, dtype=np.float32)
    mask = resid > delta
    w[mask] = (delta / np.maximum(resid[mask], 1e-6)).astype(np.float32)
    return w


class IcpOdom(Node):
    def __init__(self):
        super().__init__('icp_odometry_robust')

        # Frames
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        # ICP parameters (tuned for small point counts and noisy scans)
        self.max_range     = 20.0
        self.downsample    = 1
        self.max_corr_dist = 1.5
        self.max_icp_iter  = 30
        self.min_pairs     = 8
        self.huber_delta   = 0.15
        self.max_med_resid = 0.25

        # Keyframe thresholds
        self.key_xy  = 0.25
        self.key_yaw = math.radians(10.0)

        # Step limits + EMA smoothing
        self.max_step_xy  = 0.40
        self.max_step_yaw = math.radians(20)
        self.pose_alpha   = 0.2

        # State (raw and filtered pose)
        self.x = self.y = self.yaw = 0.0
        self.x_f = self.y_f = self.yaw_f = 0.0
        self.prev_pts = None
        self.keyframe_pts = None
        self.key_x = self.key_y = self.key_yaw = 0.0

        # ROS I/O
        self.odom_pub = self.create_publisher(Odometry, '/odom_icp', 10)
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.srv_reset = self.create_service(Empty, 'icp_reset', self._on_reset)

        self.get_logger().info("Robust ICP Laser Odometry started (listening /scan)")

    def _on_reset(self, req, resp):
        """Reset cumulative pose and keyframe."""
        self.x = self.y = self.yaw = 0.0
        self.x_f = self.y_f = self.yaw_f = 0.0
        self.prev_pts = None
        self.keyframe_pts = None
        self.key_x = self.key_y = self.key_yaw = 0.0
        self.get_logger().warn("ICP pose reset to (0,0,0).")
        return resp

    def scan_cb(self, scan: LaserScan):
        """Main callback: convert scan → points → ICP vs keyframe → integrate pose → publish."""
        # 1) Convert to points (with median and downsample)
        pts = ranges_to_points(scan, self.max_range, self.downsample)
        n_fin = pts.shape[0]
        if n_fin < 15:
            self.get_logger().warn(f"/scan finite points {n_fin} < 15, skip")
            return

        # 2) Initialize keyframe with the first valid scan
        if self.prev_pts is None:
            self.prev_pts = pts
            self.keyframe_pts = pts.copy()
            self.key_x = self.x
            self.key_y = self.y
            self.key_yaw = self.yaw
            self.get_logger().info(f"Init keyframe with {n_fin} pts")
            return

        # 3) ICP loop: align current frame (src) to keyframe (dst)
        src = pts.copy()
        dst = self.keyframe_pts.copy()
        R_acc = np.eye(2, dtype=np.float32)
        t_acc = np.zeros(2, dtype=np.float32)
        used_pairs = 0
        med_resid = 999.0

        for _ in range(self.max_icp_iter):
            A, B, resid = nearest_neighbors(src, dst, self.max_corr_dist)
            if A.shape[0] < self.min_pairs:
                break
            used_pairs = A.shape[0]
            med_resid = float(np.median(resid)) if resid.size > 0 else 999.0

            w = huber_weights(resid, self.huber_delta)
            R, t, _ = best_fit_transform_w(A, B, w)

            # Apply and accumulate
            src = (R @ src.T).T + t
            R_acc = R @ R_acc
            t_acc = R @ t_acc + t

            # Early stop if tiny update
            dth = math.atan2(R[1, 0], R[0, 0])
            if np.linalg.norm(t) < 1e-4 and abs(dth) < 1e-4:
                break

        # 4) Quality gate
        if used_pairs < self.min_pairs or med_resid > self.max_med_resid:
            self.get_logger().warn(f"Reject frame: pairs={used_pairs}, med_resid={med_resid:.3f}m")
            self.prev_pts = pts
            return

        # 5) Extract increments
        d_yaw = math.atan2(R_acc[1, 0], R_acc[0, 0])
        dx, dy = float(t_acc[0]), float(t_acc[1])

        # 6) Step capping
        d_norm = math.hypot(dx, dy)
        if d_norm > self.max_step_xy:
            s = self.max_step_xy / max(d_norm, 1e-6)
            dx *= s
            dy *= s
        if abs(d_yaw) > self.max_step_yaw:
            d_yaw = math.copysign(self.max_step_yaw, d_yaw)

        # 7) Integrate in world (odom) frame
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        self.x  += ( cy * dx - sy * dy)
        self.y  += ( sy * dx + cy * dy)
        self.yaw =  math.atan2(math.sin(self.yaw + d_yaw), math.cos(self.yaw + d_yaw))

        # 8) EMA smoothing (for published pose only)
        a = self.pose_alpha
        self.x_f = (1 - a) * self.x_f + a * self.x
        self.y_f = (1 - a) * self.y_f + a * self.y
        yaw_err = math.atan2(math.sin(self.yaw - self.yaw_f), math.cos(self.yaw - self.yaw_f))
        self.yaw_f = math.atan2(math.sin(self.yaw_f + a * yaw_err), math.cos(self.yaw_f + a * yaw_err))

        # 9) Keyframe update
        if (math.hypot(self.x - self.key_x, self.y - self.key_y) > self.key_xy or
            abs(math.atan2(math.sin(self.yaw - self.key_yaw), math.cos(self.yaw - self.key_yaw))) > self.key_yaw):
            self.keyframe_pts = pts.copy()
            self.key_x = self.x
            self.key_y = self.y
            self.key_yaw = self.yaw
            self.get_logger().info(f"New keyframe: pairs={used_pairs}, med_resid={med_resid:.3f}m")

        # 10) Publish
        self._publish(scan.header.stamp)
        self.prev_pts = pts
        self.get_logger().info(
            f"Δ=({dx:.3f},{dy:.3f},{d_yaw:.3f}) | "
            f"pos=({self.x_f:.3f},{self.y_f:.3f},{self.yaw_f:.3f}) | "
            f"pairs={used_pairs}, med_resid={med_resid:.3f}m, pts={n_fin}"
        )

    def _publish(self, stamp):
        """Publish nav_msgs/Odometry and TF (odom->base_link)."""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = self.x_f
        odom.pose.pose.position.y = self.y_f
        odom.pose.pose.orientation = yaw_to_quat(self.yaw_f)

        # Simple covariance (x, y, yaw) for visualization
        s2 = 0.05 ** 2
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[35] = s2
        odom.pose.covariance = cov
        self.odom_pub.publish(odom)

        # Sync TF with odometry
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x_f
        t.transform.translation.y = self.y_f
        t.transform.rotation = odom.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    n = IcpOdom()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
