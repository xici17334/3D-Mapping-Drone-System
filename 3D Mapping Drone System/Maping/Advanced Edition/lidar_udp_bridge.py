#!/usr/bin/env python3
# Author: Ye Li 
# Description:
#   Receive LiDAR frames via UDP (custom CSV "F,..." / "P,...") and publish
#   them as sensor_msgs/LaserScan on /scan.
#
#   Expected packet lines:
#     F,<seq>,<lsn>,<fsa_deg>,<lsa_deg>
#     P,<idx>,<angle_raw?>,<dist_mm>,<intensity>
#
#   Notes:
#     - Publishes early when: fill_ratio reached or frame_timeout exceeded
#     - Ensures angle_increment is positive (reverses arrays if needed)
#     - Optionally holds previous ranges to reduce empty bins

import socket
import threading
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data


class UdpToScan(Node):
    def __init__(self):
        super().__init__("udp_to_scan")

        # ---------------- Parameters ----------------
        self.declare_parameter('port', 9000)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('frame_timeout', 0.25)
        self.declare_parameter('fill_ratio', 0.80)
        self.declare_parameter('min_pub_period', 0.08)   # ~12.5 Hz max
        self.declare_parameter('hold_prev_on_gap', True) # fill gaps with last good frame

        self.PORT = int(self.get_parameter('port').value)
        self.FRAME_ID = str(self.get_parameter('frame_id').value)
        self.RANGE_MIN = float(self.get_parameter('range_min').value)
        self.RANGE_MAX = float(self.get_parameter('range_max').value)
        self.FRAME_TIMEOUT = float(self.get_parameter('frame_timeout').value)
        self.FILL_RATIO = float(self.get_parameter('fill_ratio').value)
        self.MIN_PUB = float(self.get_parameter('min_pub_period').value)
        self.HOLD_PREV = bool(self.get_parameter('hold_prev_on_gap').value)

        # ---------------- ROS I/O ----------------
        self.pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.PORT))
        self.sock.settimeout(0.2)

        # Current frame buffers
        self.cur_seq = None
        self.cur_lsn = 0
        self.cur_fsa = 0.0
        self.cur_lsa = 0.0
        self.cur_ranges = []
        self.cur_intens = []
        self.cur_filled = None
        self.cur_received = 0
        self.last_frame_time = time.time()
        self.last_pub = 0.0

        self.prev_ranges = None

        # Start RX thread
        t = threading.Thread(target=self._rx_loop, daemon=True)
        t.start()
        self.get_logger().info(f"Listening UDP :{self.PORT} → /scan (frame={self.FRAME_ID})")

    def _rx_loop(self):
        """Receive UDP packets, parse lines, and assemble frames."""
        buf = b""
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(8192)
            except socket.timeout:
                # On timeout, check whether to publish a partial frame
                self._maybe_publish(force_timeout=True)
                continue
            except Exception as e:
                self.get_logger().warn(f"UDP recv error: {e}")
                continue

            buf += data
            # Assume newline-aligned lines from sender
            lines = buf.decode(errors="ignore").splitlines(keepends=False)
            buf = b""

            for line in lines:
                if not line:
                    continue
                try:
                    if line.startswith("F,"):
                        parts = line.split(",")
                        seq = int(parts[1]); lsn = int(parts[2])
                        fsa = float(parts[3]) * math.pi / 180.0
                        lsa = float(parts[4]) * math.pi / 180.0
                        self._start_new_frame(seq, lsn, fsa, lsa)
                    elif line.startswith("P,"):
                        parts = line.split(",")
                        idx = int(parts[1])
                        dist = float(parts[3]) / 1000.0    # mm -> m
                        inten = float(parts[4])
                        self._add_point(idx, dist, inten)
                except Exception as e:
                    self.get_logger().warn(f"parse error: {e}; line={line[:80]}")

            self._maybe_publish()

    def _start_new_frame(self, seq, lsn, fsa, lsa):
        """Start a new incoming frame with meta data."""
        self.cur_seq = seq
        self.cur_lsn = max(0, int(lsn))
        self.cur_fsa = fsa
        self.cur_lsa = lsa

        if self.HOLD_PREV and self.prev_ranges and len(self.prev_ranges) == self.cur_lsn:
            self.cur_ranges = self.prev_ranges.copy()
        else:
            self.cur_ranges = [float("inf")] * self.cur_lsn

        self.cur_intens = [0.0] * self.cur_lsn
        self.cur_filled = [False] * self.cur_lsn
        self.cur_received = 0
        self.last_frame_time = time.time()

    def _add_point(self, idx, dist, inten):
        """Insert a single beam into the current frame (with validity check)."""
        if 0 <= idx < self.cur_lsn:
            if not self.cur_filled[idx]:
                self.cur_received += 1
                self.cur_filled[idx] = True
            val = dist if (self.RANGE_MIN <= dist <= self.RANGE_MAX) else float("inf")
            self.cur_ranges[idx] = val
            self.cur_intens[idx] = inten

    def _maybe_publish(self, force_timeout: bool = False):
        """Decide when to publish: enough beams or timeout reached."""
        if self.cur_lsn <= 0:
            return
        now = time.time()
        ratio = self.cur_received / float(self.cur_lsn) if self.cur_lsn > 0 else 0.0
        timeout_hit = (now - self.last_frame_time) >= self.FRAME_TIMEOUT
        should_pub = (ratio >= self.FILL_RATIO) or (force_timeout and timeout_hit) or ((not force_timeout) and timeout_hit)
        if should_pub:
            if now - self.last_pub >= self.MIN_PUB:
                self._publish()
            # Clear current frame to wait for the next one
            self.cur_lsn = 0

    def _publish(self):
        """Convert the assembled frame into LaserScan and publish."""
        span = self.cur_lsa - self.cur_fsa
        if span < -math.pi:
            span += 2 * math.pi
        if span > math.pi:
            span -= 2 * math.pi

        angle_increment = span / max(1, (self.cur_lsn - 1))
        angle_min = self.cur_fsa
        angle_max = self.cur_fsa + angle_increment * (self.cur_lsn - 1)

        # Ensure increasing angles by reversing arrays if needed
        if angle_increment < 0:
            self.cur_ranges.reverse()
            self.cur_intens.reverse()
            angle_increment = abs(angle_increment)
            angle_min = self.cur_lsa
            angle_max = self.cur_lsa + angle_increment * (self.cur_lsn - 1)

        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.FRAME_ID
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.scan_time = max(self.MIN_PUB, 0.01)
        scan.time_increment = scan.scan_time / max(1.0, float(self.cur_lsn))
        scan.range_min = self.RANGE_MIN
        scan.range_max = self.RANGE_MAX
        scan.ranges = self.cur_ranges
        scan.intensities = self.cur_intens

        self.prev_ranges = list(scan.ranges)
        self.pub.publish(scan)
        self.last_pub = time.time()
        # Optional: uncomment for verbose logging
        # self.get_logger().info(f"pub /scan: lsn={self.cur_lsn}, recv={self.cur_received}, ratio={self.cur_received/max(1,self.cur_lsn):.2f}")


def main():
    rclpy.init()
    n = UdpToScan()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
