#!/usr/bin/env python3
"""
ROS2 Node: UDP → /scan (sensor_msgs/LaserScan)

Function:
- Listens on a UDP port for point data from a LiDAR device
- Input format (per line): "P,idx,angle_deg,dist_mm,intensity"
- Collects one full 360° revolution worth of points into bins
- Publishes a LaserScan message on the topic "/scan"

Author: Li Ye 
"""

import socket, threading, time, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

# -------------------------
# Network and frame settings
# -------------------------
PORT = 9000           # UDP listening port
FRAME_ID = "laser"    # Frame ID for published LaserScan messages

# -------------------------
# LiDAR specifications
# -------------------------
RANGE_MIN = 0.05      # Minimum measurable distance (meters)
RANGE_MAX = 12.0      # Maximum measurable distance (meters)

# -------------------------
# LaserScan angular resolution
# -------------------------
BINS = 720                               # number of angular bins (0.5° resolution)
ANGLE_MIN = -math.pi                     # -180 degrees
ANGLE_MAX = math.pi                      # +180 degrees
ANGLE_STEP = (ANGLE_MAX - ANGLE_MIN) / BINS   # angular step per bin

def ang_to_bin(rad):
    """
    Normalize angle into [-pi, +pi) and map it into the corresponding bin index.

    Args:
        rad (float): angle in radians (from UDP data, 0° = forward, increasing CCW)

    Returns:
        b (int): bin index in range [0, BINS-1]
        a (float): normalized angle in radians
    """
    # Wrap angle into [-pi, +pi)
    a = (rad + math.pi) % (2*math.pi) - math.pi
    # Compute bin index
    b = int((a - ANGLE_MIN) / ANGLE_STEP)
    # Clamp bin index into valid range
    return max(0, min(BINS-1, b)), a


class UdpToScan(Node):
    """
    A ROS2 node that listens for LiDAR UDP packets and publishes LaserScan.
    """

    def __init__(self):
        super().__init__("udp_to_scan")

        # Publisher: publish LaserScan messages on "/scan"
        self.pub = self.create_publisher(LaserScan, "/scan", 10)

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", PORT))   # Listen on all interfaces
        self.sock.setblocking(True)         # Blocking mode (safe for separate thread)

        # Buffers for accumulating one revolution of data
        self.bin_ranges = [float("inf")] * BINS   # default: no return = inf
        self.bin_intens = [0.0] * BINS            # default: 0 intensity
        self.prev_angle = None
        self.last_frame_time = time.time()

        # Start UDP receive thread (runs in background)
        t = threading.Thread(target=self._rx_loop, daemon=True)
        t.start()

        self.get_logger().info(f"Listening UDP :{PORT} → /scan (full circle mode)")

    def _rx_loop(self):
        """
        Background thread: receives UDP packets, parses them line by line,
        and fills angular bins with range & intensity data.
        """
        while rclpy.ok():
            # Receive raw UDP data
            data, _ = self.sock.recvfrom(65536)  # up to 64 KB
            for line in data.decode(errors="ignore").strip().splitlines():
                if not line:
                    continue
                try:
                    # Expected format: "P,idx,angle_deg,dist_mm,intensity"
                    if line.startswith("P,"):
                        parts = line.split(",")
                        ang   = float(parts[2]) * math.pi/180.0   # convert deg → rad
                        dist  = float(parts[3]) / 1000.0          # convert mm → m
                        inten = float(parts[4])                   # raw intensity

                        # Map angle into bin index
                        b, a_norm = ang_to_bin(ang)

                        # Store point into bin if within valid range
                        if RANGE_MIN <= dist <= RANGE_MAX:
                            # Strategy: keep the closest distance for each bin
                            if dist < self.bin_ranges[b]:
                                self.bin_ranges[b] = dist
                                self.bin_intens[b] = inten

                        # Detect wrap-around (angle decreasing significantly)
                        if self.prev_angle is not None:
                            if a_norm < self.prev_angle - (20.0*math.pi/180.0):
                                # If angle jumped backwards more than ~20°
                                # → assume a new revolution started
                                self._publish_full_circle()
                                self._clear_bins()
                        self.prev_angle = a_norm

                except Exception as e:
                    # Robustness: catch parse errors and continue
                    self.get_logger().warn(f"parse error: {e} line={line[:80]}")

            # Timeout safeguard:
            # If no full circle detected for >0.25s, force publish partial scan
            if time.time() - self.last_frame_time > 0.25:
                self._publish_full_circle()
                self._clear_bins()

    def _clear_bins(self):
        """
        Reset bin buffers for the next revolution.
        """
        self.bin_ranges = [float("inf")] * BINS
        self.bin_intens = [0.0] * BINS
        self.last_frame_time = time.time()

    def _publish_full_circle(self):
        """
        Publish a LaserScan message containing all bins from one revolution.
        """
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = FRAME_ID

        # Define angular metadata
        scan.angle_min = ANGLE_MIN
        scan.angle_max = ANGLE_MAX - ANGLE_STEP   # exclude last step to fit BINS
        scan.angle_increment = ANGLE_STEP

        # Timing metadata
        scan.scan_time = 0.1                      # assume 10 Hz LiDAR
        scan.time_increment = scan.scan_time / BINS

        # Range and intensity arrays
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
        scan.ranges = self.bin_ranges
        scan.intensities = self.bin_intens

        # Publish LaserScan
        self.pub.publish(scan)


def main():
    """
    Main entry point:
    - Initialize ROS2
    - Create node
    - Spin until shutdown
    """
    rclpy.init()
    n = UdpToScan()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
