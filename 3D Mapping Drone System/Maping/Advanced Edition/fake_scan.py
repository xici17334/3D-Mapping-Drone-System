#!/usr/bin/env python3
# Author: Ye Li 
# Description:
#   Publish a synthetic LaserScan on /scan for testing.
#   Simulates a semicircle scan with mild noise and slow range oscillation.

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScan(Node):
    def __init__(self):
        super().__init__('fake_scan_pub')

        # Publisher
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # Scan geometry: [-90°, +90°] with 1° step
        self.angle_min = -math.pi / 2
        self.angle_max =  math.pi / 2
        self.angle_increment = math.radians(1.0)
        self.range_min, self.range_max = 0.05, 10.0
        self.beams = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1

        # Publish rate
        self.scan_hz = 10.0
        self.timer = self.create_timer(1.0 / self.scan_hz, self.tick)
        self.t = 0.0

        self.get_logger().info(f'fake_scan_pub started: beams={self.beams}, {self.scan_hz} Hz')

    def tick(self):
        """Generate a noisy ring-like scan and publish it."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'  # must match the TF static laser frame
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.scan_hz
        msg.range_min, msg.range_max = self.range_min, self.range_max

        # Base radius slowly oscillates to emulate motion or environment change
        base_r = 2.0 + 0.2 * math.sin(self.t * 0.5)

        rng = []
        for _ in range(self.beams):
            r = base_r + 0.03 * random.uniform(-1.0, 1.0)
            r = min(max(r, self.range_min + 0.01), self.range_max - 0.01)
            rng.append(float(r))
        msg.ranges = rng
        msg.intensities = [0.0] * self.beams

        self.pub.publish(msg)
        self.t += 1.0 / self.scan_hz


def main():
    rclpy.init()
    rclpy.spin(FakeScan())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
