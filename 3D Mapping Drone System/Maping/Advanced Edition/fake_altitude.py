#!/usr/bin/env python3
# Author: Ye Li 
# Description:
#   Publish a synthetic altitude waveform on /altitude (std_msgs/Float32).
#   Useful for simulation/testing of the altitude_to_tf pipeline.

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class FakeAlt(Node):
    def __init__(self):
        super().__init__('fake_altitude')

        # Publisher
        self.pub = self.create_publisher(Float32, '/altitude', 10)

        # Parameters (center, amplitude, frequency)
        self.declare_parameter('z_mid', 0.75)   # mean altitude (m)
        self.declare_parameter('z_amp', 0.45)   # amplitude (m)
        self.declare_parameter('hz',  5.0)      # publish frequency (Hz)
        self.declare_parameter('omega', 0.5)    # angular frequency (rad/s)

        self._t = 0.0
        period = 1.0 / float(self.get_parameter('hz').value)
        self.timer = self.create_timer(period, self.tick)

    def tick(self):
        """Compute z(t) = z_mid + z_amp * sin(omega * t) and publish."""
        z_mid = float(self.get_parameter('z_mid').value)
        z_amp = float(self.get_parameter('z_amp').value)
        omega = float(self.get_parameter('omega').value)

        z = z_mid + z_amp * math.sin(self._t * omega)
        msg = Float32()
        msg.data = float(z)
        self.pub.publish(msg)

        # Advance time by the publish period
        hz = float(self.get_parameter('hz').value)
        self._t += 1.0 / max(hz, 1e-6)


def main():
    rclpy.init()
    rclpy.spin(FakeAlt())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
