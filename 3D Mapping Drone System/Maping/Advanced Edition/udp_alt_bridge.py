#!/usr/bin/env python3
# Author: Ye Li 
# Description:
#   Receive altitude messages via UDP in the form "ALT:<value>",
#   optionally filter with EMA, clamp to limits, and publish to a ROS topic.

import socket
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Regex to capture numeric value in "ALT:<number>"
PATTERN = re.compile(rb'ALT\s*:\s*([+-]?[0-9]*\.?[0-9]+)')


class UdpAltBridge(Node):
    def __init__(self):
        super().__init__('udp_alt_bridge')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', 8888)         # UDP port to listen
        self.declare_parameter('topic', '/altitude') # ROS topic to publish
        self.declare_parameter('clamp_min', 0.0)
        self.declare_parameter('clamp_max', 10.0)
        self.declare_parameter('use_ema', True)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('log_every_n', 20)

        self.port = int(self.get_parameter('port').value)
        self.topic = str(self.get_parameter('topic').value)
        self.clamp_min = float(self.get_parameter('clamp_min').value)
        self.clamp_max = float(self.get_parameter('clamp_max').value)
        self.use_ema = bool(self.get_parameter('use_ema').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.log_every_n = int(self.get_parameter('log_every_n').value)

        # ---------------- ROS I/O ----------------
        self.pub = self.create_publisher(Float32, self.topic, 10)

        # UDP socket (non-blocking)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        self.get_logger().info(f'Listening UDP :{self.port} → publish {self.topic}')
        self.timer = self.create_timer(0.0, self._poll_socket)

        # Internal state for EMA and logging
        self._ema = None
        self._count = 0

    def _poll_socket(self):
        """Poll the UDP socket; parse and publish when a valid line is seen."""
        try:
            data, addr = self.sock.recvfrom(1024)
        except BlockingIOError:
            return
        except Exception as e:
            self.get_logger().warn(f'UDP recv error: {e}')
            return

        m = PATTERN.search(data)
        if not m:
            return

        try:
            z = float(m.group(1))
        except Exception:
            return

        # Clamp
        z = max(self.clamp_min, min(self.clamp_max, z))

        # EMA filter (optional)
        if self.use_ema:
            self._ema = z if self._ema is None else self.ema_alpha * z + (1 - self.ema_alpha) * self._ema
            z_pub = self._ema
        else:
            z_pub = z

        msg = Float32()
        msg.data = float(z_pub)
        self.pub.publish(msg)

        self._count += 1
        if self._count % max(1, self.log_every_n) == 0:
            self.get_logger().info(f'ALT={z:.3f} m (pub={z_pub:.3f}) from {addr[0]}')


def main():
    rclpy.init()
    node = UdpAltBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
