#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Node: UDP → /altitude (std_msgs/Float32)

Function:
- Listen on a given UDP port for altitude messages in the format "ALT:1.234\n"
- Parse the numeric altitude value (in meters)
- Optionally filter the value with Exponential Moving Average (EMA)
- Publish the result to a ROS2 topic (default: `/altitude`)

Adjustable ROS2 Parameters:
- port (int, default 8888): UDP listening port
- topic (string, default "/altitude"): Name of the topic to publish altitude values
- clamp_min (float, default 0.0): Minimum altitude; values below this will be clamped
- clamp_max (float, default 10.0): Maximum altitude; values above this will be clamped
- use_ema (bool, default True): Whether to apply EMA filtering
- ema_alpha (float, default 0.3): Smoothing factor for EMA; higher means faster response
- log_every_n (int, default 20): Print log message every N received packets (prevents spam)

Usage examples:
    # Run directly (no need to build into a ROS package)
    python3 udp_alt_bridge.py

    # Run inside a ROS2 package with custom UDP port
    ros2 run <your_pkg> udp_alt_bridge --ros-args -p port:=9999

Author: Li Ye  
"""

import socket
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Regex pattern to match altitude data in the form: ALT:1.234
# - "ALT" keyword (case-sensitive)
# - Optional spaces around the colon
# - Signed or unsigned float number (integer or decimal)
PATTERN = re.compile(rb'ALT\s*:\s*([+-]?[0-9]*\.?[0-9]+)')


class UdpAltBridge(Node):
    """
    A ROS2 node that receives altitude data via UDP and publishes it as std_msgs/Float32.
    """

    def __init__(self):
        super().__init__('udp_alt_bridge')

        # Declare ROS2 parameters (with default values)
        self.declare_parameter('port', 8888)
        self.declare_parameter('topic', '/altitude')
        self.declare_parameter('clamp_min', 0.0)
        self.declare_parameter('clamp_max', 10.0)
        self.declare_parameter('use_ema', True)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('log_every_n', 20)

        # Retrieve parameter values
        self.port = int(self.get_parameter('port').value)
        self.topic = str(self.get_parameter('topic').value)
        self.clamp_min = float(self.get_parameter('clamp_min').value)
        self.clamp_max = float(self.get_parameter('clamp_max').value)
        self.use_ema = bool(self.get_parameter('use_ema').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.log_every_n = int(self.get_parameter('log_every_n').value)

        # Create ROS2 publisher for Float32 messages
        self.pub = self.create_publisher(Float32, self.topic, 10)

        # Initialize UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))  # listen on all interfaces
        self.sock.setblocking(False)  # non-blocking mode

        self.get_logger().info(
            f'Listening UDP on 0.0.0.0:{self.port} → publishing to {self.topic}'
        )

        # Create a zero-period timer that continuously polls the UDP socket
        # (effectively makes _poll_socket run as fast as possible)
        self.timer = self.create_timer(0.0, self._poll_socket)

        # Internal state for EMA filtering
        self._ema = None
        self._count = 0  # received packet counter (for logging)

    def _poll_socket(self):
        """
        Callback executed repeatedly by the ROS2 timer.
        It polls the UDP socket for new data, processes it, and publishes altitude.
        """
        try:
            data, addr = self.sock.recvfrom(1024)  # receive up to 1024 bytes
        except BlockingIOError:
            # No data available right now → simply return
            return
        except Exception as e:
            # Handle unexpected socket errors gracefully
            self.get_logger().warn(f'UDP recv error: {e}')
            return

        # Match incoming data against the regex pattern
        m = PATTERN.search(data)
        if not m:
            # If format does not match "ALT:xxx", ignore this packet
            return

        try:
            z = float(m.group(1))  # Extract altitude value
        except Exception:
            return

        # Clamp altitude to valid range [clamp_min, clamp_max]
        if z < self.clamp_min:
            z = self.clamp_min
        if z > self.clamp_max:
            z = self.clamp_max

        # Apply Exponential Moving Average (EMA) filtering
        if self.use_ema:
            if self._ema is None:
                # First value → initialize EMA
                self._ema = z
            else:
                a = self.ema_alpha
                self._ema = a * z + (1.0 - a) * self._ema
            z_pub = self._ema
        else:
            z_pub = z

        # Publish altitude as std_msgs/Float32
        msg = Float32()
        msg.data = float(z_pub)
        self.pub.publish(msg)

        # Log altitude periodically (every N packets)
        self._count += 1
        if self._count % max(1, self.log_every_n) == 0:
            self.get_logger().info(
                f'ALT={z:.3f} m (published={z_pub:.3f}) from {addr[0]}'
            )


def main():
    """
    Main entry point.
    Initializes ROS2, creates the node, and spins until shutdown.
    """
    rclpy.init()
    node = UdpAltBridge()
    try:
        rclpy.spin(node)
    finally:
        # Clean shutdown: destroy node and shutdown ROS2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
