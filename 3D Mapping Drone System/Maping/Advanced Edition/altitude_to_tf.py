#!/usr/bin/env python3
# Author: Ye Li 
# Description:
#   Convert /altitude (std_msgs/Float32, meters) to a dynamic TF:
#     base_link -> base_link_z (z translation only)
#   and publish a static TF:
#     base_link_z -> laser (mounting extrinsics)
#
#   Typical TF chain in this project:
#     odom -> base_link -> base_link_z -> laser

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class AltitudeTFBroadcaster(Node):
    def __init__(self):
        super().__init__('altitude_tf_broadcaster')

        # ---------------- Parameters ----------------
        # Parent/child for dynamic TF (z-only motion from altitude)
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame',  'base_link_z')

        # Laser frame name for static mounting transform
        self.declare_parameter('laser_frame',  'laser')

        # Static extrinsics (base_link_z -> laser)
        self.declare_parameter('laser_xyz', [0.0, 0.0, 0.0])      # meters
        self.declare_parameter('laser_rpy_deg', [0.0, 0.0, 0.0])  # degrees

        # Altitude signal conditioning
        self.declare_parameter('use_ema', True)
        self.declare_parameter('ema_alpha', 0.25)
        self.declare_parameter('clamp_min', 0.0)
        self.declare_parameter('clamp_max', 5.0)

        # Resolve parameters
        self.parent = self.get_parameter('parent_frame').value
        self.child  = self.get_parameter('child_frame').value
        self.laser  = self.get_parameter('laser_frame').value
        self.use_ema   = bool(self.get_parameter('use_ema').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.clamp_min = float(self.get_parameter('clamp_min').value)
        self.clamp_max = float(self.get_parameter('clamp_max').value)

        # TF broadcasters
        self.br  = TransformBroadcaster(self)
        self.sbr = StaticTransformBroadcaster(self)

        # Internal state
        self._ema = None
        self._z = 0.0

        # ---------------- ROS I/O ----------------
        # Subscribe to altitude and periodically broadcast TF
        self.sub = self.create_subscription(Float32, '/altitude', self.cb_alt, 10)
        self.timer = self.create_timer(0.05, self.broadcast_tf)   # 20 Hz TF

        # Publish static base_link_z -> laser once at startup
        xyz = [float(v) for v in self.get_parameter('laser_xyz').value]
        rpy_deg = [float(v) for v in self.get_parameter('laser_rpy_deg').value]
        self._send_static_laser_tf(xyz, rpy_deg)

        self.get_logger().info(
            f"Altitude TF active: {self.parent} → {self.child} (dynamic), "
            f"{self.child} → {self.laser} (static)"
        )

    def _send_static_laser_tf(self, xyz, rpy_deg):
        """Publish the fixed extrinsic transform base_link_z -> laser."""
        rx, ry, rz = [math.radians(v) for v in rpy_deg]

        # RPY (intrinsic XYZ) -> quaternion
        cr, sr = math.cos(rx/2), math.sin(rx/2)
        cp, sp = math.cos(ry/2), math.sin(ry/2)
        cy, sy = math.cos(rz/2), math.sin(rz/2)
        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.child
        t.child_frame_id  = self.laser
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = (qx, qy, qz, qw)
        self.sbr.sendTransform(t)

    def cb_alt(self, msg: Float32):
        """Receive altitude in meters, clamp and optionally filter with EMA."""
        z = max(self.clamp_min, min(self.clamp_max, float(msg.data)))
        if self.use_ema:
            self._ema = z if self._ema is None else (self.ema_alpha * z + (1 - self.ema_alpha) * self._ema)
            self._z = float(self._ema)
        else:
            self._z = z

    def broadcast_tf(self):
        """Publish dynamic TF: base_link -> base_link_z with z translation only."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id  = self.child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self._z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
