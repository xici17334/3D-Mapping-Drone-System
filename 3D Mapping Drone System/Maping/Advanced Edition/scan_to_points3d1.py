#!/usr/bin/env python3
# Author: Ye Li 
# Description:
#   Transform a 2D LaserScan (/scan) into 3D points (/points_3d) expressed
#   in the world frame (e.g., 'odom') using TF world->laser at the scan stamp.

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time, Duration


def quat_to_R(qx, qy, qz, qw):
    """Quaternion -> 3x3 rotation matrix."""
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return [
        [1-2*(yy+zz),   2*(xy-wz),     2*(xz+wy)],
        [2*(xy+wz),     1-2*(xx+zz),   2*(yz-wx)],
        [2*(xz-wy),     2*(yz+wx),     1-2*(xx+yy)],
    ]


class ScanToPoints3D(Node):
    def __init__(self):
        super().__init__('scan_to_points3d')

        # Frames
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('laser_frame', 'laser')
        self.world = self.get_parameter('world_frame').value
        self.laser = self.get_parameter('laser_frame').value

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS I/O
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, '/points_3d', 10)

        self.get_logger().info(f"3D converter: world={self.world}, laser={self.laser}")

    def scan_cb(self, scan: LaserScan):
        """Lookup TF at the scan timestamp and transform ranges into 3D points."""
        try:
            # Try a short wait for TF availability to avoid log flooding
            ok = self.tf_buffer.can_transform(self.world, self.laser, Time(), timeout=Duration(seconds=0.05))
            if not ok:
                return
            tf = self.tf_buffer.lookup_transform(self.world, self.laser, Time())
        except Exception:
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)

        pts = []
        a = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and (scan.range_min <= r <= scan.range_max):
                # 2D laser plane (z=0 in laser frame)
                xl = r * math.cos(a)
                yl = r * math.sin(a)
                zl = 0.0
                # world = R * laser + t
                xw = R[0][0]*xl + R[0][1]*yl + R[0][2]*zl + tx
                yw = R[1][0]*xl + R[1][1]*yl + R[1][2]*zl + ty
                zw = R[2][0]*xl + R[2][1]*yl + R[2][2]*zl + tz
                pts.append((xw, yw, zw))
            a += scan.angle_increment

        if not pts:
            return

        header = scan.header
        header.frame_id = self.world  # output in world frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(header, fields, pts)
        self.pub.publish(cloud)


def main():
    rclpy.init()
    rclpy.spin(ScanToPoints3D())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
