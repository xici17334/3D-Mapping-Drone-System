#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Ye Li
# File: scan_to_points3d1.py
# Description:
#   A ROS 2 node that subscribes to 2D LaserScan (/scan), retrieves altitude (z)
#   from TF, and converts the scan into 3D point cloud data (PointCloud2).
#   This enables building 3D maps by stacking 2D scans with height information.
# -----------------------------------------------------------------------------



import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data   # Key: use sensor QoS for real-time data

class ScanToPoints3D(Node):
    def __init__(self):
        # Initialize the node named scan_to_points3d
        super().__init__('scan_to_points3d')
        self.world_frame = 'world'      # Global/world frame
        self.base_frame  = 'base_link'  # Robot body/base frame

        # TF buffer and listener, used to query base_link pose in world
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to /scan (LaserScan) with sensor QoS for low-latency data
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            qos_profile_sensor_data
        )

        # Publisher for 3D point cloud on /points_3d
        self.pub = self.create_publisher(PointCloud2, '/points_3d', 10)

        self.get_logger().info('scan_to_points3d node started, listening /scan')

    def scan_cb(self, scan: LaserScan):
        """Callback: Convert LaserScan to PointCloud2"""
        # 1) Query TF to get base_link’s z height in world
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, rclpy.time.Time())
            tz = float(tf.transform.translation.z)  # current z height
        except Exception as e:
            # Skip if no TF available yet
            self.get_logger().warn(f'No TF yet: {e}')
            return

        # 2) Convert LaserScan data into (x, y, z) point list
        pts = []
        ang = scan.angle_min
        for r in scan.ranges:
            # Only keep valid values within sensor range
            if math.isfinite(r) and (scan.range_min < r < scan.range_max):
                x = r * math.cos(ang)   # compute x coordinate
                y = r * math.sin(ang)   # compute y coordinate
                pts.append((x, y, tz))  # add z height to form 3D point
            ang += scan.angle_increment

        if not pts:
            return  # Exit if no valid points

        # 3) Build PointCloud2 message
        header = scan.header
        header.frame_id = self.world_frame  # Output in world frame for TF alignment

        # Define fields (x, y, z as float32)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Create point cloud message
        cloud = point_cloud2.create_cloud(header, fields, pts)

        # Publish the point cloud
        self.pub.publish(cloud)

def main():
    rclpy.init()
    node = ScanToPoints3D()
    rclpy.spin(node)   # Keep spinning until shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
