#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Author: Ye Li
# File: altitude_to_tf1.py
# Description:
#   A ROS 2 node that subscribes to altitude values (/altitude) and broadcasts
#   them as a TF transform (world -> base_link). The altitude is mapped to the
#   z-axis translation, allowing integration with other TF-based systems.
# -----------------------------------------------------------------------------


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class AltitudeTFBroadcaster(Node):
    def __init__(self):
        # Initialize the node with name "altitude_tf_broadcaster"
        super().__init__('altitude_tf_broadcaster')
        self.get_logger().info('Node constructed')

        # Declare configurable parameters: parent and child frames
        # Default: world -> base_link
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'base_link')
        self.parent = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child  = self.get_parameter('child_frame').get_parameter_value().string_value
        self.get_logger().info(f'Frames: {self.parent} -> {self.child}')

        # Create a TF broadcaster
        self.br = TransformBroadcaster(self)

        # Current altitude value (meters)
        self.z = 0.0

        # Subscribe to the /altitude topic (std_msgs/Float32)
        # Incoming data will update self.z
        self.sub = self.create_subscription(Float32, '/altitude', self.cb_alt, 10)
        self.get_logger().info('Subscription created on /altitude')

        # Timer to publish TF every 0.5 seconds (2 Hz)
        self.timer = self.create_timer(0.5, self.publish_tf)
        self.get_logger().info('Timer started (0.5s)')

    def cb_alt(self, msg: Float32):
        """Callback: update altitude from /altitude topic"""
        self.z = float(msg.data)
        self.get_logger().info(f'Recv altitude: {self.z:.3f} m')

    def publish_tf(self):
        """Publish a TF transform: parent_frame -> child_frame with altitude as z"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()  # timestamp
        t.header.frame_id = self.parent                   # parent frame
        t.child_frame_id  = self.child                    # child frame

        # Translation: x, y fixed at 0, z comes from altitude
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.z

        # Rotation: identity quaternion (no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the transform
        self.br.sendTransform(t)
        self.get_logger().info(f'Broadcast TF z={self.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeTFBroadcaster()
    node.get_logger().info('Spinning ...')
    rclpy.spin(node)  # Keep the node alive
    node.get_logger().info('Shutdown')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
