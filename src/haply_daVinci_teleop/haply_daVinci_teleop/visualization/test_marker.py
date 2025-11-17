#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class SimpleMarker(Node):
    def __init__(self):
        super().__init__("simple_marker_node")
        self.pub = self.create_publisher(Marker, "visualization_marker", 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "test_marker"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = -0.153
        marker.pose.position.y = 1.34
        marker.pose.position.z = 0.957
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0

        # Größe des Markers (1 cm Kugel)
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMarker()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
