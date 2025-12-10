#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import numpy as np

class SimulatedContactDetection(Node):
    def __init__(self):
        super().__init__('simulated_contact_detection')

        self.loop_marker_points = []       # Greifer-Loop
        self.hotwire_marker_points = []    # Hotwire
        self.contact_distance_threshold = 0.01  # 1 cm

        # Marker-Subscriber
        self.create_subscription(Marker, '/visualization_marker', self.marker_callback, 10)

        # Publisher für Kontaktstatus
        self.contact_pub = self.create_publisher(Int32, '/contact_detection', 10)

        # Timer für periodische Prüfung
        self.timer = self.create_timer(1/1000, self.calculate_contact)

        self.get_logger().info("SimulatedContactDetection node initialized")

    def marker_callback(self, msg: Marker):
        if msg.ns == "gripper_loop":
            self.loop_marker_points = [(p.x, p.y, p.z) for p in msg.points]
        elif msg.ns == "hot_wire" or msg.ns == "hotwire_marker":
            self.hotwire_marker_points = [(p.x, p.y, p.z) for p in msg.points]

    def point_to_segment_distance(self, p, a, b):
        """Minimal distance between point p and segment a-b."""
        p = np.array(p)
        a = np.array(a)
        b = np.array(b)
        ab = b - a
        ap = p - a
        t = np.clip(np.dot(ap, ab) / np.dot(ab, ab), 0.0, 1.0)
        closest = a + t * ab
        return np.linalg.norm(p - closest)

    def segment_to_segment_distance(self, a1, a2, b1, b2):
        """Approximate minimal distance between two segments by checking endpoints."""
        return min(
            self.point_to_segment_distance(a1, b1, b2),
            self.point_to_segment_distance(a2, b1, b2),
            self.point_to_segment_distance(b1, a1, a2),
            self.point_to_segment_distance(b2, a1, a2)
        )

    def calculate_contact(self):
        if len(self.loop_marker_points) < 2 or len(self.hotwire_marker_points) < 2:
            self.get_logger().info("Not enough points to calculate distance.")
            return  # nicht genügend Punkte

        min_dist = float('inf')

        # Loop über alle Segmente von Loop und Hotwire
        for i in range(len(self.loop_marker_points)-1):
            a1 = self.loop_marker_points[i]
            a2 = self.loop_marker_points[i+1]
            for j in range(len(self.hotwire_marker_points)-1):
                b1 = self.hotwire_marker_points[j]
                b2 = self.hotwire_marker_points[j+1]
                dist = self.segment_to_segment_distance(a1, a2, b1, b2)
                if dist < min_dist:
                    min_dist = dist

        self.get_logger().info(f"Min distance between loop and hotwire: {min_dist:.6f} m")

        # Kontaktstatus
        contact = 1 if min_dist < self.contact_distance_threshold else 0
        msg = Int32()
        msg.data = contact
        self.contact_pub.publish(msg)

        if contact:
            self.get_logger().info(f"Contact detected! Min distance: {min_dist:.6f} m")


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedContactDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
