#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math


class HotWireVisualization(Node):
    """Visualizes a straight wire (cylinder) for the Hot Wire experiment"""

    def __init__(self):
        super().__init__("hot_wire_visualization")

        # Publisher
        self.marker_publisher = self.create_publisher(Marker, "hotwire_marker", 10)

        # Timer for continuous visualization (10 Hz)
        self.marker_timer = self.create_timer(1 / 10, self.publish_wire_marker)

        self.get_logger().info(
            f"Hot Wire Visualization started"
        )

        # Define start and end points of the wire in cartesian coordinates (unit: meters)
        self.wire_start = [-0.05, 0.0, -0.1]
        self.wire_end = [0.05, 0.05, -0.1]

    def publish_wire_marker(self):
        """Publishes a line marker representing the wire"""

        # compute start / end scaled
        start_x = self.wire_start[0] 
        start_y = self.wire_start[1] 
        start_z = self.wire_start[2] 
        end_x = self.wire_end[0] 
        end_y = self.wire_end[1] 
        end_z = self.wire_end[2] 

        # direction and length
        distance_x = end_x - start_x
        distance_y = end_y - start_y
        distance_z = end_z - start_z
        wirelength = math.sqrt(distance_x*distance_x + distance_y*distance_y + distance_z*distance_z)
        if wirelength == 0.0:
            # avoid division by zero
            wirelength = 0.001
            direction = (0.0, 0.0, 1.0)
        else:
            direction = (distance_x/wirelength, distance_y/wirelength, distance_z/wirelength)

        # compute midpoints
        midpoint_x = (start_x + end_x) / 2.0
        midpoint_y = (start_y + end_y) / 2.0
        midpoint_z = (start_z + end_z) / 2.0

        # compute quaternion rotating +Z to direction
        def quat_from_two_vectors(v_from, v_to):
            # returns (x,y,z,w)
            ux, uy, uz = v_from
            vx, vy, vz = v_to
            # cross and dot
            cx = uy * vz - uz * vy
            cy = uz * vx - ux * vz
            cz = ux * vy - uy * vx
            dot = ux*vx + uy*vy + uz*vz
            norm_c = math.sqrt(cx*cx + cy*cy + cz*cz)
            if norm_c < 1e-6:
                # parallel or anti-parallel
                if dot > 0.9999:
                    return (0.0, 0.0, 0.0, 1.0)
                else:
                    # 180 deg rotation about X (arbitrary orthogonal axis)
                    return (1.0, 0.0, 0.0, 0.0)
            axis = (cx / norm_c, cy / norm_c, cz / norm_c)
            angle = math.atan2(norm_c, dot)
            s = math.sin(angle/2.0)
            qx = axis[0] * s
            qy = axis[1] * s
            qz = axis[2] * s
            qw = math.cos(angle/2.0)
            return (qx, qy, qz, qw)

        qx, qy, qz, qw = quat_from_two_vectors((0.0, 0.0, 1.0), direction)

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hot_wire"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # pose = midpoint + orientation aligning cylinder's local Z with direction
        marker.pose.position.x = midpoint_x
        marker.pose.position.y = midpoint_y
        marker.pose.position.z = midpoint_z
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        # scale: x/y = diameter, z = length
        marker.scale.x = 0.002  # diameter (m)
        marker.scale.y = 0.002
        marker.scale.z = wirelength

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = HotWireVisualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
