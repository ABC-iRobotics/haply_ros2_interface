#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String


class HapticLoop(Node):
    """Visualizes a loop attached to the PSM1 gripper in WORLD frame"""

    def __init__(self):
        super().__init__("greifer_loop_cylinders")

        # Force PSM1 to publish in world frame
        self.base_frame_pub = self.create_publisher(String, "/PSM1/set_base_frame", 10)
        msg = String()
        msg.data = "world"
        self.base_frame_pub.publish(msg)
        self.get_logger().info("Set PSM1 base frame to 'world'")

        # Publisher for markers and loop center
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", 10)
        self.loop_center_pub = self.create_publisher(PoseStamped, "loop_center", 10)

        # Subscriber – now already in world frame
        self.create_subscription(PoseStamped, "/PSM1/local/measured_cp",
                                 self.gripper_callback, 10)

        # Loop parameters
        self.loop_radius = 0.005
        self.loop_segments = 10
        self.loop_thickness = 0.002
        self.loop_x_offset = 0.00
        self.loop_y_offset = 0.01 + self.loop_radius
        self.loop_z_offset = 0.00

        # Storage for last pose
        self.gripper_pose_world = None

        # Timer 50 Hz
        self.timer = self.create_timer(1/10, self.publish_loop_marker)

        self.get_logger().info("Haptic Loop Visualization initialized.")


    # Incoming pose is already in world frame (because we forced the base frame)
    def gripper_callback(self, msg: PoseStamped):
        self.gripper_pose_world = msg
        self.get_logger().info("Received new gripper pose: {:.3f}, {:.3f}, {:.3f}".format(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ))


    # Quaternion vector rotation helper
    def rotate_vector_by_quaternion(self, qx, qy, qz, qw, vx, vy, vz):
        cx = qy * vz - qz * vy
        cy = qz * vx - qx * vz
        cz = qx * vy - qy * vx

        tx = 2.0 * cx
        ty = 2.0 * cy
        tz = 2.0 * cz

        sx = qy * tz - qz * ty
        sy = qz * tx - qx * tz
        sz = qx * ty - qy * tx

        rx = vx + qw * tx + sx
        ry = vy + qw * ty + sy
        rz = vz + qw * tz + sz
        return rx, ry, rz


    def multiply_quaternions(self, a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b

        w = aw*bw - ax*bx - ay*by - az*bz
        x = aw*bx + ax*bw + ay*bz - az*by
        y = aw*by - ax*bz + ay*bw + az*bx
        z = aw*bz + ax*by - ay*bx + az*bw
        return x, y, z, w


    def publish_loop_marker(self):
        if self.gripper_pose_world is None:
            return

        pose = self.gripper_pose_world

        px = pose.pose.position.x
        py = pose.pose.position.y
        pz = pose.pose.position.z
        qx = pose.pose.orientation.x
        qy = pose.pose.orientation.y
        qz = pose.pose.orientation.z
        qw = pose.pose.orientation.w

        # Apply additional rotation of 90° around X if needed
        loop_angle = 90.0
        if loop_angle != 0.0:
            half = math.radians(loop_angle / 2.0)
            rot = (math.sin(half), 0.0, 0.0, math.cos(half))
            qx, qy, qz, qw = self.multiply_quaternions((qx, qy, qz, qw), rot)

        # Apply offsets in gripper frame
        ox, oy, oz = self.rotate_vector_by_quaternion(qx, qy, qz, qw,
                                                      self.loop_x_offset,
                                                      self.loop_y_offset,
                                                      self.loop_z_offset)
        px += ox
        py += oy
        pz += oz

               # --------------------------
        # Build marker (world frame)
        # --------------------------
        marker = Marker()
        marker.header.frame_id = "world"  # <-- jetzt world
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gripper_loop"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = self.loop_thickness
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Zusätzliche Welt-Verschiebung
        world_offset_x = -0.183
        world_offset_y = 1.36
        world_offset_z = 1.08

        # Create circular loop
        for i in range(self.loop_segments + 1):
            angle = 2 * math.pi * i / self.loop_segments
            lx = self.loop_radius * math.cos(angle)
            ly = self.loop_radius * math.sin(angle)
            lz = 0.0

            wx, wy, wz = self.rotate_vector_by_quaternion(qx, qy, qz, qw, lx, ly, lz)

            p = Point()
            p.x = px + wx + world_offset_x  # <-- hier Offset addiert
            p.y = py + wy + world_offset_y
            p.z = pz + wz + world_offset_z
            marker.points.append(p)

        self.marker_pub.publish(marker)

        # Publish center pose
        center = PoseStamped()
        center.header = marker.header
        center.pose.position.x = px + world_offset_x
        center.pose.position.y = py + world_offset_y
        center.pose.position.z = pz + world_offset_z
        center.pose.orientation = pose.pose.orientation
        self.loop_center_pub.publish(center)




def main(args=None):
    rclpy.init(args=args)
    node = HapticLoop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
