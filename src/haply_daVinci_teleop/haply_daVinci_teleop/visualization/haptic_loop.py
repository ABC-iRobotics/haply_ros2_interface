#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import math
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs

class GreiferLoopCylinders(Node):
    """Visualizes a loop attached to the PSM1 gripper using a single Marker"""

    def __init__(self):
        super().__init__("greifer_loop_cylinders")

        # Publisher für Marker
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", 10)

        # Subscriber auf Greifer-Pose
        # store full PoseStamped so we can reuse header.frame_id
        self.create_subscription(PoseStamped, "/PSM1/local/measured_cp", self.gripper_callback, 10)

        # Parameter Loop
        self.loop_radius = 0.01         # 1 cm Radius
        self.loop_segments = 24         # number segments to approximate the circle
        self.loop_thickness = 0.002     # cylinder diameter 2 mm
        self.loop_x_offset = 0.00       # offset along gripper X axis
        self.loop_y_offset = 0.02       # offset along gripper Y axis
        self.loop_z_offset = 0.00       # offset along gripper Z axis

        # Aktuelle Greifer-Pose (PoseStamped)
        self.gripper_pose_stamped = None
        self.gripper_pose_world = None
        # TF buffer/listener zum Transformieren nach 'world'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer für kontinuierliches Visualisieren
        self.timer = self.create_timer(1/10, self.publish_loop_marker)

        self.get_logger().info("Greifer Loop (Cylinder) Visualization initialized.")

    def gripper_callback(self, msg: PoseStamped):
        # keep full pose (including header.frame_id)
        self.gripper_pose_stamped = msg
        # try to transform into 'world' frame immediately (non-fatal)
        try:
            if msg.header.frame_id != "world":
                # lookup transform world <- msg.header.frame_id
                t = self.tf_buffer.lookup_transform("world", msg.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.5))
                transformed = tf2_geometry_msgs.do_transform_pose(msg, t)
                self.gripper_pose_world = transformed
            else:
                self.gripper_pose_world = msg
        except Exception as e:
            # TF not available yet — we keep gripper_pose_world None (will fallback)
            self.get_logger().debug(f"TF transform to 'world' failed: {e}")
            self.gripper_pose_world = None

    def _quat_rotate_vector(self, qx, qy, qz, qw, vx, vy, vz):
        """
        Rotate vector v by quaternion q (qx,qy,qz,qw).
        Uses optimized formula: t = 2 * cross(q_xyz, v); v' = v + q_w * t + cross(q_xyz, t)
        """
        # cross(q_xyz, v)
        cx = qy * vz - qz * vy
        cy = qz * vx - qx * vz
        cz = qx * vy - qy * vx
        # t = 2 * cross
        tx = 2.0 * cx
        ty = 2.0 * cy
        tz = 2.0 * cz
        # v' = v + q_w * t + cross(q_xyz, t)
        # cross(q_xyz, t)
        c2x = qy * tz - qz * ty
        c2y = qz * tx - qx * tz
        c2z = qx * ty - qy * tx
        vx_p = vx + qw * tx + c2x
        vy_p = vy + qw * ty + c2y
        vz_p = vz + qw * tz + c2z
        return vx_p, vy_p, vz_p

    def _quat_mul(self, a, b):
        """Quaternion multiplication a * b. a,b as (x,y,z,w)."""
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        w = aw*bw - ax*bx - ay*by - az*bz
        x = aw*bx + ax*bw + ay*bz - az*by
        y = aw*by - ax*bz + ay*bw + az*bx
        z = aw*bz + ax*by - ay*bx + az*bw
        return x, y, z, w

    def publish_loop_marker(self):
        if self.gripper_pose_stamped is None:
            return
        # prefer the pose already transformed to world (if available), otherwise use original frame
        ps = self.gripper_pose_world if self.gripper_pose_world is not None else self.gripper_pose_stamped

        cx = ps.pose.position.x
        cy = ps.pose.position.y
        cz = ps.pose.position.z
        qx = ps.pose.orientation.x
        qy = ps.pose.orientation.y
        qz = ps.pose.orientation.z
        qw = ps.pose.orientation.w

        # --- Zusatzrotation zum Test: 90 Grad um lokale X-Achse ---
        angle_deg = 90.0
        if angle_deg != 0.0:
            a = math.radians(angle_deg / 2.0)
            sx = math.sin(a)
            ca = math.cos(a)        # <-- benutze 'ca' statt 'cx' um Position nicht zu überschreiben
            # Quaternion für Rotation um X: (x,y,z,w) = (sin(a),0,0,cos(a))
            q_extra = (sx, 0.0, 0.0, ca)
            q_orig = (qx, qy, qz, qw)
            # Falls du die Zusatzrotation VOR der Originalausrichtung anwenden willst, tausche die Reihenfolge:
            # qx, qy, qz, qw = self._quat_mul(q_extra, q_orig)
            qx, qy, qz, qw = self._quat_mul(q_orig, q_extra)

        if abs(self.loop_x_offset) > 0 or abs(self.loop_y_offset) > 0 or abs(self.loop_z_offset) > 0:
            ox, oy, oz = self._quat_rotate_vector(qx, qy, qz, qw, self.loop_x_offset, self.loop_y_offset, self.loop_z_offset)
            cx += ox
            cy += oy
            cz += oz

        marker = Marker()
        # publish marker in the pose's frame (if transformed to world, header.frame_id == "world")
        marker.header.frame_id = ps.header.frame_id if ps.header and ps.header.frame_id else "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gripper_loop"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # simple and efficient
        marker.action = Marker.ADD
        marker.scale.x = self.loop_thickness  # Strichdicke
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # build circle in gripper-local XY plane, rotate each point by gripper orientation
        for i in range(self.loop_segments + 1):  # +1, um den Kreis zu schließen
            angle = 2 * math.pi * i / self.loop_segments
            lx = self.loop_radius * math.cos(angle)
            ly = self.loop_radius * math.sin(angle)
            lz = 0.0  # circle lies in local XY plane

            # rotate local point into world by quaternion (with optional extra rotation applied)
            rx, ry, rz = self._quat_rotate_vector(qx, qy, qz, qw, lx, ly, lz)

            p = Point()
            p.x = cx + rx
            p.y = cy + ry
            p.z = cz + rz
            marker.points.append(p)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GreiferLoopCylinders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
