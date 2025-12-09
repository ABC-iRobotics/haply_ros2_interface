#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math
import tf2_ros
import tf2_geometry_msgs
import crtk
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class HapticLoop(Node):
    """Visualizes a loop attached to the PSM1 gripper using a single Marker"""

    def __init__(self):
        super().__init__("greifer_loop_cylinders")

        # Publisher for marker and loop center
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", 10)
        self.loop_center_pub = self.create_publisher(PoseStamped, "loop_center", 10)

        # Subscriber for PSM1 pose
        self.create_subscription(PoseStamped, "/PSM1/local/measured_cp", self.arm_callback, 10)
        self.create_subscription(JointState, "/PSM1/jaw/measured_js", self.gripper_callback, 10)

        # Parameter Loop
        self.loop_radius = 0.011                    # loop radius
        self.loop_segments = 10                     # number segments to approximate the circle (keep low for performance)
        self.loop_thickness = 0.002                 # cylinder diameter 2 mm
        self.loop_x_offset = 0.00                   # offset along gripper X axis
        self.loop_y_offset = 0.01+self.loop_radius  # offset along gripper Y axis
        self.loop_z_offset = 0.00                   # offset along gripper Z axis

        # Initialize variables
        self.gripper_pose_stamped = None
        self.gripper_pose_world = None
        # TF buffer/listener for transforming to world
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Static transform broadcaster for PSM1_base -> world frame (needed for smooth RViz visualization)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "world"
        static_transform.child_frame_id = "PSM1_base"
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_transform)

        # Timer for continuous visualization
        self.timer = self.create_timer(1/1000, self.publish_loop_marker)

        self.get_logger().info("Haptic Loop Visualization initialized.")


    def gripper_callback(self, msg: JointState):
        # Read gripper jaw position 
        jaw_pos = msg.position
        if jaw_pos is not None and len(jaw_pos) > 0:
            self.get_logger().info(f"Actual gripper state: {jaw_pos[0]}")
        else:
            self.get_logger().info("No gripper state available.")
        if jaw_pos[0] < 0.5:
            # gripper closed -> hide loop by setting radius to zero
            self.loop_radius = 0.011
        else:
            # gripper open -> set loop radius
            self.loop_radius = 0.0
        

    def arm_callback(self, msg: PoseStamped):
        # keep full pose
        self.gripper_pose_stamped = msg

        # try to transform into world frame 
        try:
            if msg.header.frame_id != "world":
                # lookup transform world <- msg.header.frame_id
                t = self.tf_buffer.lookup_transform("world", msg.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.5))
                transformed = tf2_geometry_msgs.do_transform_pose(msg, t)
                self.gripper_pose_world = transformed
            else:
                self.gripper_pose_world = msg
        except Exception as e:
            self.get_logger().debug(f"TF transform to 'world' failed: {e}")
            self.gripper_pose_world = None


    def rotate_vector_by_quaternion(self, qx, qy, qz, qw, vector_x, vector_y, vector_z):
        # Compute the cross product q_xyz × v, which is part of the quaternion rotation formula
        cross_x = qy * vector_z - qz * vector_y
        cross_y = qz * vector_x - qx * vector_z
        cross_z = qx * vector_y - qy * vector_x

        # Multiply the cross product by 2 to form the intermediate vector 
        buffervector_x = 2.0 * cross_x
        buffervector_y = 2.0 * cross_y
        buffervector_z = 2.0 * cross_z

        # Compute the final rotated vector 
        second_cross_x = qy * buffervector_z - qz * buffervector_y
        second_cross_y = qz * buffervector_x - qx * buffervector_z
        second_cross_z = qx * buffervector_y - qy * buffervector_x
        rotated_vector_x = vector_x + qw * buffervector_x + second_cross_x
        rotated_vector_y = vector_y + qw * buffervector_y + second_cross_y
        rotated_vector_z = vector_z + qw * buffervector_z + second_cross_z
        # Return the rotated vector in global/world coordinates
        return rotated_vector_x, rotated_vector_y, rotated_vector_z


    def multiply_quaternions(self, quaternion_a, quaternion_b):
        ax, ay, az, aw = quaternion_a
        bx, by, bz, bw = quaternion_b
        # Combines the rotations of a and b into a single rotation
        result_quaternion_w = aw*bw - ax*bx - ay*by - az*bz
        result_quaternion_x = aw*bx + ax*bw + ay*bz - az*by
        result_quaternion_y = aw*by - ax*bz + ay*bw + az*bx
        result_quaternion_z = aw*bz + ax*by - ay*bx + az*bw
        # Return the resulting quaternion (x, y, z, w)
        return result_quaternion_x, result_quaternion_y, result_quaternion_z, result_quaternion_w


    def publish_loop_marker(self):
        if self.gripper_pose_stamped is None:
            return
        # prefer the pose already transformed to world (if available), otherwise use original frame
        pose_gripper = self.gripper_pose_world if self.gripper_pose_world is not None else self.gripper_pose_stamped

        position_gripper_x = pose_gripper.pose.position.x
        position_gripper_y = pose_gripper.pose.position.y
        position_gripper_z = pose_gripper.pose.position.z
        quaternion_gripper_x = pose_gripper.pose.orientation.x
        quaternion_gripper_y = pose_gripper.pose.orientation.y
        quaternion_gripper_z = pose_gripper.pose.orientation.z
        quaternion_gripper_w = pose_gripper.pose.orientation.w

        # additional rotation of 90 degrees around gripper X to align loop plane with gripper jaws
        loop_angle = 90.0
        if loop_angle != 0.0:
            half_angle_rad = math.radians(loop_angle / 2.0)
            sin_half_angle = math.sin(half_angle_rad)
            cos_half_angle = math.cos(half_angle_rad)        
            # rotation quaternion around X axis
            loop_rotation_quaternion = (sin_half_angle, 0.0, 0.0, cos_half_angle)
            gripper_orientation_quaternion = (quaternion_gripper_x, quaternion_gripper_y, quaternion_gripper_z, quaternion_gripper_w)
            quaternion_gripper_x, quaternion_gripper_y, quaternion_gripper_z, quaternion_gripper_w = self.multiply_quaternions(gripper_orientation_quaternion, loop_rotation_quaternion)

        # apply optional offsets along gripper axes
        if abs(self.loop_x_offset) > 0 or abs(self.loop_y_offset) > 0 or abs(self.loop_z_offset) > 0:
            offset_world_x, offset_world_y, offset_world_z = self.rotate_vector_by_quaternion(quaternion_gripper_x, quaternion_gripper_y, quaternion_gripper_z, quaternion_gripper_w, self.loop_x_offset, self.loop_y_offset, self.loop_z_offset)
            position_gripper_x += offset_world_x
            position_gripper_y += offset_world_y
            position_gripper_z += offset_world_z

        marker = Marker()
        # publish marker in the pose's frame ("world" leads to latency issues)
        marker.header.frame_id = pose_gripper.header.frame_id if pose_gripper.header and pose_gripper.header.frame_id else "world"
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
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # build circle in gripper-local XY plane, rotate each point by gripper orientation
        for segment_index in range(self.loop_segments + 1):  
            segment_angle = 2 * math.pi * segment_index / self.loop_segments
            local_x = self.loop_radius * math.cos(segment_angle)
            local_y = self.loop_radius * math.sin(segment_angle)
            local_z = 0.0  

            # rotate local point into world by quaternion (with optional extra rotation applied)
            world_x, world_y, world_z = self.rotate_vector_by_quaternion(quaternion_gripper_x, quaternion_gripper_y, quaternion_gripper_z, quaternion_gripper_w, local_x, local_y, local_z)

            p = Point()
            p.x = position_gripper_x + world_x
            p.y = position_gripper_y + world_y
            p.z = position_gripper_z + world_z
            marker.points.append(p)

        self.marker_pub.publish(marker)

        # publish loop center pose
        loop_center = PoseStamped()
        loop_center.header = marker.header
        loop_center.pose.position.x = position_gripper_x
        loop_center.pose.position.y = position_gripper_y
        loop_center.pose.position.z = position_gripper_z
        loop_center.pose.orientation.x = quaternion_gripper_x
        loop_center.pose.orientation.y = quaternion_gripper_y
        loop_center.pose.orientation.z = quaternion_gripper_z
        loop_center.pose.orientation.w = quaternion_gripper_w

        self.loop_center_pub.publish(loop_center)
        #self.get_logger().info(f"Loopcenter published at: {loop_center.pose.position.x},{loop_center.pose.position.y},{loop_center.pose.position.z}")


def main(args=None):
    rclpy.init(args=args)
    node = HapticLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
