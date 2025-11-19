#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Vector3, Point
from haply_msgs.msg import HaplyState, HaplyControl


class HotWire(Node):
    """Visualizes a straight wire (cylinder) for the Hot Wire experiment"""

    def __init__(self):
        super().__init__("hot_wire_visualization")   

        # Publisher
        self.marker_publisher = self.create_publisher(Marker, "hotwire_marker", 10)
        self.force_publisher = self.create_publisher(HaplyControl, 'haply_target', 10)

        # Subscriber
        self.state_subscriber = self.create_subscription(HaplyState,'haply_state', self.state_callback, 10)
        self.loop_subscription = self.create_subscription(PoseStamped,'loop_center', self.loop_center_callback, 10)
        self.psm_cp_subscriber = self.create_subscription(PoseStamped, '/PSM1/local/measured_cp', self.psm_cp_callback, 10)

        # Timer for continuous visualization (10 Hz)
        self.marker_timer = self.create_timer(1 / 10, self.publish_wire_marker)

        # Initialize button and wire state 
        self.last_button_c = False
        self.wire_points = []

        # Haptic stiffness and damping factors for virtual spring-damper system
        self.stiffness = 200.0
        self.damping = 0.0
        # define attraction zone (distance loopcenter from wire) 
        self.inner_limit_deadzone = 0.001  
        self.outer_limit_deadzone = 0.005  
        # use force feedback toggle (True/False)
        self.use_force_feedback = True

        #self.get_logger().info(f"Haply Force Controller initialized: stiffness={self.stiffness} and damping={self.damping}")
        self.get_logger().info(f"Hot Wire Visualization started. Waiting for measuring points...")
        

    def psm_cp_callback(self, msg: PoseStamped):
        """Stores the most recent PSM1 TCP coordinate"""
        self.current_psm_cp = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]


    def state_callback(self, msg: HaplyState):
        # Rising edge detection for button C
        current = msg.buttons.c

        if current and not self.last_button_c:
            if self.current_psm_cp is None:
                self.get_logger().warning("No PSM position received yet!")
            else:
                self.wire_points.append(self.current_psm_cp.copy())
                self.get_logger().info(f"Added point {len(self.wire_points)} / {self.current_psm_cp}")

        self.last_button_c = current

        haply_velocity = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        #self.get_logger().info(f"haply Velocity: x={haply_velocity[0]:.3f}, y={haply_velocity[1]:.3f}, z={haply_velocity[2]:.3f}")


    def loop_center_callback(self, msg: PoseStamped):
        """Callback to update wire position based on loop center"""

        # Extract device position
        device_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # Compute the haptic force
        force = self.compute_haptic_force(device_position)

        # Create the force control message
        control_msg = HaplyControl()
        control_msg.use_position = False  
        # Attention: x and y direction is mirrored in haply frame!
        control_msg.force = Vector3(x=-force[0], y=-force[1], z=force[2])
        control_msg.target_position = Point(x=0.0, y=0.0, z=0.0) # default   

        # Publish
        self.force_publisher.publish(control_msg)
        self.get_logger().info(f"Published Force: x={force[0]:.3f}, y={force[1]:.3f}, z={force[2]:.3f}")


    def compute_haptic_force(self, device_position):
        """Compute attractive force pulling the loop center toward the wire when distance is between deadzone and loop."""

        if len(self.wire_points) < 2:
            return [0.0,0.0,0.0]

        closest_force = None
        closest_dist = 999

        # Loop over all segments
        for i in range(len(self.wire_points)):
            if i == len(self.wire_points)-1:
                # Close shape if polygon
                p1 = self.wire_points[i]
                p2 = self.wire_points[0]
            else:
                p1 = self.wire_points[i]
                p2 = self.wire_points[i+1]

            force, dist = self.compute_force_on_segment(device_position, p1, p2)

            if dist < closest_dist:
                closest_dist = dist
                closest_force = force

        if closest_force is None:
            return [0.0, 0.0, 0.0]
        return [float(closest_force[0]), float(closest_force[1]), float(closest_force[2])]

        

    def compute_force_on_segment(self, loopcenter_position, wire_start, wire_end):
        wire_vector = [wire_end[i] - wire_start[i] for i in range(3)]
        wire_length = math.sqrt(sum(v*v for v in wire_vector))
        if wire_length < 1e-6:
            return [0.0,0.0,0.0], 999.0

        wire_dir = [v / wire_length for v in wire_vector]

        w2l = [loopcenter_position[i] - wire_start[i] for i in range(3)]
        proj = sum(w2l[i] * wire_dir[i] for i in range(3))
        proj = max(0.0, min(wire_length, proj))

        closest = [wire_start[i] + proj * wire_dir[i] for i in range(3)]
        diff = [closest[i] - loopcenter_position[i] for i in range(3)]
        dist = math.sqrt(sum(v*v for v in diff))

        if dist < self.inner_limit_deadzone or dist > self.outer_limit_deadzone:
            return [0.0,0.0,0.0], float(dist)

        direction = [d/dist for d in diff]
        force = [direction[i] * self.stiffness * dist for i in range(3)]

        return force, dist


    def publish_wire_marker(self):

        if len(self.wire_points) < 2:
            return

        # If polygon closed (4 points)
        closed = False
        if len(self.wire_points) >= 4:
            closed = True

        # Publish one CYLINDER per segment
        marker_array = []

        for i in range(len(self.wire_points)):
            if i == len(self.wire_points)-1:
                if closed:
                    p1 = self.wire_points[i]
                    p2 = self.wire_points[0]
                else:
                    break
            else:
                p1 = self.wire_points[i]
                p2 = self.wire_points[i+1]

            sx, sy, sz = p1
            ex, ey, ez = p2

            dx = ex - sx
            dy = ey - sy
            dz = ez - sz

            length = math.sqrt(dx*dx + dy*dy + dz*dz)
            if length < 1e-6:
                continue

            direction = (dx/length, dy/length, dz/length)
            mid = ((sx+ex)/2, (sy+ey)/2, (sz+ez)/2)

            qx,qy,qz,qw = self.quat_from_two_vectors((0,0,1), direction)

            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "hot_wire"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = mid[0]
            marker.pose.position.y = mid[1]
            marker.pose.position.z = mid[2]
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw

            marker.scale.x = 0.002
            marker.scale.y = 0.002
            marker.scale.z = length

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_publisher.publish(marker)


    def quat_from_two_vectors(self, vector_from, vector_to):
        """ returns (x,y,z,w) quaternion rotating vector_from to vector_to """
        from_x, from_y, from_z = vector_from
        to_x, to_y, to_z = vector_to

        # calculate cross product for rotation axis
        cross_x = from_y * to_z - from_z * to_y
        cross_y = from_z * to_x - from_x * to_z
        cross_z = from_x * to_y - from_y * to_x

        # calculate scalar product
        dot_product = from_x*to_x + from_y*to_y + from_z*to_z
        # Normal of cross product axis
        cross_norm = math.sqrt(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z)

        if cross_norm < 1e-6:
            # parallel or anti-parallel
            if dot_product > 0.9999:
                return (0.0, 0.0, 0.0, 1.0)
            else:
                # 180 deg rotation about X (arbitrary orthogonal axis)
                return (1.0, 0.0, 0.0, 0.0)
        
        # rotation axis 
        rotation_axis = (cross_x / cross_norm, cross_y / cross_norm, cross_z / cross_norm)

        # rotation angle
        rotation_angle = math.atan2(cross_norm, dot_product)
        sin_half = math.sin(rotation_angle/2.0)
        qx = rotation_axis[0] * sin_half
        qy = rotation_axis[1] * sin_half
        qz = rotation_axis[2] * sin_half
        qw = math.cos(rotation_angle/2.0)
        return (qx, qy, qz, qw)


def main(args=None):
    rclpy.init(args=args)
    node = HotWire()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
