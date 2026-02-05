#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Vector3, Point
from haply_msgs.msg import HaplyState, HaplyControl
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class HotWire(Node):
    """Visualizes a straight wire (cylinder) for the Hot Wire experiment"""

    def __init__(self):
        super().__init__("hot_wire_visualization")   

        # declare parameter with default
        self.declare_parameter("mode", 0)

        self.force_feedback_mode = (
            self.get_parameter("mode")
            .get_parameter_value()
            .integer_value
        )

        if self.force_feedback_mode not in (0, 1, 2):
            raise ValueError("mode must be 0, 1 or 2")


        # Publisher
        self.marker_publisher = self.create_publisher(Marker, "hotwire_marker", 10)
        self.force_publisher = self.create_publisher(HaplyControl, "haply_target", 10)
        self.contact_publisher = self.create_publisher(Bool, "contact_status", 10)

        # Subscriber
        self.state_subscriber = self.create_subscription(HaplyState,"haply_state", self.state_callback, 10)
        self.loop_subscription = self.create_subscription(PoseStamped,"loop_center", self.loop_center_callback, 10)
        self.psm_cp_subscriber = self.create_subscription(PoseStamped, "/PSM1/local/measured_cp", self.psm_cp_callback, 10)
        self.gripper_subscriber = self.create_subscription(JointState, "/PSM1/jaw/measured_js", self.gripper_callback, 10)
        
        # Store current loop orientation for angle calculation
        self.loop_orientation = None
        self.loop_radius = 0.01  # make sure this matches the real loop radius [m]

        # Timer for continuous visualization (100 Hz)
        self.marker_timer = self.create_timer(1 / 100, self.publish_wire_marker)
        self.contact_timer = self.create_timer(1 / 100, self.publish_contact_status)  

        # Initalize contact status for contact detection publishing
        self.contact_status = False
        self.contact_counter = 0
        self.contact_counter_hot = False

        # Initialize button and wire state 
        self.last_button_c = False
        self.gripper_closed = False
        self.wire_points = []
 
        # Distance from Haply tip to PSM1 base (unit: [m])
        self.distance_tip_to_PSM1_base = 0.01

        #self.force_feedback_mode = 1
        # choose wirepoints source
        self.measure_wirepoints_mode = False
        # predefined wirepoints 
        self.given_wirepoints = [
            [-0.05, 0.05, -0.2],
            [-0.05, 0.05, -0.16],
            [-0.02, 0.05, -0.16],
            [-0.02, 0.1, -0.16],
            [0.03, 0.1, -0.16],
            [0.03, 0.1, -0.2],
        ]

        # Haptic stiffness and damping
        self.stiffness = 80.0
        self.damping = 2.0

        # Attraction zone (distance wire-to-loop center, unit: [m])
        self.inner_limit_deadzone = 0.001  
        self.outer_limit_deadzone = self.loop_radius + 0.005
        self.midzone = (self.inner_limit_deadzone + self.outer_limit_deadzone) / 2.0

        self.current_psm_cp = None
        self.closest_distance = 999
        self.closest_wire_segment_vector = None
        self.get_logger().info(f"Hot Wire Visualization started. Waiting for measuring points...")
        
        if self.measure_wirepoints_mode == False:
            if len(self.wire_points) == 0:
                self.wire_points = self.given_wirepoints.copy()
                self.get_logger().info("Loaded predefined wire points.")
        

    def psm_cp_callback(self, msg: PoseStamped):
        """ Store current PSM1 cartesian position """
        self.current_psm_cp = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    # -------------------------------------------------------------------------
    def publish_contact_status(self):
        """ Publish contact status as Bool message """
        # Calculate contact status based on closest distance and loop angle
        self.contact_status = self.compute_contact_status(self.closest_distance, self.closest_wire_segment_vector)
        
        msg = Bool()
        
        if self.contact_status == True:
            msg.data = True
            if self.contact_counter_hot == True:
                self.contact_counter += 1
                self.contact_counter_hot = False
        else:
            self.contact_counter_hot = True
            msg.data = False
        self.contact_publisher.publish(msg)

    # -------------------------------------------------------------------------
    def gripper_callback(self, msg: JointState):
        """ Receive gripper jaw position to determine open/closed state """
        # Read gripper jaw position 
        jaw_pos = msg.position
        if jaw_pos[0] < 0.5:
            # gripper closed -> hide loop by setting radius to zero
            self.gripper_closed = True
        else:
            # gripper open -> set loop radius
            self.gripper_closed = False

    # -------------------------------------------------------------------------
    def state_callback(self, msg: HaplyState):
        """ Receive Haply State """
        current_button_state = msg.buttons.c

        if self.measure_wirepoints_mode == True and current_button_state and not self.last_button_c:
            if self.current_psm_cp is None:
                self.get_logger().warning("No PSM position received yet!")
            else:
                self.wire_points.append(self.current_psm_cp.copy())
                self.get_logger().info(f"Added point {len(self.wire_points)} / {self.current_psm_cp}")

        self.last_button_c = current_button_state
        self.velocity = [msg.velocity.x, msg.velocity.y, -msg.velocity.z]
        #self.get_logger().info(f"velocity: x={self.velocity[0]:.3f}, y={self.velocity[1]:.3f}, z={self.velocity[2]:.3f}")

    # -------------------------------------------------------------------------
    def loop_center_callback(self, msg: PoseStamped):
        """ Measure loop position, calculate distance and force and publish. """
        loop_center_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.current_loop_center = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

        # Store loop orientation for angle calculation
        self.loop_orientation = msg.pose.orientation
        force_vector = self.compute_haptic_force(loop_center_position)

        control_msg = HaplyControl()
        control_msg.use_position = False  
        control_msg.force = Vector3(
            x=force_vector[0],
            y=force_vector[1],
            z=-force_vector[2]
        )
        control_msg.target_position = Point(x=0.0, y=0.0, z=0.0)

        self.force_publisher.publish(control_msg)
        #self.get_logger().info(f"Published Force: x={force_vector[0]:.3f}, y={force_vector[1]:.3f}, z={force_vector[2]:.3f}")

    #  -------------------------------------------------------------------------
    def compute_haptic_force(self, loop_center_position):
        """ Compute haptic force using the distance from loopcenter to wire and velocity of haply """
        if len(self.wire_points) < 2:
            return [0.0, 0.0, 0.0]

        closest_force_vector = None
        self.closest_distance = 999
        self.closest_wire_segment_vector = None

        # calculate iteratively the closest distance between loop center and wire as well as the appropriate force vector
        for idx in range(len(self.wire_points)):
            if idx == len(self.wire_points)-1:
                break
            else:
                wire_start_point = self.wire_points[idx]
                wire_end_point = self.wire_points[idx+1]

            # compute force and distance on each segment
            force_vector, segment_distance = self.compute_force_on_segment(loop_center_position, wire_start_point, wire_end_point)

            # find the overall closest segment and its force vector
            if segment_distance < self.closest_distance:
                self.closest_distance = segment_distance
                closest_force_vector = force_vector
                self.closest_wire_segment_vector = [wire_end_point[i] - wire_start_point[i] for i in range(3)]

        if closest_force_vector is None or self.gripper_closed == False:
            return [0.0, 0.0, 0.0]

        return [
            float(closest_force_vector[0]),
            float(closest_force_vector[1]),
            float(closest_force_vector[2])
        ]
    
    # -------------------------------------------------------------------------
    def segment_intersects_loop(self, P0, P1, C, n, R, tol=5e-4):
        """Check if wire segment P0->P1 intersects the hollow loop ring. Contact only if intersection point lies on the ring (≈ radius R)."""

        # Segment direction
        d = [P1[i] - P0[i] for i in range(3)]

        denom = sum(n[i] * d[i] for i in range(3))
        if abs(denom) < 1e-6:
            # Segment parallel to loop plane
            return False

        t = sum(n[i] * (C[i] - P0[i]) for i in range(3)) / denom

        if t < 0.0 or t > 1.0:
            # Intersection not on segment
            return False

        # Intersection point
        X = [P0[i] + t * d[i] for i in range(3)]

        # Radial distance from loop center
        r = math.sqrt(sum((X[i] - C[i])**2 for i in range(3)))

        # Contact ONLY near the ring radius
        return abs(r - R) <= tol

    # -------------------------------------------------------------------------
    def compute_contact_status(self, _, __):
        if self.loop_orientation is None or self.current_loop_center is None:
            return False

        # Loop geometry
        C = self.current_loop_center
        n = self.quat_to_normal_vector(self.loop_orientation)
        R = self.loop_radius

        # Check all wire segments
        for i in range(len(self.wire_points) - 1):
            P0 = self.wire_points[i]
            P1 = self.wire_points[i + 1]

            if self.segment_intersects_loop(P0, P1, C, n, R) or self.closest_distance >= self.loop_radius:
                #self.get_logger().info("Contact detected!")
                return True

        return False
    # -------------------------------------------------------------------------

    def quat_to_normal_vector(self, quat):
        """ Convert quaternion to normal vector (z-axis of the rotated frame).This represents the orientation of the loop plane."""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        
        # Transform the z-axis unit vector (0, 0, 1) using the quaternion
        normal_x = 2 * (x * z + w * y)
        normal_y = 2 * (y * z - w * x)
        normal_z = 1 - 2 * (x * x + y * y)
        
        # Normalize
        norm = math.sqrt(normal_x**2 + normal_y**2 + normal_z**2)
        if norm < 1e-6:
            return [0.0, 0.0, 1.0]
        
        return [normal_x / norm, normal_y / norm, normal_z / norm]

    # -------------------------------------------------------------------------
    def compute_force_on_segment(self, loop_center_position, wire_start_point, wire_end_point):
        """ Be aware that this function is inside a loop over all segments! """
        """ Computes force vector and distance from loop center to a wire segment defined by start and end point """
        wire_segment_vector = [wire_end_point[i] - wire_start_point[i] for i in range(3)]
        wire_segment_length = math.sqrt(sum(v*v for v in wire_segment_vector))

        # make sure that segment length is not zero
        if wire_segment_length < 1e-6:
            return [0.0, 0.0, 0.0], 999.0

        wire_direction_unit = [v / wire_segment_length for v in wire_segment_vector]
        loop_to_wire_vector = [loop_center_position[i] - wire_start_point[i] for i in range(3)]

        projection_length = sum(loop_to_wire_vector[i] * wire_direction_unit[i] for i in range(3))
        projection_length = max(0.0, min(wire_segment_length, projection_length))

        closest_point_on_wire = [wire_start_point[i] + projection_length * wire_direction_unit[i] for i in range(3)]
        vector_wire_to_loop = [loop_center_position[i] - closest_point_on_wire[i] for i in range(3)]
        distance_center_to_wire = math.sqrt(sum(v*v for v in vector_wire_to_loop))

        # subtract loop radius
        effective_distance = distance_center_to_wire 
        #self.get_logger().info(f"effective distance: {effective_distance:.4f} m")

        if effective_distance < self.inner_limit_deadzone or effective_distance > self.outer_limit_deadzone:
            return [0.0, 0.0, 0.0], effective_distance

        normalized_dir = [vector_wire_to_loop[i] / distance_center_to_wire for i in range(3)]

        # compute force based on mode
        if self.force_feedback_mode == 0:
            force_vector = [0.0, 0.0, 0.0]
        elif self.force_feedback_mode == 1:
            force_vector = [normalized_dir[i] * self.stiffness * effective_distance - self.velocity[i] * self.damping for i in range(3)]
        elif self.force_feedback_mode == 2:
            # calculate force stepwise. First half of the zone: low force, second half: high force. There should be no step at the boundary.
            k1 = self.stiffness * 0.5
            delta1 = self.midzone - self.inner_limit_deadzone
            delta2 = self.outer_limit_deadzone - self.midzone
            target_force_outer = self.stiffness * self.outer_limit_deadzone  # Mode 1 force at outer limit
            k2 = (target_force_outer - k1 * delta1) / delta2
            
            if effective_distance <= self.midzone:
                force_magnitude = k1 * (effective_distance - self.inner_limit_deadzone)
            else:
                force_offset = k1 * (self.midzone - self.inner_limit_deadzone)
                force_magnitude = force_offset + k2 * (effective_distance - self.midzone)
            force_magnitude = max(force_magnitude, 0.0)
            force_vector = [normalized_dir[i] * force_magnitude - self.velocity[i] * self.damping for i in range(3)]
        else:
            self.get_logger().info("Please select a valid force-mode.")

        return force_vector, effective_distance
    
    # -------------------------------------------------------------------------
    def publish_wire_marker(self):
        if len(self.wire_points) < 2:
            return

        for idx in range(len(self.wire_points)):
            if idx == len(self.wire_points)-1:
                break
            else:
                point_start = self.wire_points[idx]
                point_end   = self.wire_points[idx+1]

            start_x, start_y, start_z = point_start
            end_x,   end_y,   end_z   = point_end

            delta_x = end_x - start_x
            delta_y = end_y - start_y
            delta_z = end_z - start_z

            segment_length = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
            if segment_length < 1e-6:
                continue

            direction_unit = (
                delta_x / segment_length,
                delta_y / segment_length,
                delta_z / segment_length
            )

            midpoint = (
                (start_x + end_x) / 2,
                (start_y + end_y) / 2,
                (start_z + end_z) / 2
            )

            quat_x, quat_y, quat_z, quat_w = self.quat_from_two_vectors((0,0,1), direction_unit)

            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "hot_wire"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = midpoint[0]
            marker.pose.position.y = midpoint[1]
            marker.pose.position.z = midpoint[2]
            marker.pose.orientation.x = quat_x
            marker.pose.orientation.y = quat_y
            marker.pose.orientation.z = quat_z
            marker.pose.orientation.w = quat_w

            marker.scale.x = 0.002
            marker.scale.y = 0.002
            marker.scale.z = segment_length

            # mark wire segment color based on contact status
            if self.contact_status == True:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            self.marker_publisher.publish(marker)

    # -------------------------------------------------------------------------
    def quat_from_two_vectors(self, vector_from, vector_to):
        from_x, from_y, from_z = vector_from
        to_x, to_y, to_z = vector_to

        cross_x = from_y * to_z - from_z * to_y
        cross_y = from_z * to_x - from_x * to_z
        cross_z = from_x * to_y - from_y * to_x

        dot_product = from_x*to_x + from_y*to_y + from_z*to_z
        cross_norm = math.sqrt(cross_x**2 + cross_y**2 + cross_z**2)

        if cross_norm < 1e-6:
            if dot_product > 0.9999:
                return (0.0, 0.0, 0.0, 1.0)
            else:
                return (1.0, 0.0, 0.0, 0.0)
        
        rotation_axis = (
            cross_x / cross_norm,
            cross_y / cross_norm,
            cross_z / cross_norm
        )

        rotation_angle = math.atan2(cross_norm, dot_product)
        sin_half = math.sin(rotation_angle/2.0)
        quat_x = rotation_axis[0] * sin_half
        quat_y = rotation_axis[1] * sin_half
        quat_z = rotation_axis[2] * sin_half
        quat_w = math.cos(rotation_angle/2.0)
        return (quat_x, quat_y, quat_z, quat_w)


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

