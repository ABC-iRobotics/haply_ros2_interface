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

        # Timer for continuous visualization (10 Hz)
        self.marker_timer = self.create_timer(1 / 10, self.publish_wire_marker)

        self.get_logger().info(
            f"Hot Wire Visualization started"
        )

        # Define start and end points of the wire in cartesian coordinates (unit: meters)
        self.wire_start = [-0.05, 0.0, -0.1]
        self.wire_end = [0.05, 0.025, -0.1]
        # Haptic stiffness and damping factors for virtual spring-damper system
        self.stiffness = 200.0
        self.damping = 0.0
        # define attraction zone (distance loopcenter from wire) 
        self.inner_limit_deadzone = 0.001  
        self.outer_limit_deadzone = 0.005  
        # use force feedback toggle (True/False)
        self.use_force_feedback = False

        self.get_logger().info(
            f"Haply Force Controller initialized: stiffness={self.stiffness}")


    def state_callback(self, msg):
        # Extract device position
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
        control_msg.force = Vector3(x=force[0], y=force[1], z=force[2])
        control_msg.target_position = Point(x=0.0, y=0.0, z=0.0)  

        # Publish
        self.force_publisher.publish(control_msg)
        self.get_logger().info(f"Published Force: x={force[0]:.3f}, y={force[1]:.3f}, z={force[2]:.3f}")


    def compute_haptic_force(self, device_position):
        """Compute attractive force pulling the loop center toward the wire when distance is between deadzone and loop."""

        wire_start = self.wire_start
        wire_end = self.wire_end
        loopcenter_position = device_position

        # wire direction vector and length 
        wire_vector = [wire_end[i] - wire_start[i] for i in range(3)]
        wire_length = math.sqrt(sum(v ** 2 for v in wire_vector))
        if wire_length == 0.0:
            self.get_logger().warning("Wire length is zero!")
            return [0.0, 0.0, 0.0]
        wire_direction = [v / wire_length for v in wire_vector]

        # vector from wire start to loop center
        wirestart_to_loopcenter = [loopcenter_position[i] - wire_start[i] for i in range(3)]

        # projection onto wire to find closest point on the wire
        projection_length = sum(wirestart_to_loopcenter[i] * wire_direction[i] for i in range(3))
        projection_length = max(0.0, min(wire_length, projection_length))  # clamp to segment

        # calculate closest point on wire 
        closest_point = [
            wire_start[i] + projection_length * wire_direction[i] for i in range(3)
        ]

        # vector from loop center to closest point on wire
        loopcenter_to_wire = [
            closest_point[i] - loopcenter_position[i] for i in range(3)
        ]
        distance = math.sqrt(sum(v ** 2 for v in loopcenter_to_wire))
        if distance < 1e-6:
            return [0.0, 0.0, 0.0]

        # normalized direction (from loopcenter toward wire)
        direction = [v / distance for v in loopcenter_to_wire]

        # check if within attraction range 
        if distance <= self.inner_limit_deadzone or distance >= self.outer_limit_deadzone:
            return [0.0, 0.0, 0.0]

        # attractive force toward wire 
        raw_force = [direction[i] * self.stiffness * distance for i in range(3)]

        if self.use_force_feedback:
            return raw_force  
        else:
            return [0.0, 0.0, 0.0] 


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
            self.get_logger().warning("Wire length is zero!")
            return
        else:
            direction = (distance_x/wirelength, distance_y/wirelength, distance_z/wirelength)

        # compute midpoints
        midpoint_x = (start_x + end_x) / 2.0
        midpoint_y = (start_y + end_y) / 2.0
        midpoint_z = (start_z + end_z) / 2.0

        # compute rotation quaternion from +Z (default orientation cylinder) to new orientation
        qx, qy, qz, qw = self.quat_from_two_vectors((0.0, 0.0, 1.0), direction)

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

        # scale: x/y = diameter, z = length (unit: meters)
        marker.scale.x = 0.002 
        marker.scale.y = 0.002
        marker.scale.z = wirelength

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_publisher.publish(marker)


    # compute quaternion rotating +Z to direction
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
