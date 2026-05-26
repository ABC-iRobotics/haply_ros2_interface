#!/usr/bin/env python3
import math

import rclpy

from haply_daVinci_teleop.haptic_visualization.haptic_hotwire import HotWire


class HotWireDemo(HotWire):
    """Hotwire demo with a short three-point wire path."""

    def __init__(self):
        super().__init__()

        self.stiffness = 100
        self.damping = 1.0

        self.measure_wirepoints_mode = False
        self.given_wirepoints = [
            [0.0, 0.0, -0.15],
            [0.05, 0.0, -0.15],
            [0.1, 0.0, -0.2],
        ]
        self.wire_points = self.given_wirepoints.copy()

        self.get_logger().info(
            "Demo mode enabled: using three predefined wire points."
        )

    def gripper_callback(self, msg):
        """Ignore empty jaw states that can appear during demo startup."""
        if not msg.position:
            return

        super().gripper_callback(msg)

    def compute_force_on_segment(self, loop_center_position, wire_start_point, wire_end_point):
        """Compute demo force without disabling it outside the attraction zone."""
        wire_segment_vector = [
            wire_end_point[i] - wire_start_point[i] for i in range(3)
        ]
        wire_segment_length = math.sqrt(sum(v * v for v in wire_segment_vector))

        if wire_segment_length < 1e-6:
            return [0.0, 0.0, 0.0], 999.0

        wire_direction_unit = [v / wire_segment_length for v in wire_segment_vector]
        loop_to_wire_vector = [
            loop_center_position[i] - wire_start_point[i] for i in range(3)
        ]

        projection_length = sum(
            loop_to_wire_vector[i] * wire_direction_unit[i] for i in range(3)
        )
        projection_length = max(0.0, min(wire_segment_length, projection_length))

        closest_point_on_wire = [
            wire_start_point[i] + projection_length * wire_direction_unit[i]
            for i in range(3)
        ]
        vector_wire_to_loop = [
            loop_center_position[i] - closest_point_on_wire[i] for i in range(3)
        ]
        distance_center_to_wire = math.sqrt(sum(v * v for v in vector_wire_to_loop))
        effective_distance = distance_center_to_wire

        if effective_distance < self.inner_limit_deadzone:
            return [0.0, 0.0, 0.0], effective_distance

        normalized_dir = [
            vector_wire_to_loop[i] / distance_center_to_wire for i in range(3)
        ]

        if self.force_feedback_mode == 0:
            force_vector = [0.0, 0.0, 0.0]
        elif self.force_feedback_mode == 1:
            force_vector = [
                normalized_dir[i] * self.stiffness * effective_distance
                - self.velocity[i] * self.damping
                for i in range(3)
            ]
        elif self.force_feedback_mode == 2:
            k1 = self.stiffness * 0.5
            delta1 = self.midzone - self.inner_limit_deadzone
            delta2 = self.outer_limit_deadzone - self.midzone
            target_force_outer = self.stiffness * self.outer_limit_deadzone
            k2 = (target_force_outer - k1 * delta1) / delta2

            if effective_distance <= self.midzone:
                force_magnitude = k1 * (
                    effective_distance - self.inner_limit_deadzone
                )
            else:
                force_offset = k1 * (self.midzone - self.inner_limit_deadzone)
                force_magnitude = force_offset + k2 * (
                    effective_distance - self.midzone
                )
            force_magnitude = max(force_magnitude, 0.0)
            force_vector = [
                normalized_dir[i] * force_magnitude
                - self.velocity[i] * self.damping
                for i in range(3)
            ]
        else:
            self.get_logger().info("Please select a valid force-mode.")
            force_vector = [0.0, 0.0, 0.0]

        return force_vector, effective_distance


def main(args=None):
    rclpy.init(args=args)
    node = HotWireDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
