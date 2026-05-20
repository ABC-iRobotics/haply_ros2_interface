#!/usr/bin/env python3
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
