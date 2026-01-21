#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import math

class AutoTrialController(Node):
    def __init__(self):
        super().__init__("auto_trial_controller")

        # Parameters
        self.start_point = [-0.05, 0.05, -0.2]
        self.end_point = [0.03, 0.1, -0.2]
        self.reset_point = [0.0, 0.05, -0.1]
        self.tolerance = 0.002  # 2 mm box

        # Default State
        self.trial_state = "IDLE"

        # Publishers
        self.state_pub = self.create_publisher(String, "/trial/state", 10)
        self.marker_pub = self.create_publisher(Marker, "/trial/tolerance_marker", 10)

        # Subscribers
        self.create_subscription(PoseStamped, "loop_center", self.loop_center_cb, 10)

        # Timer 
        self.create_timer(0.2, self.publish_markers)
        self.get_logger().info("AutoTrialController initialized.")

    # -----------------------------------------------------------------------------
    def loop_center_cb(self, msg: PoseStamped):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

        if self.is_inside_tolerance([x, y, z], self.start_point):
            if self.trial_state != "RUNNING":
                self.trial_state = "RUNNING"
                self.get_logger().info("Trial started (RUNNING)!")
        elif self.is_inside_tolerance([x, y, z], self.end_point):
            if self.trial_state != "FINISHED":
                self.trial_state = "FINISHED"
                self.get_logger().info("Trial ended (FINISHED)")
        elif self.is_inside_tolerance([x, y, z], self.reset_point):
            if self.trial_state != "IDLE":
                self.trial_state = "IDLE"
                self.get_logger().info("Trial reset (IDLE)")

        # Publish state
        self.get_logger().info(f"Current trial state: {self.trial_state}")
        self.state_pub.publish(String(data=self.trial_state))

    # -----------------------------------------------------------------------------
    def is_inside_tolerance(self, point, target):
        return all(abs(point[i] - target[i]) <= self.tolerance for i in range(3))

    # -----------------------------------------------------------------------------
    def publish_markers(self):
        # Start Marker
        start_marker = self.create_marker(self.start_point, marker_id=0)
        if self.trial_state == "RUNNING":
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0  # green = running
            start_marker.color.b = 0.0
        elif self.trial_state == "FINISHED":
            start_marker.color.r = 1.0  # red = finished
            start_marker.color.g = 0.0
            start_marker.color.b = 0.0  
        else:
            start_marker.color.r = 0.0
            start_marker.color.g = 0.0
            start_marker.color.b = 1.0  # blue = idle
        self.marker_pub.publish(start_marker)

        # End Marker
        end_marker = self.create_marker(self.end_point, marker_id=1)
        if self.trial_state == "RUNNING":
            end_marker.color.r = 0.0
            end_marker.color.g = 0.0  
            end_marker.color.b = 1.0  # blue = running
        elif self.trial_state == "FINISHED":
            end_marker.color.r = 0.0 
            end_marker.color.g = 1.0  # green = finished
            end_marker.color.b = 0.0  
        else:
            end_marker.color.r = 1.0  # red = idle
            end_marker.color.g = 0.0
            end_marker.color.b = 0.0  
        self.marker_pub.publish(end_marker)

        # Reset Marker
        reset_marker = self.create_marker(self.reset_point, marker_id=2)
        if self.trial_state == "RUNNING":
            reset_marker.color.r = 1.0  # red = running
            reset_marker.color.g = 0.0  
            reset_marker.color.b = 0.0
        elif self.trial_state == "FINISHED":
            reset_marker.color.r = 0.0  
            reset_marker.color.g = 0.0
            reset_marker.color.b = 1.0  # blue = finished
        else:
            reset_marker.color.r = 0.0
            reset_marker.color.g = 1.0  # green = idle
            reset_marker.color.b = 0.0 
        self.marker_pub.publish(reset_marker)

    # -----------------------------------------------------------------------------
    def create_marker(self, point, marker_id=0):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trial_tolerance"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.tolerance * 2
        marker.scale.y = self.tolerance * 2
        marker.scale.z = self.tolerance * 2

        marker.color.a = 0.6  # semi-transparent
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = AutoTrialController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
