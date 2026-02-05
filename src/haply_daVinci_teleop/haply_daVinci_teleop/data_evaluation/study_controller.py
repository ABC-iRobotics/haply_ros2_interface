#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker

class StudyStateController(Node):
    def __init__(self):
        super().__init__("study_state_controller")

        # Parameters
        self.start_point = [-0.05, 0.05, -0.2]
        self.intersection_point1 = [-0.05, 0.05, -0.16]
        self.intersection_point2 = [-0.02, 0.05, -0.16]
        self.intersection_point3 = [-0.02, 0.1, -0.16]
        self.intersection_point4 = [0.03, 0.1, -0.16]
        self.end_point = [0.03, 0.1, -0.2]
        self.reset_point = [0.0, 0.1, -0.12]

        self.tolerance = 0.005  # 5 mm sphere radius

        # Default State
        self.trial_state = "IDLE"
        self.GREEN = (0.0, 1.0, 0.0)
        self.BLUE  = (0.0, 0.0, 1.0)
        self.RED   = (1.0, 0.0, 0.0)

        # Publishers
        self.state_pub = self.create_publisher(String, "/trial/state", 10)
        self.marker_pub = self.create_publisher(Marker, "/trial/tolerance_marker", 10)

        # Subscribers
        self.create_subscription(PoseStamped, "loop_center", self.loop_center_cb, 10)

        # Timer
        self.create_timer(1/100, self.publish_all)

        self.get_logger().info("StudyController initialized.")

    # -----------------------------------------------------------------------------
    def loop_center_cb(self, msg: PoseStamped):
        point = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

        new_state = self.compute_state(point) 

        if new_state != self.trial_state:
            self.trial_state = new_state
            self.get_logger().info(f"Trial state changed to: {self.trial_state}")
            self.publish_all()
    # -----------------------------------------------------------------------------
    def compute_state(self, point):

        if self.is_inside_tolerance(point, self.start_point) and self.trial_state == "IDLE":
            return "RUNNING1"
        elif self.is_inside_tolerance(point, self.intersection_point1) and self.trial_state == "RUNNING1":
            return "RUNNING2"
        elif self.is_inside_tolerance(point, self.intersection_point2) and self.trial_state == "RUNNING2":
            return "RUNNING3"
        elif self.is_inside_tolerance(point, self.intersection_point3) and self.trial_state == "RUNNING3":
            return "RUNNING4"
        elif self.is_inside_tolerance(point, self.intersection_point4) and self.trial_state == "RUNNING4":
            return "RUNNING5"
        elif self.is_inside_tolerance(point, self.end_point) and self.trial_state == "RUNNING5":
            return "FINISHED"
        elif self.is_inside_tolerance(point, self.reset_point) and self.trial_state == "FINISHED":
            return "IDLE"
        
        return self.trial_state  # No state change
    # -----------------------------------------------------------------------------
    def is_inside_tolerance(self, point, target):
        return all(abs(point[i] - target[i]) <= self.tolerance for i in range(3))

    # -----------------------------------------------------------------------------
    def publish_all(self):

        # Publish current trial state
        self.state_pub.publish(String(data=self.trial_state))

        # Publish all markers individually
        self.publish_marker(self.start_point, 0, self.color_start())
        self.publish_marker(self.intersection_point1, 1, self.color_i1())
        self.publish_marker(self.intersection_point2, 2, self.color_i2())
        self.publish_marker(self.intersection_point3, 3, self.color_i3())
        self.publish_marker(self.intersection_point4, 4, self.color_i4())
        self.publish_marker(self.end_point, 5, self.color_end())
        self.publish_marker(self.reset_point, 6, self.color_reset())
    # ------------------------------------------------------------------
    def publish_marker(self, point, marker_id, color):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trial_tolerance"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime.sec = 0  

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.tolerance * 2
        marker.scale.y = self.tolerance * 2
        marker.scale.z = self.tolerance * 2

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 0.6

        self.marker_pub.publish(marker)

    # -----------------------------------------------------------------------------    
    def color_start(self):
        if self.trial_state == "RUNNING1":
            return self.GREEN
        if self.trial_state == "IDLE":
            return self.BLUE
        return self.RED

    def color_i1(self):
        if self.trial_state == "RUNNING2":
            return self.GREEN
        if self.trial_state == "RUNNING1":
            return self.BLUE
        return self.RED

    def color_i2(self):
        if self.trial_state == "RUNNING3":
            return self.GREEN
        if self.trial_state == "RUNNING2":
            return self.BLUE
        return self.RED

    def color_i3(self):
        if self.trial_state == "RUNNING4":
            return self.GREEN
        if self.trial_state == "RUNNING3":
            return self.BLUE
        return self.RED

    def color_i4(self):
        if self.trial_state == "RUNNING5":
            return self.GREEN
        if self.trial_state == "RUNNING4":
            return self.BLUE
        return self.RED

    def color_end(self):
        if self.trial_state == "RUNNING5":
            return self.BLUE
        if self.trial_state == "FINISHED":
            return self.GREEN
        return self.RED

    def color_reset(self):
        if self.trial_state == "FINISHED":
            return self.BLUE
        if self.trial_state == "IDLE":
            return self.GREEN
        return self.RED


def main(args=None):
    rclpy.init(args=args)
    node = StudyStateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
