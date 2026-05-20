#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

from haply_daVinci_teleop.data_evaluation.study_controller import StudyStateController


class StudyStateControllerDemo(StudyStateController):
    """Study controller for the short three-point demo wire."""

    def __init__(self):
        super().__init__()

        self.start_point = [0.0, 0.0, -0.15]
        self.middle_point = [0.05, 0.0, -0.15]
        self.end_point = [0.1, 0.0, -0.2]
        self.reset_point = [0.0, 0.0, -0.2]

        self.get_logger().info(
            "Demo mode enabled: using start, middle, end, and reset points."
        )

    def compute_state(self, point):
        if (
            self.is_inside_tolerance(point, self.start_point)
            and self.trial_state == "IDLE"
        ):
            return "RUNNING1"
        if (
            self.is_inside_tolerance(point, self.middle_point)
            and self.trial_state == "RUNNING1"
        ):
            return "RUNNING2"
        if (
            self.is_inside_tolerance(point, self.end_point)
            and self.trial_state == "RUNNING2"
        ):
            return "FINISHED"
        if (
            self.is_inside_tolerance(point, self.reset_point)
            and self.trial_state == "FINISHED"
        ):
            return "IDLE"

        return self.trial_state

    def publish_all(self):
        self.state_pub.publish(self._state_msg())
        self.publish_marker(self.start_point, 0, self.color_start())
        self.publish_marker(self.middle_point, 1, self.color_middle())
        self.publish_marker(self.end_point, 2, self.color_end_demo())
        self.publish_marker(self.reset_point, 3, self.color_reset())

    def _state_msg(self):
        return String(data=self.trial_state)

    def color_start(self):
        if self.trial_state == "RUNNING1":
            return self.GREEN
        if self.trial_state == "IDLE":
            return self.BLUE
        return self.RED

    def color_middle(self):
        if self.trial_state == "RUNNING2":
            return self.GREEN
        if self.trial_state == "RUNNING1":
            return self.BLUE
        return self.RED

    def color_end_demo(self):
        if self.trial_state == "RUNNING2":
            return self.BLUE
        if self.trial_state == "FINISHED":
            return self.GREEN
        return self.RED


def main(args=None):
    rclpy.init(args=args)
    node = StudyStateControllerDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
