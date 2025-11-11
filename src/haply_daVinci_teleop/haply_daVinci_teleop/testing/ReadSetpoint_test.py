#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class DaVinciSetpointReader(Node):
    """Liest den aktuellen Setpoint des PSM1 über das ROS-Topic aus."""

    def __init__(self):
        super().__init__("psm1_setpoint_reader")

        # Topic kann je nach System "/PSM1/setpoint_cp" oder "/PSM1/local/setpoint_cp" heißen
        topic_name = "/PSM1/local/setpoint_cp"

        # Subscriber anlegen
        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.setpoint_callback,
            10
        )
        self.get_logger().info(f"Listening to: {topic_name}")

    def setpoint_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.get_logger().info(
            f"Setpoint -> Position [x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}] | "
            f"Orientation [qx={ori.x:.3f}, qy={ori.y:.3f}, qz={ori.z:.3f}, qw={ori.w:.3f}]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DaVinciSetpointReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
