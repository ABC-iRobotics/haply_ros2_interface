#!/usr/bin/env python3
import rclpy

from haply_daVinci_teleop.bridge.haply_daVinci_bridge_node_jointcontrol import (
    HaplyToDaVinciBridge,
)


class HaplyToDaVinciBridgeDemo(HaplyToDaVinciBridge):
    """Demo bridge that keeps the PSM1 insertion/z joint fixed."""

    def __init__(self):
        super().__init__()
        self.get_logger().info(
            "Demo mode enabled: PSM1 z/insertion movement is locked."
        )

    def set_target_daVinci_joints(self, dx, dy, dz, droll, dpitch, dyaw):
        """Send target joints while ignoring Haply z translation."""
        if self.calibrated_daVinci_joints is None:
            self.get_logger().warning("Calibrated daVinci joints are not set.")
            return

        if self.daVinci_joints is None:
            self.get_logger().warning("Current daVinci joints are not available.")
            return

        target_joints = self.calibrated_daVinci_joints.copy()

        try:
            target_joints[self.daVinci_x_index] -= dx * 10.0
            target_joints[self.daVinci_y_index] += dy * 1.0
            target_joints[self.daVinci_z_index] -= dz * 1.0
            

            target_joints[self.daVinci_pitch_index] -= dyaw
            target_joints[self.daVinci_yaw_index] += droll
            target_joints[self.daVinci_roll_index] += dpitch

        except IndexError:
            self.get_logger().error(
                f"Joint index out of range, got len={len(target_joints)}"
            )
            return

        self.send_target_daVinci_joints(target_joints)


def main(args=None):
    rclpy.init(args=args)
    node = HaplyToDaVinciBridgeDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
