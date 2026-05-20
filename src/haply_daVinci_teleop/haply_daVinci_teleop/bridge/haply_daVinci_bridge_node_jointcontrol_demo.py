#!/usr/bin/env python3
import rclpy

from haply_daVinci_teleop.bridge.haply_daVinci_bridge_node_jointcontrol import (
    HaplyToDaVinciBridge,
)
from haply_daVinci_teleop.coordinate_transforms import (
    haply_rotation_delta_to_psm,
    haply_translation_delta_to_psm,
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
            psm_dx, psm_dy, psm_dz = haply_translation_delta_to_psm(dx, dy, dz)
            psm_droll, psm_dpitch, psm_dyaw = haply_rotation_delta_to_psm(
                droll, dpitch, dyaw
            )

            target_joints[self.daVinci_x_index] += psm_dx * 10.0
            target_joints[self.daVinci_y_index] += psm_dy * 1.0
            target_joints[self.daVinci_z_index] += psm_dz * 1.0

            target_joints[self.daVinci_roll_index] += psm_droll
            target_joints[self.daVinci_pitch_index] += psm_dpitch
            target_joints[self.daVinci_yaw_index] += psm_dyaw

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
