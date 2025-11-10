#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from haply_msgs.msg import HaplyState
from geometry_msgs.msg import PoseStamped
import crtk
import time
import PyKDL
import math

# class for using CRTK arm operations
class ArmOps:
    def __init__(self, ral, arm_name, connection_timeout=10.0):
        self._ral = ral.create_child(arm_name)
        u = crtk.utils(self, self._ral, connection_timeout)
        u.add_operating_state()
        u.add_setpoint_cp()     # read the set Cartesian pose of endeffector (Attention: not the actual pose!)
        u.add_servo_cp()        # set Cartesian pose of endeffector

    def ral(self):
        return self._ral

class HaplyToDaVinciBridge(Node):
    def __init__(self):
        super().__init__('haply_to_davinci_bridge')

        # Connection to CRTK
        self.ral = crtk.ral("daVinci_init")
        self.arm = ArmOps(self.ral, "PSM1")

        try:
            self.arm.enable(5.0)
            self.arm.home(5.0)
            self.get_logger().info("Arm enabled and homed")
        except Exception as e:
            self.get_logger().error(f"Enable/Home failed: {e}")

        # dsubscribe daVinci data
        self.daVinci_pose = None
        self.create_subscription(PoseStamped, '/PSM1/local/setpoint_cp', self.daVinci_setpoint_callback, 10)

        # subscribe Haply data
        self.create_subscription(HaplyState, "haply_state", self.haply_state_callback, 10)

        self.scale_arm_movement = 0.25
        self.calibrated_haply_position = None
        self.calibrated_haply_orientation = None  
        self.get_logger().info("haply_to_daVinci_bridge_node started")

    # callback function for daVinci data
    def daVinci_setpoint_callback(self, msg: PoseStamped):
        # store daVinci position
        daVinci_position = msg.pose.position
        # store daVinci orientation
        daVinci_orientation = msg.pose.orientation
        daVinci_orientation_pyKDL = PyKDL.Rotation.Quaternion(daVinci_orientation.x, daVinci_orientation.y, daVinci_orientation.z, daVinci_orientation.w)
        daVinci_position_pyKDL = PyKDL.Vector(daVinci_position.x, daVinci_position.y, daVinci_position.z)
        # store current daVinci pose
        self.daVinci_pose = PyKDL.Frame(daVinci_orientation_pyKDL, daVinci_position_pyKDL)

    # callback function for Haply data (position, velocity, orientation, buttons)
    def haply_state_callback(self, msg: HaplyState):

        # set reference positions on first call
        if self.calibrated_haply_position is None:
            self.calibrated_haply_position = msg.position  
            try:
                daVinci_arm_setpoint, ts = self.arm.setpoint_cp()
            except Exception:
                daVinci_arm_setpoint, ts = None, 0.0
            if daVinci_arm_setpoint is None and self.daVinci_pose is not None:
                daVinci_arm_setpoint = self.daVinci_pose
            if daVinci_arm_setpoint is not None:
                # save reference position (calibrated origin)
                self.calibrated_daVinci_pose = PyKDL.Frame(daVinci_arm_setpoint)
            self.get_logger().info("Reference positions set.")
            return

        # compute actual difference between current and reference position (calibrated origin)
        dx, dy, dz = self.compute_position_difference(msg.position, self.calibrated_haply_position)
        # send target pose to daVinci
        self.send_target_daVinci_pose(dx, dy, dz)

    # function to compute haply position difference
    def compute_position_difference(self, current_haply_position, calibrated_haply_position):
        dx = (current_haply_position.x - calibrated_haply_position.x) * self.scale_arm_movement
        dy = (current_haply_position.y - calibrated_haply_position.y) * self.scale_arm_movement
        dz = (current_haply_position.z - calibrated_haply_position.z) * self.scale_arm_movement
        return dx, dy, dz   

    # function to send target daVinci pose to daVinci via CRTK
    def send_target_daVinci_pose(self, dx, dy, dz): 
        if self.calibrated_daVinci_pose is None:
            self.get_logger().warning("Calibrated daVinci pose is not set.")
            return  
        
        # calculate target position based on difference between current and reference position
        target_daVinci_pose = PyKDL.Frame(self.calibrated_daVinci_pose)
        target_daVinci_pose.p[0] += dx
        target_daVinci_pose.p[1] += dy
        target_daVinci_pose.p[2] += dz

        # send target position to daVinci via CRTK
        try:
            self.arm.servo_cp(target_daVinci_pose)
        except Exception as e:
            self.get_logger().error(f"Error sending servo_cp: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HaplyToDaVinciBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


