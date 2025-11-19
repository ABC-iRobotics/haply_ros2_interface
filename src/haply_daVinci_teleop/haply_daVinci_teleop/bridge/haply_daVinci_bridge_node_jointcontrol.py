#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from haply_msgs.msg import HaplyState
from sensor_msgs.msg import JointState
import crtk
import PyKDL # type: ignore
import math

# class for using CRTK jaw operations
class JawOps:
    def __init__(self, arm_ral, connection_timeout=10.0):
        self._ral = arm_ral.create_child("jaw")
        u = crtk.utils(self, self._ral, connection_timeout)
        u.add_measured_js()   
        u.add_servo_jp()      

# class for using CRTK arm operations
class ArmOps:
    def __init__(self, ral, arm_name, connection_timeout=10.0):
        self._ral = ral.create_child(arm_name)
        u = crtk.utils(self, self._ral, connection_timeout)
        u.add_operating_state()
        u.add_setpoint_cp()   # read the set Cartesian pose of endeffector (Attention: not the actual pose!)
        u.add_servo_cp()      # set Cartesian pose of endeffector
        u.add_servo_jp()      # set joint positions

        # Create a child RAL for the jaw
        self.jaw = JawOps(self._ral, connection_timeout)

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

        # dVRK joint state (reference)
        self.daVinci_joints = None          
        self.calibrated_daVinci_joints = None

        # Subscriber for daVinci joint data
        self.create_subscription(
            JointState,
            '/PSM1/measured_js',
            self.daVinci_measured_js_callback,
            10
        )

        # Subscribe for Haply data
        self.create_subscription(
            HaplyState,
            "haply_state",
            self.haply_state_callback,
            10
        )

        # Scale Haply Movements
        self.scale_arm_movement = 0.25   # position scale
        self.rot_scale = 1.0             # orientation scale

        # joint indexes
        self.outer_yaw_index = 0
        self.outer_pitch_index = 1
        self.insertion_index = 2
        self.wrist_pitch_index = 3
        self.wrist_yaw_index = 4
        self.wrist_roll_index = 5
        
        # initial gripper parameters
        self.gripper_status = 0
        self.last_button_a = False     

        # initial calibration parameters
        self.calibrated_haply_position = None
        self.calibrated_haply_orientation = None

        self.get_logger().info("haply_to_daVinci_bridge_node started")


    # ---------- Callbacks ----------

    def daVinci_measured_js_callback(self, msg: JointState):
        """ Read daVinci joint states """
        if not msg.position:
            return
        self.daVinci_joints = np.array(msg.position, dtype=float)


    def haply_state_callback(self, msg: HaplyState):
        """ Update daVinci joint states """
        # check for calibration
        if self.calibrated_haply_position is None or msg.buttons.b:
            self.set_reference_pose_arm(msg)
            return

        # compute difference between current and reference pose (calibrated origin)
        dx, dy, dz, droll, dpitch, dyaw = self.compute_pose_difference(
            msg.position,
            msg.quaternion,
            self.calibrated_haply_position,
            self.calibrated_haply_orientation
        )

        # send target joints to daVinci
        self.send_target_daVinci_joints(dx, dy, dz, droll, dpitch, dyaw)
        self.control_jaw(msg)

    # ---------- Gripper ----------

    def control_jaw(self, msg: HaplyState):
        """ Control Gripper """
        try:
            current = msg.buttons.a
            if current and not self.last_button_a:
                # Toggle gripper state
                if self.gripper_status == 0:
                    # open
                    self.arm.jaw.servo_jp(np.array([1.0]))   
                    self.gripper_status = 1
                else:
                    # close
                    self.arm.jaw.servo_jp(np.array([0.00]))   
                    self.gripper_status = 0

            # Update last state
            self.last_button_a = current

        except Exception as e:
            self.get_logger().error(f"Grasp control failed: {e}")

    # ---------- Calibration ----------

    def set_reference_pose_arm(self, msg: HaplyState):
        """ Calibrate daVinci joint positions as reference """
        if self.daVinci_joints is None:
            self.get_logger().warning("No daVinci joint state available yet for calibration.")
            return

        # store current daVinci joint states
        self.calibrated_daVinci_joints = self.daVinci_joints.copy()

        # store haply reference pose
        self.calibrated_haply_position = msg.position
        self.calibrated_haply_orientation = msg.quaternion

        self.get_logger().info(
            f"Haply calibrated to daVinci joints, e.g. insertion={self.calibrated_daVinci_joints[self.insertion_index]:.3f}"
        )

    # ---------- Calculate Control Delta ----------

    def compute_pose_difference(self, current_haply_position, current_haply_orientation, calibrated_haply_position, calibrated_haply_orientation):
        """ Compute current difference between daVinci and Haply """

        dx = (current_haply_position.x - calibrated_haply_position.x) * self.scale_arm_movement
        dy = (current_haply_position.y - calibrated_haply_position.y) * self.scale_arm_movement
        dz = (current_haply_position.z - calibrated_haply_position.z) * self.scale_arm_movement

        try:
            R_cur = PyKDL.Rotation.Quaternion(
                current_haply_orientation.x,
                current_haply_orientation.y,
                current_haply_orientation.z,
                current_haply_orientation.w
            )
            current_roll, current_pitch, current_yaw = R_cur.GetRPY()
            #self.get_logger().info(f"Current Roll:{current_roll}, Current Pitch:{current_pitch}, Current Yaw:{current_yaw}")
            self.get_logger().info(f"Current Yaw:{current_yaw}")
        except Exception as e:
            self.get_logger().debug(f"Failed to compute current RPY: {e}")
            current_roll = current_pitch = current_yaw = 0.0

        try:
            R_cal = PyKDL.Rotation.Quaternion(
                calibrated_haply_orientation.x,
                calibrated_haply_orientation.y,
                calibrated_haply_orientation.z,
                calibrated_haply_orientation.w
            )
            calibrated_roll, calibrated_pitch, calibrated_yaw = R_cal.GetRPY()
        except Exception:
            calibrated_roll = calibrated_pitch = calibrated_yaw = 0.0

        # rotational deltas (scaled)
        droll = (current_roll - calibrated_roll) * self.rot_scale
        dpitch = (current_pitch - calibrated_pitch) * self.rot_scale
        dyaw = (current_yaw - calibrated_yaw) * self.rot_scale

        return dx, dy, dz, droll, dpitch, dyaw

    # ---------- send target joint angles to daVinci ----------

    def send_target_daVinci_joints(self, dx, dy, dz, droll, dpitch, dyaw):
        """ Send target joint angles to daVinci """

        if self.calibrated_daVinci_joints is None:
            self.get_logger().warning("Calibrated daVinci joints are not set.")
            return

        if self.daVinci_joints is None:
            self.get_logger().warning("Current daVinci joints are not available.")
            return

        target_joints = self.calibrated_daVinci_joints.copy()

        try:
            target_joints[self.outer_yaw_index] -= dx * 10.0  
            target_joints[self.outer_pitch_index] += dy * 10.0
            target_joints[self.insertion_index] -= dz * 1.0

            target_joints[self.wrist_pitch_index] -= dyaw
            target_joints[self.wrist_yaw_index]   += droll
            target_joints[self.wrist_roll_index]  += dpitch

        except IndexError:
            self.get_logger().error(
                f"Joint index out of range, got len={len(target_joints)}"
            )
            return

        try:
            self.arm.servo_jp(target_joints)
        except Exception as e:
            self.get_logger().error(f"Error sending servo_jp: {e}")


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



