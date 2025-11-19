#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from haply_msgs.msg import HaplyState
from geometry_msgs.msg import PoseStamped
import crtk
import PyKDL # type: ignore
import math

class JawOps:
    def __init__(self, arm_ral, connection_timeout=10.0):
        self._ral = arm_ral.create_child("jaw")
        u = crtk.utils(self, self._ral, connection_timeout)
        u.add_measured_js()   # read jaw
        u.add_servo_jp()      
    
# class for using CRTK arm operations
class ArmOps:
    def __init__(self, ral, arm_name, connection_timeout=10.0):
        self._ral = ral.create_child(arm_name)
        u = crtk.utils(self, self._ral, connection_timeout)
        u.add_operating_state()
        u.add_setpoint_cp()     # read the set Cartesian pose of endeffector (Attention: not the actual pose!)
        u.add_servo_cp()        # set Cartesian pose of endeffector
        u.add_servo_jp()        

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

        # dsubscribe daVinci data
        self.daVinci_pose = None
        self.create_subscription(PoseStamped, '/PSM1/measured_cp', self.daVinci_measured_callback, 10)

        # subscribe Haply data
        self.create_subscription(HaplyState, "haply_state", self.haply_state_callback, 10)

        # initial gripper parameters
        self.gripper_status = 0
        self.last_button_a = False   
        self.button_calibration = False  

        self.scale_arm_movement = 0.25
        self.rot_scale = 0.5  
        self.calibrated_haply_position = None
        self.calibrated_haply_orientation = None  
        self.calibrated_daVinci_pose = None
        self.get_logger().info("haply_to_daVinci_bridge_node started")


    # callback function for daVinci data
    def daVinci_measured_callback(self, msg: PoseStamped):
        daVinci_position = msg.pose.position
        daVinci_orientation = msg.pose.orientation
        daVinci_orientation_pyKDL = PyKDL.Rotation.Quaternion(daVinci_orientation.x, daVinci_orientation.y, daVinci_orientation.z, daVinci_orientation.w)
        daVinci_position_pyKDL = PyKDL.Vector(daVinci_position.x, daVinci_position.y, daVinci_position.z)
        # store current daVinci pose
        self.daVinci_pose = PyKDL.Frame(daVinci_orientation_pyKDL, daVinci_position_pyKDL)
        

    # callback function for Haply data (position, velocity, orientation, buttons)
    def haply_state_callback(self, msg: HaplyState):
        # check for calibration
        if self.calibrated_haply_position is None or msg.buttons.b: 
            if msg.buttons.b:
                self.button_calibration = True
            self.set_reference_pose_arm(msg)
            return
        
        # compute actual difference between current and reference pose (calibrated origin)
        dx, dy, dz, droll, dpitch, dyaw = self.compute_pose_difference(msg.position, msg.quaternion, self.calibrated_haply_position, self.calibrated_haply_orientation)
        # send target pose to daVinci
        self.send_target_daVinci_pose(dx, dy, dz, droll, dpitch, dyaw)
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


    def set_reference_pose_arm(self, msg: HaplyState):
        if self.daVinci_pose is None:
            self.get_logger().warning("No daVinci pose available yet for calibration.")
            return

        # Now set the calibrated poses
        self.calibrated_daVinci_pose = PyKDL.Frame(self.daVinci_pose)
        # The current Haply position is set as an offset so that later movements are calculated relative to the daVinci start pose.
        self.calibrated_haply_position = msg.position  
        if self.button_calibration == False:
            self.calibrated_haply_orientation = msg.quaternion  


    # function to compute haply pose difference
    def compute_pose_difference(self, current_haply_position, current_haply_orientation, calibrated_haply_position, calibrated_haply_orientation):
        dx = (current_haply_position.x - calibrated_haply_position.x) * self.scale_arm_movement
        dy = (current_haply_position.y - calibrated_haply_position.y) * self.scale_arm_movement
        dz = (current_haply_position.z - calibrated_haply_position.z) * self.scale_arm_movement
        
        try:
            R_cur = PyKDL.Rotation.Quaternion(current_haply_orientation.x, current_haply_orientation.y, current_haply_orientation.z, current_haply_orientation.w)
            r_cur, p_cur, y_cur = R_cur.GetRPY()
        except Exception as e:
            self.get_logger().debug(f"Failed to compute current RPY: {e}")
            r_cur = p_cur = y_cur = 0.0

        try:
            R_cal = PyKDL.Rotation.Quaternion(calibrated_haply_orientation.x, calibrated_haply_orientation.y, calibrated_haply_orientation.z, calibrated_haply_orientation.w)
            r_cal, p_cal, y_cal = R_cal.GetRPY()
        except Exception:
            r_cal = p_cal = y_cal = 0.0

        # rotational deltas (scaled)
        droll = (r_cur - r_cal) * self.rot_scale
        dpitch = (p_cur - p_cal) * self.rot_scale
        dyaw = (y_cur - y_cal) * self.rot_scale
        self.get_logger().info(f"droll={droll}, dpitch={dpitch}, dyaw={dyaw}")

        return dx, dy, dz, droll, dpitch, dyaw


    # function to send target daVinci pose to daVinci via CRTK
    def send_target_daVinci_pose(self, dx, dy, dz, droll, dpitch, dyaw): 
        if self.calibrated_daVinci_pose is None:
            self.get_logger().warning("Calibrated daVinci pose is not set.")
            return  
        
        # calculate target position based on difference between current and reference position
        target_daVinci_pose = PyKDL.Frame(self.calibrated_daVinci_pose)
        target_daVinci_pose.p[0] -= dx
        target_daVinci_pose.p[1] += dy
        target_daVinci_pose.p[2] -= dz

        # get current orientation as RPY, add deltas, set new rotation
        calibrated_Rotation = target_daVinci_pose.M
        r_cal, p_cal, y_cal = calibrated_Rotation.GetRPY()
        
        # swap roll and pitch for daVinci
        new_Rotation = PyKDL.Rotation.RPY(r_cal - dpitch, p_cal - droll, y_cal - dyaw) 
        #new_Rotation = PyKDL.Rotation.RPY(r_cal + droll, p_cal + dpitch, y_cal + dyaw)
        target_daVinci_pose.M = new_Rotation
        
        # send target position to daVinci via CRTK
        try:
            self.arm.servo_cp(target_daVinci_pose)
            #self.get_logger().info(f"Sent target daVinci pose to position: x={target_daVinci_pose.p.x():.3f}, y={target_daVinci_pose.p.y():.3f}, z={target_daVinci_pose.p.z():.3f}")
        except Exception as e:
            self.get_logger().error(f"Error sending servo_cp: {e}")


    # function to convert quaternion to roll angle
    def _quaternion_to_roll(self, x, y, z, w):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
        return math.atan2(sinr_cosp, cosr_cosp)


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


