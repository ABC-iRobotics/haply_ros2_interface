#!/usr/bin/env python3
import time
import rclpy
import numpy as np
from rclpy.node import Node
from haply_msgs.msg import HaplyState
from sensor_msgs.msg import JointState
import crtk
import PyKDL # type: ignore
import math

""" 
This script defines a bridge node to map Haply device states to daVinci joint commands using CRTK. 
It subscribes to HaplyState and daVinci joint states, computes pose differences between current and calibrated poses, and sends joint commands.
It also enables clutching via Button B and gripper control via Button A of the Haply device.
"""

# class for using CRTK jaw operations
class JawOps:
    def __init__(self, arm_ral, connection_timeout=10.0):
        self._ral = arm_ral.create_child("jaw")
        u = crtk.utils(self, self._ral, connection_timeout)
        u.add_measured_js()   
        u.add_servo_jp()    
        u.add_move_jp()

# class for using CRTK arm operations
class ArmOps:
    def __init__(self, ral, arm_name, connection_timeout=10.0):
        self._ral = ral.create_child(arm_name)
        u = crtk.utils(self, self._ral, connection_timeout)
        u.add_operating_state()
        u.add_setpoint_cp()
        u.add_servo_cp()
        u.add_servo_jp()
        u.add_move_jp()

        self.jaw = JawOps(self._ral, connection_timeout)

    def ral(self):
        return self._ral

# main bridge node class
class HaplyToDaVinciBridge(Node):
    def __init__(self):
        super().__init__('haply_to_davinci_bridge')

        self.ral = crtk.ral("daVinci_init")
        self.arm = ArmOps(self.ral, "PSM1")

        try:
            self.arm.disable(5.0)
            time.sleep(10.0)
            self.arm.enable(5.0)
            self.arm.home(5.0)
            self.get_logger().info("Arm enabled and homed")
        except Exception as e:
            self.get_logger().error(f"Enable/Home failed: {e}")

        self.daVinci_joints = None          
        self.calibrated_daVinci_joints = None

        self.create_subscription(JointState, '/PSM1/measured_js',
                                 self.daVinci_measured_js_callback, 10)
        self.create_subscription(HaplyState, "haply_state",
                                self.haply_state_callback, 10)

        self.scale_arm_movement = 0.25
        self.rot_scale = 1.0

        # daVinci joint indices
        self.daVinci_x_index = 0
        self.daVinci_y_index = 1
        self.daVinci_z_index = 2
        self.daVinci_pitch_index = 3
        self.daVinci_yaw_index = 4
        self.daVinci_roll_index = 5

        # for continous yaw rotation
        self.last_yaw_raw = None
        self.unwrapped_yaw = 0.0
        self.calibrated_yaw_unwrapped = None

        # gripper control variables
        self.gripper_status = 0
        self.last_button_a = False     

        # calibration variables for haply
        self.calibrated_haply_position = None
        self.calibrated_haply_orientation = None

        # flag for simulation mode
        self.simulation_mode = True
        self.initial_position_sent = False

        self.get_logger().info("haply_to_daVinci_bridge_node started")


    def daVinci_measured_js_callback(self, msg: JointState):
        if not msg.position:
            return
        self.daVinci_joints = np.array(msg.position, dtype=float)


    def haply_state_callback(self, msg: HaplyState):
        """ Callback for HaplyState messages. """
        # If not calibrated, set the reference pose
        if self.calibrated_haply_position is None:
            self.set_reference_pose_arm(msg)
            return
        # If button B is pressed, reset the reference position (only position for intuitive control)
        if msg.buttons.b:
            self.set_reference_position_arm(msg)
            
        # Compute pose difference between current and calibrated pose    
        dx, dy, dz, droll, dpitch, dyaw = self.compute_pose_difference(
            msg.position,
            msg.quaternion,
            self.calibrated_haply_position,
            self.calibrated_haply_orientation
        )
        # Set new daVinci joint targets based on pose difference and publish
        self.set_target_daVinci_joints(dx, dy, dz, droll, dpitch, dyaw)
        # Control gripper based on button A
        self.control_jaw(msg)


    def control_jaw(self, msg: HaplyState):
        """ Control the gripper based on button A state. """
        try:
            current = msg.buttons.a
            if current and not self.last_button_a:
                if self.gripper_status == 0:
                    #self.arm.jaw.servo_jp(np.array([1.0]))   
                    self.arm.jaw.move_jp(np.array([1.0])) 
                    self.gripper_status = 1
                else:
                    #self.arm.jaw.servo_jp(np.array([0.00]))  
                    self.arm.jaw.move_jp(np.array([0.00]))    
                    self.gripper_status = 0

            self.last_button_a = current

        except Exception as e:
            self.get_logger().error(f"Grasp control failed: {e}")


    def set_reference_pose_arm(self, msg: HaplyState):
        """ Set the reference pose for the arm based on the current HaplyState. """
        if self.daVinci_joints is None:
            self.get_logger().warning("No daVinci joint state available yet for calibration.")
            return

        self.calibrated_daVinci_joints = self.daVinci_joints.copy()
        self.calibrated_haply_position = msg.position
        self.calibrated_haply_orientation = msg.quaternion

        # store unwrapped yaw at calibration time
        try:
            R_cal = PyKDL.Rotation.Quaternion(
                self.calibrated_haply_orientation.x,
                self.calibrated_haply_orientation.y,
                self.calibrated_haply_orientation.z,
                self.calibrated_haply_orientation.w
            )
            _, _, yaw_raw = R_cal.GetRPY()

            self.last_yaw_raw = None
            self.unwrapped_yaw = 0.0
            self.calibrated_yaw_unwrapped = self.unwrap_yaw(yaw_raw)

        except Exception as e:
            self.get_logger().warning(f"Failed to compute calibrated yaw: {e}")
            self.calibrated_yaw_unwrapped = 0.0

        self.get_logger().info(f"Haply calibrated to daVinci joints, e.g. insertion={self.calibrated_daVinci_joints[self.daVinci_z_index]:.3f}")


    def set_reference_position_arm(self, msg: HaplyState):
        """ Set the reference position for the arm based on the current HaplyState position only. """
        if self.daVinci_joints is None:
            self.get_logger().warning("No daVinci joint state available yet for calibration.")
            return

        self.calibrated_daVinci_joints[0:3] = self.daVinci_joints[0:3]
        self.calibrated_haply_position = msg.position


    def unwrap_yaw(self, yaw_raw):
        """ Unwrap yaw angle to handle continuous rotation. """
        if self.last_yaw_raw is None:
            self.last_yaw_raw = yaw_raw
            self.unwrapped_yaw = yaw_raw
            return self.unwrapped_yaw
        # Compute difference between current and last yaw
        delta = yaw_raw - self.last_yaw_raw
        # Adjust delta to be within the range [-pi, pi]
        if delta > math.pi:
            delta -= 2*math.pi
        elif delta < -math.pi:
            delta += 2*math.pi
        # Update unwrapped yaw
        self.unwrapped_yaw += delta
        self.last_yaw_raw = yaw_raw
        return self.unwrapped_yaw


    def angle_diff(self, current_angle, calibrated_angle):
        """ Compute the shortest angle difference between two angles. """
        angle_difference = current_angle - calibrated_angle
        return ((angle_difference + math.pi) % (2 * math.pi) - math.pi)
    

    def compute_pose_difference(self, current_haply_position, current_haply_orientation, calibrated_haply_position, calibrated_haply_orientation):
        """ Compute the pose difference between current and calibrated Haply poses. """
        difference_x = (current_haply_position.x - calibrated_haply_position.x) * self.scale_arm_movement
        difference_y = (current_haply_position.y - calibrated_haply_position.y) * self.scale_arm_movement
        difference_z = (current_haply_position.z - calibrated_haply_position.z) * self.scale_arm_movement

        try:
            R_cur = PyKDL.Rotation.Quaternion(
                current_haply_orientation.x,
                current_haply_orientation.y,
                current_haply_orientation.z,
                current_haply_orientation.w
            )
            current_roll, current_pitch, current_yaw_raw = R_cur.GetRPY()
            current_yaw = self.unwrap_yaw(current_yaw_raw)

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
            calibrated_roll, calibrated_pitch, calibrated_yaw_raw = R_cal.GetRPY()
        except Exception:
            calibrated_roll = calibrated_pitch = calibrated_yaw_raw = 0.0

        difference_roll  = self.angle_diff(current_roll,  calibrated_roll)  * self.rot_scale
        difference_pitch = self.angle_diff(current_pitch, calibrated_pitch) * self.rot_scale
        # Handle yaw unwrapping and difference calculation
        if self.calibrated_yaw_unwrapped is None:
            self.calibrated_yaw_unwrapped = current_yaw

        difference_yaw = (current_yaw - self.calibrated_yaw_unwrapped) * self.rot_scale
        return difference_x, difference_y, difference_z, difference_roll, difference_pitch, difference_yaw


    def set_target_daVinci_joints(self, dx, dy, dz, droll, dpitch, dyaw):
        """ Send target daVinci joints based on pose differences. """
        if self.calibrated_daVinci_joints is None:
            self.get_logger().warning("Calibrated daVinci joints are not set.")
            return

        if self.daVinci_joints is None:
            self.get_logger().warning("Current daVinci joints are not available.")
            return

        target_joints = self.calibrated_daVinci_joints.copy()

        try:
            target_joints[self.daVinci_x_index] -= dx * 10.0  
            target_joints[self.daVinci_y_index] += dy * 10.0
            target_joints[self.daVinci_z_index] -= dz * 1.0

            target_joints[self.daVinci_pitch_index] -= dyaw
            target_joints[self.daVinci_yaw_index]   += droll
            target_joints[self.daVinci_roll_index]  += dpitch

        except IndexError:
            self.get_logger().error(
                f"Joint index out of range, got len={len(target_joints)}"
            )
            return

        self.send_target_daVinci_joints(target_joints)


    def send_target_daVinci_joints(self, target_joints):
        """ Send the target joint positions to the daVinci arm via CRTK. """
        try:
            #self.arm.servo_jp(target_joints)
            self.arm.move_jp(target_joints) # use move_jp for smoother motion
        except Exception as e:
            self.get_logger().error(f"Error sending pose: {e}")


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
