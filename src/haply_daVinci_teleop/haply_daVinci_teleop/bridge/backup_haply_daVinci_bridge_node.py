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
        u.add_setpoint_cp()     # set joint angles arm
        u.add_servo_cp()        # read joint angles arm

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


        # daVinci data
        self._local_setpoint = None
        self.create_subscription(PoseStamped, '/PSM1/local/setpoint_cp', self._local_setpoint_cb, 10)

        # Haply data
        self.create_subscription(HaplyState, "haply_state", self.haply_callback, 10)

        # Scaling factor for movement
        self.scale = 0.25
        # initialise 
        self.last_pos = None
        self.last_quat = None  
        self.get_logger().info("haply_to_daVinci_bridge_node started")

    # callback function for daVinci data
    def _local_setpoint_cb(self, msg: PoseStamped):
        # store daVinci position
        p = msg.pose.position
        # store daVinci orientation
        q = msg.pose.orientation
        rot = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        vec = PyKDL.Vector(p.x, p.y, p.z)
        # store current daVinci pose
        self._local_setpoint = PyKDL.Frame(rot, vec)

    # callback function for Haply data
    def haply_callback(self, msg: HaplyState):

        # set reference positions on first call
        if self.last_pos is None:
            self.last_pos = msg.position  
            try:
                current_cp, ts = self.arm.setpoint_cp()
            except Exception:
                current_cp, ts = None, 0.0
            if current_cp is None and self._local_setpoint is not None:
                current_cp = self._local_setpoint
            if current_cp is not None:
                # save reference position (calibrated origin)
                self.start_cp = PyKDL.Frame(current_cp)
            self.get_logger().info("Reference positions set.")
            return

        # actual difference between current and reference position (calibrated origin)
        dx = (msg.position.x - self.last_pos.x) * self.scale
        dy = (msg.position.y - self.last_pos.y) * self.scale
        dz = (msg.position.z - self.last_pos.z) * self.scale

        # calculate target position based on difference between current and reference position
        target = PyKDL.Frame(self.start_cp)
        target.p[0] += dx
        target.p[1] += dy
        target.p[2] += dz

        # send target position to CRTK
        try:
            self.arm.servo_cp(target)
        except Exception as e:
            self.get_logger().error(f"Error sending servo_cp: {e}")

        """
        # recalibrate reference position if button A is pressed
        if msg.buttons.a:
            self.last_pos = msg.position
            current_cp, ts = self.arm.setpoint_cp()
            self.start_cp = PyKDL.Frame(current_cp)
            self.get_logger().info("Recalibrated reference position")
            return
        """
        
   
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

