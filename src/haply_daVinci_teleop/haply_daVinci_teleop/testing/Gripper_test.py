import rclpy
from rclpy.node import Node
import PyKDL
import crtk
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
    
class SimpleDaVinciControl(Node):
    def __init__(self):
        super().__init__('simple_davinci_control')

        # CRTK initialisieren und Arm laden (z. B. PSM1)
        self.ral = crtk.ral("daVinci_control")
        self.arm = ArmOps(self.ral, "PSM1")

        # Arm aktivieren und ggf. homing durchführen
        try:
            self.arm.enable(5.0)
            self.arm.home(5.0)
            self.get_logger().info("Arm enabled and homed")
        except Exception as e:
            self.get_logger().error(f"Enable/Home failed: {e}")

        # Nach kurzer Zeit feste Pose senden
        self.timer = self.create_timer(2.0, self.send_fixed_pose)

    def send_fixed_pose(self):
        """Setzt den PSM auf eine feste Pose im Raum."""
        # --- feste Position (x,y,z) in Metern ---
        x, y, z = 0.00, 0.00, -0.12

        # --- feste Orientierung (Roll, Pitch, Yaw) in Radiant ---
        roll, pitch, yaw = 0.0, math.pi/8, 0.0  
        R = PyKDL.Rotation.RPY(roll, pitch, yaw)

        # --- PyKDL Frame zusammensetzen ---
        frame = PyKDL.Frame(R, PyKDL.Vector(x, y, z))

        try:
            self.arm.servo_cp(frame)
            self.get_logger().info(f"Sent fixed pose: ({x:.3f}, {y:.3f}, {z:.3f})")
        except Exception as e:
            self.get_logger().error(f"servo_cp failed: {e}")

def main():
    rclpy.init()
    node = SimpleDaVinciControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
