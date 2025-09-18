#!/usr/bin/env python3

import sys
import time
import math
import argparse
import crtk
import PyKDL
import numpy

class CircleMotion:
    def __init__(self, ral, arm_name, period=0.01):
        self.ral = ral
        self.arm_name = arm_name
        self.period = period

        # Arm device inicializálása
        self.arm = crtk.utils(self, ral.create_child(arm_name))
        self.arm.add_operating_state()
        self.arm.add_setpoint_js()
        self.arm.add_setpoint_cp()
        self.arm.add_measured_js()
        self.arm.add_move_jp()
        self.arm.add_servo_cp()

    def home(self):
        print('> Enabling...')
        if not self.arm.enable(10):
            print('! Failed to enable')
            self.ral.shutdown()

        print('> Homing...')
        if not self.arm.home(10):
            print('! Failed to home')
            self.ral.shutdown()

        # biztosítsuk, hogy kartéziánus módban biztonságos helyzetben legyen
        print('> Preparing Cartesian start pose')
        jp, ts = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        goal[0] = 0.0
        goal[1] = 0.0
        goal[2] = 0.12  # kicsit előretolva
        goal[3] = 0.0
        self.arm.move_jp(goal).wait()

    def circle_motion(self, duration=10.0, radius=0.01, z=-0.12):
        print(f'> Starting circle motion for {duration} sec')
        cp, _ = self.arm.setpoint_cp()
        center = PyKDL.Frame(cp.M, PyKDL.Vector(cp.p[0], cp.p[1], z))

        sleep_rate = self.ral.create_rate(1.0 / self.period)
        steps = int(duration / self.period)
        for i in range(steps):
            angle = 2.0 * math.pi * (i / steps)
            goal = PyKDL.Frame(
                cp.M,
                PyKDL.Vector(
                    center.p[0] + radius * math.cos(angle),
                    center.p[1] + radius * math.sin(angle),
                    center.p[2]
                )
            )
            self.arm.servo_cp(goal)
            sleep_rate.sleep()
        print('< Circle motion complete')

    def run(self):
        self.home()
        self.circle_motion()


if __name__ == '__main__':
    argv = crtk.ral.parse_argv(sys.argv[1:])
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, default='PSM1',
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help='Arm name (default=PSM1)')
    parser.add_argument('-p', '--period', type=float, default=0.01,
                        help='Loop period (default=0.01s)')
    args = parser.parse_args(argv)

    ral = crtk.ral('psm1_circle')
    app = CircleMotion(ral, args.arm, args.period)
    ral.spin_and_execute(app.run)
