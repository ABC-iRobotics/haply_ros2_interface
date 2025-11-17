import crtk, dvrk

# create the ROS Abstraction Layer with the name of the node
ral = crtk.ral('dvrk_python_node')

# create a Python proxy for PSM1, name must match ROS namespace
p = dvrk.psm(ral, 'PSM1')

# wait and check until all the topics are connected
# default timeout is 5 seconds
ral.check_connections()

# spin to make sure subscribers callbacks are called
# this is required for ROS 2 and does nothing on ROS 1
# use for portability!
ral.spin()

# now you can home from Python
p.enable()
p.home()

# retrieve current info (numpy.array) with timestamps
p.measured_jp()
p.measured_jv()
p.measured_jf()

# retrieve PID desired position and effort computed with timestamps
p.setpoint_jp()
p.setpoint_jf()

# retrieve cartesian current and desired positions
# PyKDL.Frame with timestamps
p.measured_cp()
p.setpoint_cp()

# move in joint space
# move is absolute (SI units)

# move multiple joints
import numpy
p.move_jp(numpy.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0]))

# move in cartesian space
import PyKDL
# start position and timestamp (ignored using variable _)
goal, _ = p.setpoint_cp()
# move 5cm in z direction
goal.p[2] += 0.05
p.move_cp(goal).wait()

import math
# start position
goal, _ = p.setpoint_cp()
# rotate tool tip frame by 25 degrees
goal.M.DoRotX(math.pi * 0.4)
p.move_cp(goal).wait()