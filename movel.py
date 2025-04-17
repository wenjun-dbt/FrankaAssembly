import math

import numpy as np
import scipy
from scipy.spatial.transform import Rotation
from franky import Robot, JointVelocityMotion, CartesianVelocityMotion, Duration, JointMotion, Twist

from franky import JointWaypointMotion, JointWaypoint, CartesianMotion, \
    CartesianWaypointMotion, CartesianWaypoint, Affine, Twist, RobotPose, ReferenceType, \
    CartesianState, JointState, RelativeDynamicsFactor
import json
import frankz
import franky


robotip = "172.16.1.3"
gripper = franky.Gripper(robotip)

speed = 0.01  # [m/s]
force = 60.0  # [N]

#
# # Your code before the pause
print("The safe position is reached. Press Enter to continue...")
# #
# # # Pause until the user presses Enter
input()
# #
# # # Your code after the pause
print("The program has resumed.")
#
robot = Robot(robotip)
cartesian_state = robot.current_cartesian_state
robot.relative_dynamics_factor = RelativeDynamicsFactor(0.05, 0.05, 0.05)
# target0 = Affine([-0.007000, 0.000000, 0.00])
target0 = Affine([0.000, 0.000000, -0.0800])
motion_forward = CartesianMotion(target0, reference_type=ReferenceType.Relative)
robot.move(motion_forward)
cartesian_state = robot.current_cartesian_state
print(cartesian_state)


# # Your code before the pause
print("The safe position is reached. Press Enter to continue...")
# #
# # # Pause until the user presses Enter
input()
# #
# # # Your code after the pause
print("The program has resumed.")
gripper.open(speed)

#success = gripper.grasp(0.0, speed, force, epsilon_outer=1.0)