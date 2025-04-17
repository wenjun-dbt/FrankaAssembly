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


robotip_1 = "172.16.1.3"
robotip_0 = "172.16.0.3"
gripper_1 = franky.Gripper(robotip_1)
gripper_0 = franky.Gripper(robotip_0)

speed = 0.01  # [m/s]
force = 60.0  # [N]

#
# # Your code before the pause
print("Open the gripper?. Press Enter to continue...")
input()
print("The program has resumed.")
#

# gripper_1.open(speed)
# gripper_0.grasp(0.0,0.1,60)
