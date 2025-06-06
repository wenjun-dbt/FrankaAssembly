import math
from time import sleep

import numpy as np
import scipy
from scipy.spatial.transform import Rotation
from scipy.special import euler
import time
from compas_eve import Publisher
from compas_eve import Topic
from compas_eve.mqtt import MqttTransport
from franky import Robot, JointVelocityMotion, CartesianVelocityMotion, Duration, JointMotion, Twist
from franky import JointWaypointMotion, JointWaypoint, CartesianMotion, \
    CartesianWaypointMotion, CartesianWaypoint, Affine, Twist, RobotPose, ReferenceType, \
    CartesianState, JointState, RelativeDynamicsFactor
from franky import *
import json
import frankz
import franky
import numpy as np

from scipy.spatial.transform import Rotation
from franka_assembly import ASSEMBLY_DIR

robotip_0 = "172.16.0.3"
robotip_1 = "172.16.0.5"
robotips = [robotip_0, robotip_1]
# gripper_1 = franky.Gripper(robotip_1)
# gripper_0 = franky.Gripper(robotip_0)


with open(ASSEMBLY_DIR + "/traj/byd/traj_11.json", "r") as f:
    data = json.load(f)


def map_to_current(traj, current):
    gap = traj[0] - current
    new_traj = []
    for t in traj:
        new_traj.append(t - gap)
    return new_traj




def forward_kinematics(joints,robot_ip):
    waypoint_mat = np.array(frankz.fk(joints,robot_ip)).reshape(4,4).T
    print(waypoint_mat)
    rot_mat = waypoint_mat[:3,:3]
    trans_mat = waypoint_mat[:3,3]
    quat = Rotation.from_matrix(rot_mat).as_quat()
    return RobotPose(Affine(trans_mat, quat))


speed = 0.1 # [m/s]
force = 60.0  # [N]

velocity = 0.005
traj_factor = int(20)
frequency = 100
safe = True

# data.reverse()
home1 = data[0]
robot1 = Robot(robotip_1)
robot1.relative_dynamics_factor = RelativeDynamicsFactor(0.05, 0.05, 0.05)
home1_place_motion = JointWaypointMotion([JointWaypoint(home1)])
print(f"Robot_1 going to home position.")
robot1.move(home1_place_motion)
robotid = 1
robotip = robotips[int(robotid)]
print(f"Robot_1 going to motion planning.")

robot = Robot(robotip)
robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
new_traj = map_to_current(data, robot.current_joint_state.position)
status = frankz.run(new_traj, robotip, traj_factor, 500.0, safe, frequency)
