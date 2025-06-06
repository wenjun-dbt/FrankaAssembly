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
import time
from franka_assembly import ASSEMBLY_DIR
import threading

robotip_1 = "172.16.0.5"
robotip_0 = "172.16.0.3"
gripper_1 = franky.Gripper(robotip_1)
gripper_0 = franky.Gripper(robotip_0)
speed = 0.1  # [m/s]
force = 60.0  # [N]
with open(ASSEMBLY_DIR + "/traj/byd/b.json", "r") as f:
    data = json.load(f)

def home(robot):
    print(f"Thread {robot} 开始执行...")

    try:
        if robot == 0:
            gripper_0.open(speed)
            home0 = data[2]["joint_states"][0]
            robot0 = Robot(robotip_0)
            robot0.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.03, 0.03)
            home0_place_motion = JointWaypointMotion([JointWaypoint(home0)])
            print(f"Robot_0 going to home position.")
            robot0.move(home0_place_motion)
        elif robot == 1:
            gripper_1.open(speed)
            home1 = data[2]["joint_states"][0]
            robot1 = Robot(robotip_1)
            robot1.relative_dynamics_factor = RelativeDynamicsFactor(0.03,0.03, 0.03)
            home1_place_motion = JointWaypointMotion([JointWaypoint(home1)])
            print(f"Robot_1 going to home position.")
            robot1.move(home1_place_motion)
    except Exception as e:
        print(f"Robot {robot} 控制失败: {e}")
        raise


thread0 = threading.Thread(target=home,args=(0,))
thread1 = threading.Thread(target=home,args=(1,))

thread0.start()
thread1.start()

thread0.join()
thread1.join()
