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


robotip = "172.16.0.3"
gripper = franky.Gripper(robotip)

speed = 0.01  # [m/s]
force = 60.0  # [N]

# Move the fingers to a specific width (5cm)
#success = gripper.move(0.08, speed)

# Release the object

robot = Robot(robotip)
robot.relative_dynamics_factor = RelativeDynamicsFactor(0.01, 0.01, 0.01)

with open("Rob_left/saved_plan_print_cali_fast.json", "r") as f:
    data = json.load(f)

print("Press Enter to continue...")
input()
print("The program has resumed.")

pause = True
traj_factor = 10
for i in range(0,11):
    print(f"_________________STEP{i}________________")
    offset = i * 9
    # traj_safetransfer = data[offset]["joint_states"]
    traj_safepick = data[offset + 2]["joint_states"]
    waypoint_pick = np.array(data[offset + 3]["joint_states"]).reshape(-1)
    waypoint_pickleave = np.array(data[offset + 5]["joint_states"]).reshape(-1)
    traj_picktoplace = data[offset + 6]["joint_states"]
    waypoint_place = np.array(data[offset + 7]["joint_states"]).reshape(-1)

    waypoint_pick_motion = JointWaypointMotion([JointWaypoint(waypoint_pick)])
    waypoint_pickleave_motion = JointWaypointMotion([JointWaypoint(waypoint_pickleave)])
    waypoint_place_motion = JointWaypointMotion([JointWaypoint(waypoint_place)])

    #"""


    gripper.open(speed)

    frankz.run(traj_safepick, robotip, traj_factor, 0.1)

    # # Your code before the pause
    print("The safe position is reached. Press Enter to continue...")
    input()
    print("The program has resumed.")

    robot = Robot(robotip)
    robot.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.03, 0.03)
    robot.move(waypoint_pick_motion)

    # # Your code before the pause
    #print("The grasp position is reached. Press Enter to continue...")
    #input()
    #print("The program has resumed.")

    # # # Grasp an object of unknown width
    #
    success = gripper.grasp(0.0, speed, force, epsilon_outer=1.0)
    # # # Get the width of the grasped object
    width = gripper.width
    print(width)

    robot = Robot(robotip)
    robot.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.03, 0.03)
    target0 = Affine([0.00000, 0.000000, -0.0200])
    motion_forward = CartesianMotion(target0, reference_type=ReferenceType.Relative)
    robot.move(motion_forward)

    robot = Robot(robotip)
    robot.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.03, 0.03)
    robot.move(waypoint_pickleave_motion)

    frankz.run(traj_picktoplace, robotip,traj_factor,0.05)

    # # Your code before the pause
    print("The safe position is reached. Press Enter to continue...")
    input()
    print("The program has resumed.")


    robot = Robot(robotip)
    robot.relative_dynamics_factor = RelativeDynamicsFactor(0.01, 0.01, 0.01)
    robot.move(waypoint_place_motion)

    #cartesian_state = robot.current_cartesian_state
    #print(cartesian_state)
    #print(cartesian_place)
    #robot.move(cartesian_place_motion)

    # # Your code before the pause
    print("The grasp position is reached. Press Enter to continue...")
    input()
    print("The program has resumed.")
    # # # Grasp an object of unknown width
    #
    
    gripper.open(speed)





