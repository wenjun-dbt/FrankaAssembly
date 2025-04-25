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

robotip_0 = "172.16.0.3"
robotip_1 = "172.16.1.3"
robotips = [robotip_0, robotip_1]
# gripper_1 = franky.Gripper(robotip_1)
# gripper_0 = franky.Gripper(robotip_0)

def send_capture_message():
    topic = Topic("/compas_eve/zivid/")
    tx = MqttTransport("broker.emqx.io")
    publisher = Publisher(topic, transport=tx)
    msg = dict(text=f"Hello world")
    print(f"Publishing message: {msg}")
    publisher.publish(msg)
    time.sleep(2)

def home(robot):
    if robot == 0:
        home0 = data[10]["joint_states"][0]
        robot0 = Robot(robotip_0)
        robot0.relative_dynamics_factor = RelativeDynamicsFactor(0.3, 0.3, 0.3)
        home0_place_motion = JointWaypointMotion([JointWaypoint(home0)])
        print(f"Robot_0 going to home position.")
        robot0.move(home0_place_motion)
    elif robot == 1:
        home1 = data[2]["joint_states"][0]
        robot1 = Robot(robotip_1)
        robot1.relative_dynamics_factor = RelativeDynamicsFactor(0.3, 0.3, 0.3)
        home1_place_motion = JointWaypointMotion([JointWaypoint(home1)])
        print(f"Robot_1 going to home position.")
        robot1.move(home1_place_motion)


def map_to_current(traj, current):
    gap = traj[0] - current
    new_traj = []
    for t in traj:
        new_traj.append(t - gap)
    return new_traj



with open("Dual-robots/saved_plan_print_dual_0425.json", "r") as f:
    data = json.load(f)


def forward_kinematics(joints,robot_ip):

    # waypoint_place = np.array(data[15]["joint_states"]).reshape(-1)
    # print(frankz.fk_current(robotip_0))
    waypoint_mat = np.array(frankz.fk(joints,robot_ip)).reshape(4,4).T
    print(waypoint_mat)
    rot_mat = waypoint_mat[:3,:3]
    trans_mat = waypoint_mat[:3,3]
    quat = Rotation.from_matrix(rot_mat).as_quat()
    return RobotPose(Affine(trans_mat, quat))

# cartesian_pick_motion = CartesianMotion(
#                         RobotPose(Affine(trans_mat, quat)))  # With target elbow angle
# print(waypoint_place)
# cartesian_state = franky.Kinematics.forward(waypoint_place)
# cartesian_state = franky.Kinematics.forward_euler(waypoint_place)
# print(cartesian_state)

speed = 0.1 # [m/s]
force = 60.0  # [N]

velocity = 0.05
traj_factor = int(8)
frequency = 100
safe = True

# exit()
for i, command in enumerate(data):
    if i >= 83:
    # if i <= 16:
    # if i >= 17:
        if i == 0:
            home(1)
            home(0)
        print(i)
        print(command["type"])
        print("Press Enter to continue...")
        input()
        print("The program has resumed.")
        # if command["robot_id"] == 0:
        #     continue
        if command["type"] == "pick_station":
            continue
        elif command["type"] == "gripper":
            robotid = command["robot_id"]
            robotip = robotips[int(robotid)]
            gripper = franky.Gripper(robotip)
            if command["activate"] == False:
                print(f"Gripper_{robotid} opening.")
                gripper.open(speed)
            elif command["activate"] == True:
                print(f"Gripper_{robotid} grasping.")
                gripper.grasp(0.0, speed, force, epsilon_outer=1.0)
        elif command["type"] == "move_j":
            traj = command["joint_states"]
            robotid = command["robot_id"]
            robotip = robotips[int(robotid)]
            robot = Robot(robotip)
            robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
            waypoint = np.array(command["joint_states"][0]).reshape(-1)
            waypoint_motion = JointWaypointMotion([JointWaypoint(waypoint)])

            print(f"Robot_{robotid} motion executing.")
            robot.move(waypoint_motion)
            new_traj = map_to_current(traj, robot.current_joint_state.position)
            status = frankz.run(new_traj, robotip, traj_factor, 500.0, safe, frequency)


        elif command["type"] == "move_l":
            robotid = command["robot_id"]
            robotip = robotips[int(robotid)]
            robot = Robot(robotip)
            robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
            if data[i - 1]["type"] == "gripper" and data[i - 1]["activate"] == True:
                # pick transfer

                cartesian_state = robot.current_cartesian_state
                robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
                ee_pose_trans = robot_pose.end_effector_pose.translation
                ee_pose_quat = robot_pose.end_effector_pose.quaternion
                print(ee_pose_trans)
                pick_safe_pose = [ee_pose_trans[0], ee_pose_trans[1], ee_pose_trans[2] + 0.015]
                print(pick_safe_pose)
                cartesian_pick_motion = CartesianMotion(
                    RobotPose(Affine(pick_safe_pose, ee_pose_quat)))  # With target elbow angle
                robot.move(cartesian_pick_motion)

                waypoint_place = np.array(command["joint_states"]).reshape(-1)
                waypoint_place_motion = JointWaypointMotion([JointWaypoint(waypoint_place)])
                print(f"Robot_{robotid} pick transfer executing.")
                robot.move(waypoint_place_motion)
            else:
                # pick and place
                if data[i + 1]["type"] == "gripper" and data[i + 1]["activate"] == True:
                    #pick
                    pass
                else:
                    #place
                    send_capture_message()


                waypoint_place = np.array(command["joint_states"]).reshape(-1)
                waypoint_place_motion = JointWaypointMotion([JointWaypoint(waypoint_place)])
                print(f"Robot_{robotid} pick/place executing.")
                # franky.LinearMotion
                place_pose = forward_kinematics(waypoint_place, robotip)
                cartesian_place_motion = CartesianMotion(place_pose)  # With target elbow angle
                robot = Robot(robotip)
                robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
                robot.move(cartesian_place_motion)
                # robot.move(waypoint_place_motion)


                """"
                if i == 3:
                    robot.move(waypoint_place_motion)
                    cartesian_state = robot.current_cartesian_state
                    robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
                    ee_pose_trans = robot_pose.end_effector_pose.translation
                    ee_pose_quat = robot_pose.end_effector_pose.quaternion
                    print(ee_pose_trans)
                    pick_safe_pose = [ee_pose_trans[0] + 0.000, ee_pose_trans[1] + 0.000, ee_pose_trans[2] - 0.002]
                    cartesian_pick_motion = CartesianMotion(
                        RobotPose(Affine(pick_safe_pose, ee_pose_quat)))  # With target elbow angle
                    robot.move(cartesian_pick_motion)
                elif i == 20:
                    robot.move(waypoint_place_motion)
                    cartesian_state = robot.current_cartesian_state
                    robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
                    ee_pose_trans = robot_pose.end_effector_pose.translation
                    ee_pose_quat = robot_pose.end_effector_pose.quaternion
                    print(ee_pose_trans)
                    pick_safe_pose = [ee_pose_trans[0] + 0.000, ee_pose_trans[1] + 0.000, ee_pose_trans[2] + 0.002]
                    cartesian_pick_motion = CartesianMotion(
                        RobotPose(Affine(pick_safe_pose, ee_pose_quat)))  # With target elbow angle
                    robot.move(cartesian_pick_motion)
                elif i == 38:
                    robot.move(waypoint_place_motion)
                    cartesian_state = robot.current_cartesian_state
                    robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
                    ee_pose_trans = robot_pose.end_effector_pose.translation
                    ee_pose_quat = robot_pose.end_effector_pose.quaternion
                    print(ee_pose_trans)
                    pick_safe_pose = [ee_pose_trans[0] + 0.000, ee_pose_trans[1] + 0.000, ee_pose_trans[2] + 0.002]
                    cartesian_pick_motion = CartesianMotion(
                        RobotPose(Affine(pick_safe_pose, ee_pose_quat)))  # With target elbow angle
                    robot.move(cartesian_pick_motion)
                elif i == 96:
                    # place
                    cartesian_state = robot.current_cartesian_state
                    robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
                    ee_pose_trans = robot_pose.end_effector_pose.translation
                    ee_pose_quat = robot_pose.end_effector_pose.quaternion
                    print(ee_pose_trans)
                    pick_safe_pose = [ee_pose_trans[0] - 0.004, ee_pose_trans[1] - 0.004, ee_pose_trans[2] - 0.02]
                    cartesian_pick_motion = CartesianMotion(
                        RobotPose(Affine(pick_safe_pose, ee_pose_quat)))  # With target elbow angle
                    robot.move(cartesian_pick_motion)
                elif i == 56:
                    robot.move(waypoint_place_motion)
                    cartesian_state = robot.current_cartesian_state
                    robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
                    ee_pose_trans = robot_pose.end_effector_pose.translation
                    ee_pose_quat = robot_pose.end_effector_pose.quaternion
                    print(ee_pose_trans)
                    pick_safe_pose = [ee_pose_trans[0] + 0.000, ee_pose_trans[1] + 0.000, ee_pose_trans[2] + 0.002]
                    cartesian_pick_motion = CartesianMotion(
                        RobotPose(Affine(pick_safe_pose, ee_pose_quat)))  # With target elbow angle
                    robot.move(cartesian_pick_motion)
                else:
                    robot.move(waypoint_place_motion)
                    
                """


# home(0)
# home(1)