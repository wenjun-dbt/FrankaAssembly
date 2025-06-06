import math
from time import sleep

import numpy as np
import scipy
from scipy.spatial.transform import Rotation
from scipy.special import euler
import time
from compas_eve import Publisher
from compas_eve import Subscriber
from compas_eve import Topic
from compas_eve.mqtt import MqttTransport
from compas_eve import Message

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



def send_capture_message(info):
    topic = Topic("/ust/zivid/")
    tx = MqttTransport(MQTT_SERVER)
    publisher = Publisher(topic, transport=tx)
    print(f"Publishing message: {info}")
    publisher.publish(info)
    time.sleep(0.1)


def move_to_calibration(transform, safe_dist, robotip = "172.16.0.3", velocity = 0.03):
    robot = Robot(robotip)
    robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
    cartesian_mat = robot.current_cartesian_state.pose.end_effector_pose.matrix
    cartesian_trans = robot.current_cartesian_state.pose.end_effector_pose.translation

    transform = np.array(transform).reshape(4,4)
    print(f"Transform: {transform}")
    cartesian_state_calibrated = transform @ cartesian_mat

    rot_mat = cartesian_state_calibrated[:3, :3]
    trans_mat = cartesian_state_calibrated[:3, 3]
    quat = Rotation.from_matrix(rot_mat).as_quat()

    moved_dis = trans_mat - cartesian_trans
    print(f"ee_pose_trans:{cartesian_trans}")
    print(f"calibrate_trans:{trans_mat}")
    print(f"moved_dis:{moved_dis}")

    for dis in moved_dis:
        if abs(dis) > safe_dist:
            print(f"moved_dis{dis} exceeds safe_dist:{safe_dist}")
            return

    place_pose = RobotPose(Affine(trans_mat, quat))
    cartesian_place_motion = CartesianMotion(place_pose)  # With target elbow angle
    robot = Robot(robotip)
    robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
    robot.move(cartesian_place_motion)

def listen_for_delta_pose(msg):
    global not_get_delta_pose, robotips, calibrated_poses
    calibrated_poses = [msg["correct_mat"],msg["insert_mat"]]
    not_get_delta_pose = False



def home(robot):
    if robot == 0:
        home0 = data[2]["joint_states"][0]
        robot0 = Robot(robotip_0)
        robot0.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.03, 0.03)
        home0_place_motion = JointWaypointMotion([JointWaypoint(home0)])
        print(f"Robot_0 going to home position.")
        robot0.move(home0_place_motion)
    elif robot == 1:
        home1 = data[2]["joint_states"][0]
        robot1 = Robot(robotip_1)
        robot1.relative_dynamics_factor = RelativeDynamicsFactor(0.03,0.03, 0.03)
        home1_place_motion = JointWaypointMotion([JointWaypoint(home1)])
        print(f"Robot_1 going to home position.")
        robot1.move(home1_place_motion)


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


not_get_delta_pose = True
robotip_0 = "172.16.0.3"
robotip_1 = "172.16.0.5"
robotips = [robotip_0, robotip_1]
calibrated_poses = []

# MQTT_SERVER  = "175.16.1.9" #"broker.emqx.io"
MQTT_SERVER  = "localhost" #"broker.emqx.io"
topic = Topic("/ust/pose/", Message)
tx = MqttTransport(MQTT_SERVER)
subscriber = Subscriber(topic, callback=listen_for_delta_pose, transport=tx)
subscriber.subscribe()

assembly_sequence = [
    [0, 4, [0, 0, 0.02], [1, 0, 0]],
    [1, 5, [0, 0, 0.02], [0, 0, -1]],
    [0, 6, [0, 0, 0.02], [1, 0, 0]],
    [1, 7, [0, 0, 0.02], [-1, 0, 0]],
    #
    # [1, 8, [0, 0, 0.02], [-1,0,0]],
    # [1, 9, [0.02, 0, 0.02], [0,-1,0]],
    # [1, 10, [0.02, 0, 0.02], [0,-1,0]],
    #
    # [0, 0, [0, 0, 0.02], [1,0,0]],
    # [0, 1, [-0.02, 0, 0.02], [1,0,0]],
    # [0, 2, [0.0, 0, 0.02], None],
    # [0, 3, [0.02, 0, 0.02], [0,-1,0]],

    [0, -1, None, None],
    [1, -1, None, None]]


with open(ASSEMBLY_DIR + "/traj/byd/y.json", "r") as f:
    data = json.load(f)

speed = 0.1 # [m/s]
force = 60.0  # [N]

velocity = 0.02
traj_factor = int(15)
frequency = 100
wait = 500.0
safe = True
k = 0
for i, command in enumerate(data):
    if command["type"] == "move_l":
        robotid = command["robot_id"]
        if data[i - 1]["type"] == "gripper" and data[i - 1]["activate"] == True:
            pass
        else:
            # pick and place
            if data[i + 1]["type"] == "gripper" and data[i + 1]["activate"] == True:
                # pick
                pass
            else:
                # place
                # data[i]["install_step"] = k
                data[i]["part_id"] = assembly_sequence[k][1]
                k += 1


# exit()
for i, command in enumerate(data):
    # if i >= 0:
    if i >= 33:
    # if i >= 42:
    # if i >= 17:
        if i == 0:
            home(1)
            home(0)
        print(i)
        print(command["type"])
        # print("Press Enter to continue...")
        # input()
        # print("The program has resumed.")
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
            status = frankz.run(new_traj, robotip, traj_factor, wait, safe, frequency)


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
                    waypoint_place = np.array(command["joint_states"]).reshape(-1)
                    waypoint_place_motion = JointWaypointMotion([JointWaypoint(waypoint_place)])
                    print(f"Robot_{robotid} pick executing.")
                    # franky.LinearMotion
                    place_pose = forward_kinematics(waypoint_place, robotip)
                    cartesian_place_motion = CartesianMotion(place_pose)  # With target elbow angle
                    robot = Robot(robotip)
                    robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
                    robot.move(cartesian_place_motion)
                    # robot.move(waypoint_place_motion)
                else:
                    #place
                    if i == 24: #24
                        waypoint_place = np.array(command["joint_states"]).reshape(-1)
                        waypoint_place_motion = JointWaypointMotion([JointWaypoint(waypoint_place)])
                        print(f"Robot_{robotid} place executing.")
                        # franky.LinearMotion
                        place_pose = forward_kinematics(waypoint_place, robotip)
                        cartesian_place_motion = CartesianMotion(place_pose)  # With target elbow angle
                        robot = Robot(robotip)
                        robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
                        robot.move(cartesian_place_motion)
                    else:
                        info = {}
                        info['step'] = i
                        info['type'] = 0
                        info["part_id"] = command["part_id"]
                        info["robot_id"] = command["robot_id"]
                        info["assembly_calibration"] = 0
                        info["time"] = time.time()
                        send_capture_message(info)

                        not_get_delta_pose = True
                        while not_get_delta_pose:
                            time.sleep(0.1)

                        print("Press Enter to continue...")
                        input()
                        print("The program has resumed.")
                        move_to_calibration(calibrated_poses[0], 0.02, robotip, 0.01)

                        print("Press Enter to continue...")
                        input()
                        print("The program has resumed.")
                        move_to_calibration(calibrated_poses[1], 0.03, robotip, 0.01)


# home(0)
# home(1)