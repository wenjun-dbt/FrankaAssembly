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
import threading

from franka_assembly import ASSEMBLY_DIR

robotip_0 = "172.16.0.3"
robotip_1 = "172.16.0.5"
robotips = [robotip_0, robotip_1]

speed = 0.1 # [m/s]
force = 60.0  # [N]

velocity = 0.02
traj_factor = int(10)
frequency = 100
safe = True
#
# def home(robot):
#     if robot == 0:
#         home0 = data[10]["joint_states"][0]
#         robot0 = Robot(robotip_0)
#         robot0.relative_dynamics_factor = RelativeDynamicsFactor(0.1, 0.1, 0.1)
#         home0_place_motion = JointWaypointMotion([JointWaypoint(home0)])
#         print(f"Robot_0 going to home position.")
#         robot0.move(home0_place_motion)
#     elif robot == 1:
#         home1 = data[2]["joint_states"][0]
#         robot1 = Robot(robotip_1)
#         robot1.relative_dynamics_factor = RelativeDynamicsFactor(0.1, 0.1, 0.1)
#         home1_place_motion = JointWaypointMotion([JointWaypoint(home1)])
#         print(f"Robot_1 going to home position.")
#         robot1.move(home1_place_motion)


def map_to_current(traj, current):
    gap = traj[0] - current
    # print(traj[0])
    # print(current)
    # print(gap)
    for g in gap:
        if abs(g) > 0.01:
            raise ("cant map to current, the gap exceeds limit")
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


def process_robot_commands(robot_id, robot_ip, commands):
    for i, command in enumerate(commands):
        print(i)
        print(f"robot_id:{robot_id}")
        print(command["type"])
        if command["type"] == "pick_station":
            continue
        elif command["type"] == "gripper":
            gripper = franky.Gripper(robot_ip)
            if command["activate"] == False:
                print(f"Gripper_{robot_id} opening.")
                gripper.open(speed)
            elif command["activate"] == True:
                print(f"Gripper_{robot_id} grasping.")
                gripper.grasp(0.0, speed, force, epsilon_outer=1.0)
        elif command["type"] == "move_j":
            traj = command["joint_states"]
            waypoint = np.array(command["joint_states"][0]).reshape(-1)
            waypoint_motion = JointWaypointMotion([JointWaypoint(waypoint)])
            print(f"Robot_{robot_id} motion executing.")
            robot = Robot(robot_ip)
            robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
            robot.move(waypoint_motion)
            new_traj = map_to_current(traj, robot.current_joint_state.position)
            status = frankz.run(new_traj, robot_ip, traj_factor, 500.0, safe, frequency)
        elif command["type"] == "move_l":
            robot = Robot(robot_ip)
            robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
            if commands[commands.index(command) - 1]["type"] == "gripper" and commands[commands.index(command) - 1]["activate"] == True:
                # pick transfer
                cartesian_state = robot.current_cartesian_state
                robot_pose = cartesian_state.pose
                ee_pose_trans = robot_pose.end_effector_pose.translation
                ee_pose_quat = robot_pose.end_effector_pose.quaternion
                pick_safe_pose = [ee_pose_trans[0], ee_pose_trans[1], ee_pose_trans[2] + 0.015]
                cartesian_pick_motion = CartesianMotion(RobotPose(Affine(pick_safe_pose, ee_pose_quat)))
                robot = Robot(robot_ip)
                robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
                robot.move(cartesian_pick_motion)

                waypoint_place = np.array(command["joint_states"]).reshape(-1)
                waypoint_place_motion = JointWaypointMotion([JointWaypoint(waypoint_place)])
                print(f"Robot_{robot_id} pick transfer executing.")
                robot = Robot(robot_ip)
                robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
                robot.move(waypoint_place_motion)
            else:
                waypoint_place = np.array(command["joint_states"]).reshape(-1)
                place_pose = forward_kinematics(waypoint_place, robot_ip)
                cartesian_place_motion = CartesianMotion(place_pose)
                robot = Robot(robot_ip)
                robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
                robot.move(cartesian_place_motion)

def control_robot(robot_ip, trajectory, traj_factor=8, velocity = 0.03, finished_event=None):
    print(f"Thread {robot_ip} 开始执行...")
    try:
        robot = Robot(robot_ip)
        robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)

        # 移动到轨迹起点
        waypoint = np.array(trajectory[0]).reshape(-1)
        waypoint_motion = JointWaypointMotion([JointWaypoint(waypoint)])
        robot.move(waypoint_motion)

        # 执行轨迹
        new_traj = map_to_current(trajectory, robot.current_joint_state.position)
        status = frankz.run(new_traj, robot_ip, traj_factor, 500.0, True, 100)
        print(f"Thread {robot_ip} 执行完成！状态: {status}")
        # 如果传入了 Event，标记完成
        if finished_event:
            finished_event.set()  # 触发事件

        return status
    except Exception as e:
        print(f"Robot {robot_ip} 控制失败: {e}")
        if finished_event:
            finished_event.set()  # 即使异常也触发事件
        raise

# 定义 thread0 结束后要执行的操作
def after_thread0():
    print("Thread0 已完成！现在可以执行其他操作...")
    # 在这里添加 thread0 结束后的逻辑


# Load your data
with open(ASSEMBLY_DIR + "/traj/byd/b.json", "r") as f:
    data_robot0 = json.load(f)

with open(ASSEMBLY_DIR + "/traj/byd/d.json", "r") as f:
    data_robot1 = json.load(f)

# Create and start threads for each robot
threads = []

thread0 = threading.Thread(target=process_robot_commands, args=(0, robotips[0], data_robot0))
thread1 = threading.Thread(target=process_robot_commands, args=(1, robotips[1], data_robot1))

# threads.append(thread0)
# threads.append(thread1)

# Start the threads
thread0.start()
thread1.start()
#
# Optionally wait for both threads to finish
thread0.join()
thread1.join()

print("双机械臂控制完成")




