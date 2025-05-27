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

def home(robot):
    if robot == 0:
        home0 = data[10]["joint_states"][0]
        robot0 = Robot(robotip_0)
        robot0.relative_dynamics_factor = RelativeDynamicsFactor(0.1, 0.1, 0.1)
        home0_place_motion = JointWaypointMotion([JointWaypoint(home0)])
        print(f"Robot_0 going to home position.")
        robot0.move(home0_place_motion)
    elif robot == 1:
        home1 = data[2]["joint_states"][0]
        robot1 = Robot(robotip_1)
        robot1.relative_dynamics_factor = RelativeDynamicsFactor(0.1, 0.1, 0.1)
        home1_place_motion = JointWaypointMotion([JointWaypoint(home1)])
        print(f"Robot_1 going to home position.")
        robot1.move(home1_place_motion)


def map_to_current(traj, current):
    gap = traj[0] - current
    for g in gap:
        if g > 0.01:
            raise "cant map to current, the gap exceeds limit"
    new_traj = []
    for t in traj:
        new_traj.append(t - gap)
    return new_traj


with open(ASSEMBLY_DIR + "/traj/dual/assembly_dual_realtime.json", "r") as f:
# with open("dual/assembly_dual_offline.json", "r") as f:
    data = json.load(f)


def forward_kinematics(joints,robot_ip):
    waypoint_mat = np.array(frankz.fk(joints,robot_ip)).reshape(4,4).T
    print(waypoint_mat)
    rot_mat = waypoint_mat[:3,:3]
    trans_mat = waypoint_mat[:3,3]
    quat = Rotation.from_matrix(rot_mat).as_quat()
    return RobotPose(Affine(trans_mat, quat))



def control_robot(robot_ip, trajectory, traj_factor=8, finished_event=None):
    print(f"Thread {robot_ip} 开始执行...")
    try:
        robot = Robot(robot_ip)
        robot.relative_dynamics_factor = RelativeDynamicsFactor(0.03, 0.03, 0.03)

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




robotip0 = robotips[0]
robotip1 = robotips[1]


traj0 = data[10]["joint_states"]
traj1 = data[2]["joint_states"]
# traj0.reverse()
# traj1.reverse()


# 创建一个事件对象来监控 thread0 的状态
thread0_finished = threading.Event()

# 启动 thread0（绑定 Event）
thread0 = threading.Thread(
    target=control_robot,
    args=(robotip0, traj0, 8),
    kwargs={"finished_event": thread0_finished}
)
thread0.start()

# 启动 thread1（不绑定 Event）
thread1 = threading.Thread(
    target=control_robot,
    args=(robotip1, traj1, 8)
)
thread1.start()

# 主线程等待 thread0 完成
thread0_finished.wait()
after_thread0()  # 立即执行后续操作

# thread1 会继续运行，不受影响




# # 创建并启动线程
# thread0 = threading.Thread(target=control_robot, args=(robotips[0], traj0, 8))
# thread1 = threading.Thread(target=control_robot, args=(robotips[1], traj1, 8))
#
# thread0.start()
# thread1.start()
#
# # 等待线程结束
# thread0.join()
# thread1.join()
#
# print("双机械臂控制完成")