import math
import time
import json
import threading
from queue import Queue
import numpy as np
from scipy.spatial.transform import Rotation

# Franka 和 MQTT 相关库
from franky import *
import frankz
from franky import JointWaypointMotion, JointWaypoint, CartesianMotion, \
    CartesianWaypointMotion, CartesianWaypoint, Affine, Twist, RobotPose, ReferenceType, \
    CartesianState, JointState, RelativeDynamicsFactor
from compas_eve import Publisher, Subscriber, Topic, Message
from compas_eve.mqtt import MqttTransport

from franka_assembly import ASSEMBLY_DIR


# ==================== 全局配置 ====================
ROBOT_IPS = ["172.16.0.3", "172.16.0.5"]
MQTT_SERVER = "localhost"
CALIBRATION_TOPIC = "/ust/pose/"
CAPTURE_TOPIC = "/ust/zivid/"

# 共享通信对象（线程安全）
keyboard_queue = Queue()  # 键盘指令
mqtt_queues = {ip: Queue() for ip in ROBOT_IPS}  # 每个机器人独立的MQTT队列
stop_event = threading.Event()  # 全局停止信号
calibration_events = {0: threading.Event(), 1: threading.Event()}  # 每个机器人的校准完成事件
calibrated_poses = {0: None, 1: None}  # 每个机器人的校准数据
calibration_lock = threading.Lock()  # 校准请求锁
calibration_in_progress = threading.Event()  # 校准进行中标志


# ==================== MQTT 通信 ====================
def setup_mqtt():
    """初始化MQTT订阅和发布"""

    # 校准数据订阅
    def on_calibration_message(msg):
        try:
            robot_id = msg["robot_id"]
            if robot_id in [0, 1]:
                calibrated_poses[robot_id] = [msg["correct_mat"], msg["insert_mat"]]
                # 设置对应机器人的校准完成事件
                calibration_events[robot_id].set()
                calibration_in_progress.clear()  # 标记校准完成
                print(f"收到机器人 {robot_id} 的校准数据")
        except Exception as e:
            print(f"校准数据处理失败: {e}")
            calibration_in_progress.clear()  # 出错时也要清除标志

    calibration_subscriber = Subscriber(
        Topic(CALIBRATION_TOPIC, Message),
        callback=on_calibration_message,
        transport=MqttTransport(MQTT_SERVER)
    )
    calibration_subscriber.subscribe()

    # 图像捕获发布
    def send_capture_message(info):
        publisher = Publisher(
            Topic(CAPTURE_TOPIC, Message),
            transport=MqttTransport(MQTT_SERVER))
        publisher.publish(info)
        print(f"发送校准请求：机器人 {info.get('robot_id')}")

    return send_capture_message


# ==================== 键盘监听 ====================
def keyboard_listener():
    """独立线程监听键盘输入"""
    while not stop_event.is_set():
        try:
            cmd = input("\n控制指令 (stop/pause/continue): ").strip().lower()
            keyboard_queue.put(cmd)
            if cmd == "stop":
                stop_event.set()
                break
            time.sleep(0.1)
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            break


# ==================== 机器人控制核心 ====================
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


def forward_kinematics(joints, robot_ip):
    waypoint_mat = np.array(frankz.fk(joints, robot_ip)).reshape(4, 4).T
    print(waypoint_mat)
    rot_mat = waypoint_mat[:3, :3]
    trans_mat = waypoint_mat[:3, 3]
    quat = Rotation.from_matrix(rot_mat).as_quat()
    return RobotPose(Affine(trans_mat, quat))


# ==================== 机器人控制核心 ====================
class RobotController:
    def __init__(self, robot_id, robot_ip):
        self.robot_id = robot_id
        self.robot_ip = robot_ip
        self.robot = None
        self.current_pose = None
        self.gripper_speed = 0.05
        self.gripper_force = 60
        self.velocity = 0.02
        self.traj_factor = int(15)

    def initialize(self, velocity = 0.01):
        """初始化机器人连接"""
        self.robot = Robot(self.robot_ip)
        self.robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity)
        self.update_pose()

    def update_pose(self):
        """更新当前位姿"""
        if self.robot:
            self.current_pose = self.robot.current_cartesian_state.pose

    def update_velocity(self, velocity):
        self.velocity = velocity

    def move_joint(self, target_joints, velocity=0.1):
        """安全关节运动"""
        self.initialize()
        waypoint = JointWaypoint(target_joints)
        motion = JointWaypointMotion([waypoint])
        # print(f"Robot_{self.robot_id} move_j executing.")
        self.robot.move(motion)

    def move_cartesian(self, target_affine, velocity=0.1):
        """安全笛卡尔运动"""
        self.initialize()
        pose = RobotPose(target_affine)
        motion = CartesianMotion(pose)
        self.robot.move(motion)

    def move_to_calibration(self, transform, safe_dist, velocity=0.003):
        self.initialize()
        cartesian_mat = self.robot.current_cartesian_state.pose.end_effector_pose.matrix
        cartesian_trans = self.robot.current_cartesian_state.pose.end_effector_pose.translation

        transform = np.array(transform).reshape(4, 4)
        # print(f"Transform: {transform}")
        cartesian_state_calibrated = transform @ cartesian_mat

        rot_mat = cartesian_state_calibrated[:3, :3]
        trans_mat = cartesian_state_calibrated[:3, 3]
        quat = Rotation.from_matrix(rot_mat).as_quat()

        moved_dis = trans_mat - cartesian_trans
        # print(f"ee_pose_trans:{cartesian_trans}")
        # print(f"calibrate_trans:{trans_mat}")
        print(f"moved_dis:{moved_dis}")
        for dis in moved_dis:
            if abs(dis) > safe_dist:
                print(f"moved_dis{dis} exceeds safe_dist:{safe_dist}")
                return
        place_pose = RobotPose(Affine(trans_mat, quat))
        self.initialize(velocity)
        self.move_cartesian(place_pose)

    def handle_gripper(self, command):
        gripper = Gripper(self.robot_ip)
        if command["activate"] == False:
            # print(f"Gripper_{self.robot_id} opening.")
            gripper.open(self.gripper_speed)
        elif command["activate"] == True:
            # print(f"Gripper_{self.robot_id} grasping.")
            gripper.grasp(0.0, self.gripper_speed, self.gripper_force, epsilon_outer=1.0)

    def handle_traj_move(self, command):
        traj = command["joint_states"]
        start_point = np.array(command["joint_states"][0]).reshape(-1)
        self.move_joint(start_point)
        mapped_traj = map_to_current(traj, self.robot.current_joint_state.position)
        status = frankz.run(mapped_traj, self.robot_ip, self.traj_factor, 500.0, True, 100)

    def handle_pick(self, command):
        self.initialize()
        waypoint = np.array(command["joint_states"]).reshape(-1)
        pose = forward_kinematics(waypoint, self.robot_ip)
        self.move_cartesian(pose)

    def handle_place(self, command):
        self.initialize()
        waypoint = np.array(command["joint_states"]).reshape(-1)
        pose = forward_kinematics(waypoint, self.robot_ip)
        self.move_cartesian(pose)

    def handle_calibration(self, command):
        """处理校准流程"""
        self.initialize()
        
        # 等待获取校准锁
        with calibration_lock:
            # 确保没有其他校准正在进行
            while calibration_in_progress.is_set():
                print(f"机器人 {self.robot_id} 等待其他校准完成...")
                time.sleep(0.5)
                if stop_event.is_set():
                    raise Exception("收到停止信号")
            
            # 设置校准进行中标志
            calibration_in_progress.set()
            
            # 清除之前的校准事件状态
            calibration_events[self.robot_id].clear()
            calibrated_poses[self.robot_id] = None
            
            # 发送校准请求
            info = {
                'step': 15,
                'type': 0,
                "part_id": command["part_id"],
                "robot_id": self.robot_id,
                "assembly_calibration": 0,
                "time": time.time()
            }
            send_capture_message(info)
            
            print(f"机器人 {self.robot_id} 等待校准数据...")
            # 等待校准数据，设置超时时间为10秒
            if not calibration_events[self.robot_id].wait(timeout=10.0):
                calibration_in_progress.clear()  # 超时时清除标志
                raise Exception(f"机器人 {self.robot_id} 校准数据等待超时！")
            
            print(f"机器人 {self.robot_id} 收到校准数据，准备执行校准移动")
            
            # 执行校准移动
            if calibrated_poses[self.robot_id]:
                correct_mat, insert_mat = calibrated_poses[self.robot_id]
                print(f"机器人 {self.robot_id} 执行第一次校准移动")
                self.move_to_calibration(correct_mat, 0.01, 0.003)
                time.sleep(0.5)
                print(f"机器人 {self.robot_id} 执行第二次校准移动")
                self.move_to_calibration(insert_mat, 0.03, 0.005)
            else:
                calibration_in_progress.clear()  # 出错时清除标志
                raise Exception(f"机器人 {self.robot_id} 未收到有效的校准数据")

    def execute_command(self, command):
        """执行单条控制指令"""
        try:
            if command["type"] == "gripper":
                self.handle_gripper(command)
            elif command["type"] == "move_j":
                self.handle_traj_move(command)
            elif command["type"] == "move_l":
                if command["func"] == "pick":
                    self.handle_pick(command)
                elif command["func"] == "place":
                    self.handle_place(command)
                elif command["func"] == "calibration":
                    self.handle_calibration(command)
        except Exception as e:
            print(f"Robot {self.robot_id} 执行失败: {e}")
            raise


# ==================== 主控制线程 ====================
def robot_control_worker(robot_id, robot_ip, command_list):
    """机器人控制主线程"""
    controller = RobotController(robot_id, robot_ip)

    try:
        controller.initialize()
        print(f"机器人 {robot_id} 初始化完成")

        for cmd_idx, command in enumerate(command_list):
            #if cmd_idx <= 24:
            #     continue

            # 检查停止信号
            if stop_event.is_set():
                print(f"机器人 {robot_id} 收到停止信号")
                break

            # 处理键盘指令
            if not keyboard_queue.empty():
                key_cmd = keyboard_queue.get()
                if key_cmd == "pause":
                    print(f"机器人 {robot_id} 已暂停，按回车继续...")
                    input()
                elif key_cmd == "stop":
                    print(f"机器人 {robot_id} 收到停止命令")
                    break

            # 执行指令
            print(f"机器人 {robot_id} 执行指令 {cmd_idx}/{len(command_list)-1}-{command['type']}")
            controller.execute_command(command)

    except Exception as e:
        print(f"机器人 {robot_id} 发生错误: {e}")
        stop_event.set()  # 出错时通知其他线程停止

    finally:
        print(f"机器人 {robot_id} 控制线程结束")


def handle_calibration_procedure(controller, step_idx):
    """处理校准流程"""
    # 发送捕获请求
    capture_info = {
        "step": step_idx,
        "robot_id": controller.robot_id,
        "timestamp": time.time()
    }
    send_capture_message(capture_info)

    # 等待校准数据（带超时）
    wait_start = time.time()
    while calibrated_poses[0] is None and (time.time() - wait_start) < 10.0:
        time.sleep(0.1)

    if calibrated_poses[0]:
        controller.execute_calibration_moves()
    else:
        print("警告：校准数据超时未收到！")


# ==================== 主程序 ====================
if __name__ == "__main__":
    print("启动双机械臂控制程序...")

    # 加载轨迹文件
    try:
        with open(ASSEMBLY_DIR + "/traj/byd/b.json", "r") as f:
            robot0_commands = json.load(f)
        with open(ASSEMBLY_DIR + "/traj/byd/d.json", "r") as f:
            robot1_commands = json.load(f)
    except Exception as e:
        print(f"加载轨迹文件失败: {e}")
        exit(1)


    assembly_sequence0 = [
        [0, 0, [0, 0, 0.02], [1, 0, 0]],
        [0, 1, [-0.02, 0, 0.02], [1, 0, 0]],
        [0, 2, [0.0, 0, 0.02], None],
        [0, 3, [0.02, 0, 0.02], [0, -1, 0]],

        [0, -1, None, None],
        [1, -1, None, None]]

    assembly_sequence1 = [
        [1, 8, [0, 0, 0.02], [-1,0,0]],
        [1, 9, [0.02, 0, 0.02], [0,-1,0]],
        [1, 10, [0.02, 0, 0.02], [0,-1,0]],
        [0, -1, None, None],
        [1, -1, None, None]]

    assembly_sequences = [assembly_sequence0, assembly_sequence1]

    for j, commands in enumerate([robot0_commands, robot1_commands]):
        k = 0
        for i, command in enumerate(commands):
            if command["type"] == "move_l":
                if commands[i - 1]["type"] == "gripper" and commands[i - 1]["activate"] == True:
                    command["func"] = "pick"
                else:
                    if commands[i + 1]["type"] == "gripper" and commands[i + 1]["activate"] == True:
                        command["func"] = "pick"
                    else:
                        command["func"] = "calibration"
                        command["part_id"] = assembly_sequences[j][k][1]
                        k += 1




    # 初始化通信
    send_capture_message = setup_mqtt()
    keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
    keyboard_thread.start()

    # 启动机器人线程
    robot_threads = [
        # threading.Thread(target=robot_control_worker, args=(0, ROBOT_IPS[0], robot0_commands)),
        threading.Thread(target=robot_control_worker, args=(1, ROBOT_IPS[1], robot1_commands))
    ]

    try:
        print("启动机器人控制线程...")
        for t in robot_threads:
            t.start()

        # 主线程监控
        while any(t.is_alive() for t in robot_threads):
            time.sleep(1)
            if stop_event.is_set():
                print("检测到停止信号，等待所有线程结束...")
                break

    except KeyboardInterrupt:
        print("\n收到键盘中断，正在停止所有机器人...")
        stop_event.set()

    finally:
        # 等待所有线程结束
        for t in robot_threads:
            t.join(timeout=5.0)
            if t.is_alive():
                print("警告：部分线程未能正常结束")

        # 清理校准状态
        for robot_id in [0, 1]:
            calibration_events[robot_id].clear()
            calibrated_poses[robot_id] = None

        print("双机械臂控制程序结束")