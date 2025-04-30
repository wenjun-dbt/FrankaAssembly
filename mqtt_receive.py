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


not_get_delta_pose = True
robotip_0 = "172.16.0.3"
robotip_1 = "172.16.1.3"
robotips = [robotip_0, robotip_1]
calibrated_poses = []
robot_id = None

MQTT_SERVER  = "175.16.1.9" #"broker.emqx.io"

info = dict({
    "step": 96,
    "type": 2,
    "part_id": 10,
    "robot_id": 1,
    "assembly_calibration": 1
})

def move_to_calibration(transform, robotip = "172.16.0.3", velocity = 0.03):
    robot = Robot(robotip)
    robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
    cartesian_mat = robot.current_cartesian_state.pose.end_effector_pose.matrix
    transform = np.array(transform).reshape(4,4)
    print(f"Transform: {transform}")
    cartesian_state_calibrated = transform @ cartesian_mat

    rot_mat = cartesian_state_calibrated[:3, :3]
    trans_mat = cartesian_state_calibrated[:3, 3]
    quat = Rotation.from_matrix(rot_mat).as_quat()

    place_pose = RobotPose(Affine(trans_mat, quat))
    cartesian_place_motion = CartesianMotion(place_pose)  # With target elbow angle
    robot = Robot(robotip)
    robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
    robot.move(cartesian_place_motion)

def listen_for_delta_pose(msg):
    global not_get_delta_pose, calibrated_poses,robot_id
    calibrated_poses = [msg["correct_mat"],msg["insert_mat"]]
    not_get_delta_pose = False

def send_capture_message(info):
    topic = Topic("/ust/zivid/")
    tx = MqttTransport(MQTT_SERVER)
    publisher = Publisher(topic, transport=tx)
    print(f"Publishing message: {info}")
    publisher.publish(info)
    time.sleep(0.01)

topic = Topic("/ust/pose/", Message)
tx = MqttTransport(MQTT_SERVER)
subscriber = Subscriber(topic, callback=listen_for_delta_pose, transport=tx)
subscriber.subscribe()

velocity = 0.01
start_time = time.time()
send_capture_message(info)

not_get_delta_pose = True
while not_get_delta_pose:
    time.sleep(0.01)

print(time.time()- start_time)
# robotip = robotips[robot_id]
# move_to_calibration(calibrated_poses[0], robotip, velocity)
# print("Press Enter to continue...")
# input()
# print("The program has resumed.")
# info['type'] = 1
# move_to_calibration(calibrated_poses[1], robotip, velocity)


#
# robotip = robotips[1]
# robot = Robot(robotip)
# robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
# cartesian_mat = robot.current_cartesian_state.pose.end_effector_pose.matrix
#
#
# tranform = [[ 0.98144076, -0.08830716,  0.17022312, -0.03058212],
#        [ 0.0742946 ,  0.993433  ,  0.08701228, -0.02423795],
#        [-0.1767891 , -0.07275074,  0.98155635,  0.06191237],
#        [ 0.        ,  0.        ,  0.        ,  1.        ]]
#
#
# cartesian_state_calibrated = tranform @ cartesian_mat
#
# rot_mat = cartesian_state_calibrated[:3, :3]
# trans_mat = cartesian_state_calibrated[:3, 3]
# quat = Rotation.from_matrix(rot_mat).as_quat()
#
# place_pose = RobotPose(Affine(trans_mat, quat))
#
# cartesian_place_motion = CartesianMotion(place_pose)  # With target elbow angle
# robot = Robot(robotip)
# robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity, velocity, velocity)
# robot.move(cartesian_place_motion)


