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


import csv
import os


import time
from compas_eve import Publisher
from compas_eve import Topic
from compas_eve.mqtt import MqttTransport

def send_capture_message():
    topic = Topic("/compas_eve/zivid/")
    tx = MqttTransport("broker.emqx.io")
    publisher = Publisher(topic, transport=tx)
    msg = dict(text=f"Hello world")
    print(f"Publishing message: {msg}")
    publisher.publish(msg)
    time.sleep(2)


# Function to save joint states to a CSV file
def save_joint_states_to_csv(file_path, joint_states):
    # Format joint states to ensure each number has 15 digits
    formatted_joint_states = [f"{state:.15f}" for state in joint_states]

    # Check if the CSV file already exists
    file_exists = os.path.isfile(file_path)

    # Open the CSV file in append mode
    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        # Write the joint states as a new row
        writer.writerow(formatted_joint_states)




robotip = "172.16.0.3"
# robotip = "172.16.1.3"

info = {}
#info["fun"] = 'Hello world!'
for i in range(20):
    print("Press Enter to Save...")
    # Pause until the user presses Enter
    input()


    robot = Robot(robotip)
    # # Get the current state as raw `franky.RobotState`
    state = robot.state
    #
    # # # Get the robot's cartesian state
    cartesian_state = robot.current_cartesian_state
    robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
    ee_pose = robot_pose.end_effector_pose
    elbow_pos = robot_pose.elbow_position
    robot_velocity = cartesian_state.velocity  # Contains end-effector twist and elbow velocity
    ee_twist = robot_velocity.end_effector_twist
    elbow_vel = robot_velocity.elbow_velocity
    # # #
    # # # # Get the robot's joint state
    joint_state = robot.current_joint_state
    joint_pos = joint_state.position
    joint_vel = joint_state.velocity
    # print(f"cartesian_state: {cartesian_state}")
    # print(f"robot pose: {robot_pose}")
    # print(f"ee_pose: {ee_pose}")
    # print(f"elbow_pos: {elbow_pos}")
    # print(f"joint_state: {joint_state}")
    # print(f"joint_pose: {joint_pos}")


    # Example usage
    # Assuming these variables are already defined in your code
    joint_state = robot.current_joint_state
    joint_pos = joint_state.position  # This should be a list or array of 7 doubles
    # Specify the path to the CSV file


    csv_file_path = 'Collect_data/zivid/zivid_1/zivid_capture_js.csv'
    save_joint_states_to_csv(csv_file_path, joint_pos)
    #send_capture_message()
    print(f"Saved joint states: {joint_pos} to {csv_file_path}")

    # #
    #

    cartpos = cartesian_state.pose.end_effector_pose.translation
    cartquat = cartesian_state.pose.end_effector_pose.quaternion
    mat = cartesian_state.pose.end_effector_pose.matrix.reshape(-1)


    info['mat'] = mat
    info['js'] = joint_pos
    print(mat)
    topic = Topic("/compas_eve/zivid/")

    tx = MqttTransport("broker.emqx.io")
    publisher = Publisher(topic, transport=tx)
    print(f"Publishing message: {info}")
    publisher.publish(info)
    time.sleep(1)

    # cart = np.concatenate((cartpos,cartquat))
    csv_file_path = 'Collect_data/zivid/zivid_1/zivid_capture_mat.csv'
    save_joint_states_to_csv(csv_file_path, mat)
    # Optionally print a message

    print(f"Saved cartesian positions: {mat} to {csv_file_path}")

    cart = []
    cartpos = cartesian_state.pose.end_effector_pose.translation
    cartquat = cartesian_state.pose.end_effector_pose.quaternion
    for p in cartpos:
        cart.append(p)
    for q in cartquat:
        cart.append(q)
    csv_file_path = 'Collect_data/zivid/zivid_1/zivid_capture_pos.csv'
    save_joint_states_to_csv(csv_file_path, cart)
    # Optionally print a message
    print(f"Saved cartesian positions: {cart} to {csv_file_path}")



