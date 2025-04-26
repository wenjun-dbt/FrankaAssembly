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



robotip = "172.16.0.3"
# robotip = "172.16.1.3"

js =[]
for i in range(10):
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
    joint_pos = joint_state.position.tolist()  # This should be a list or array of 7 doubles
    # Specify the path to the CSV file
    print(f"Added joint states: {joint_pos}")
    js.append(joint_pos)



file_path = 'Collect_data/robot0_pick/js_7.json'
with open(file_path, 'w') as f:
    json.dump(js, f, indent=4)



