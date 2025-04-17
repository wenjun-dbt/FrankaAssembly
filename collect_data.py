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


    csv_file_path = 'Collect_data/robot0/world_frame3/hole_1.csv'
    save_joint_states_to_csv(csv_file_path, joint_pos)
    print(f"Saved joint states: {joint_pos} to {csv_file_path}")
    # #

    cartpos = cartesian_state.pose.end_effector_pose.translation
    csv_file_path = 'Collect_data/robot0/world_frame3/cart_pos_1.csv'
    save_joint_states_to_csv(csv_file_path, cartpos)
    # Optionally print a message
    print(f"Saved cartesian positions: {cartpos} to {csv_file_path}")



