import math
import scipy
from scipy.spatial.transform import Rotation
from franky import JointWaypointMotion, JointWaypoint, CartesianMotion, \
    CartesianWaypointMotion, CartesianWaypoint, Affine, Twist, RobotPose, ReferenceType, \
    CartesianState, JointState, RelativeDynamicsFactor
from franky import Robot

from franky import Robot, JointVelocityMotion, JointPositionStopMotion,CartesianPoseStopMotion,CartesianVelocityMotion, Duration, JointMotion, Twist

robot = Robot("172.16.0.3")

# Get the current state as raw `franky.RobotState`
state = robot.state

# Get the robot's cartesian state
print(state)
cartesian_state = robot.current_cartesian_state
robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
ee_pose = robot_pose.end_effector_pose
# elbow_pos = robot_pose.elbow_position
robot_velocity = cartesian_state.velocity  # Contains end-effector twist and elbow velocity
ee_twist = robot_velocity.end_effector_twist
elbow_vel = robot_velocity.elbow_velocity

# Get the robot's joint state
joint_state = robot.current_joint_state
joint_pos = joint_state.position
joint_vel = joint_state.velocity

# Reduce the acceleration and velocity dynamic
robot.relative_dynamics_factor = RelativeDynamicsFactor(0.05, 0.05, 0.05)

# A point-to-point motion in the joint space
m1 = JointWaypointMotion([JointWaypoint([-0.25, 0.1, 0.3, -1.8, 0.1, 1.8, 0.7])])

# A motion in joint space with multiple waypoints

nway = 10
positions = [
    [0.1, 0.4, 0.3 + 0.01 * id, -1.6 - 0.01 * id, -0.3, 1.7, 0.9]
    for id in range(nway)
]
velocities = [
    [0.0, 0.0, 0.05, -0.05, 0, 0, 0]
    for id in range(nway)
]
velocities[0] = [0] * 7
velocities[-1] = [0] * 7
# print(positions)

waypoints = [JointWaypoint(
        JointState(
            position=positions[id],
            velocity=velocities[id]
        )
) for id in range(nway)]

m2 = JointWaypointMotion(waypoints)

# Intermediate waypoints also permit to specify target velocities. The default target velocity is 0, meaning that the
# robot will stop at every waypoint.
m3 = JointWaypointMotion([
    JointWaypoint([-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7]),
    JointWaypoint(
        JointState(
            position=[0.0, 0.3, 0.3, -1.5, -0.2, 1.5, 0.8],
            velocity=[0.1, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0])),
    JointWaypoint([0.1, 0.4, 0.3, -1.4, -0.3, 1.7, 0.9]),
])

# Stop the robot
m4 = JointPositionStopMotion()

# A linear motion in cartesian space
quat = Rotation.from_euler("xyz", [math.pi, 0, 0]).as_quat()
# print(quat)
# quat = [0.522124,0.103382,0.528315,0.6615]
# m5 = CartesianMotion(Affine([0.4, -0.2, 0.3], quat))
m5 = CartesianMotion(Affine([0.6, 0.2, 0.3], quat))
m6 = CartesianMotion(RobotPose(Affine([0.6, 0.2, 0.3], quat), elbow_position=0.3))  # With target elbow angle

# A linear motion in cartesian space relative to the initial position
# (Note that this motion is relative both in position and orientation. Hence, when the robot's end-effector is oriented
# differently, it will move in a different direction)
m7 = CartesianMotion(Affine([0.0, 0.0, -0.05]), ReferenceType.Relative)

# Generalization of CartesianMotion that allows for multiple waypoints
m8 = CartesianWaypointMotion([
    CartesianWaypoint(RobotPose(Affine([0.4, -0.2, 0.3], quat), elbow_position=0.3)),
    # The following waypoint is relative to the prior one and 50% slower
    CartesianWaypoint(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative, RelativeDynamicsFactor(0.5, 1.0, 1.0))
])

# Cartesian waypoints also permit to specify target velocities
m9 = CartesianWaypointMotion([
    CartesianWaypoint(Affine([0.5, -0.2, 0.3], quat)),
    CartesianWaypoint(
        CartesianState(
            pose=Affine([0.4, -0.1, 0.3], quat),
            velocity=Twist([-0.01, 0.01, 0.0]))),
    CartesianWaypoint(Affine([0.3, 0.0, 0.3], quat))
])

# Stop the robot. The difference of JointPositionStopMotion to CartesianPoseStopMotion is that JointPositionStopMotion
# stops the robot in joint position control mode while CartesianPoseStopMotion stops it in cartesian pose control mode.
# The difference becomes relevant when asynchronous move commands are being sent (see below).
m10 = CartesianPoseStopMotion()

# Cartesian waypoints also permit to specify target velocities
m11 = CartesianWaypointMotion([
    CartesianWaypoint(Affine([0.6, 0.2, 0.3], quat)),
    CartesianWaypoint(Affine([0.62, 0.2, 0.3], quat)),
    CartesianWaypoint(Affine([0.64, 0.2, 0.3], quat)),
    CartesianWaypoint(Affine([0.66, 0.2, 0.3], quat)),
    CartesianWaypoint(Affine([0.68, 0.2, 0.3], quat))
])

# robot.move(m6)
# robot.move(m11)
# robot.move(m6)
# robot.move(m7)
# robot.move(m11)
print(cartesian_state.pose.end_effector_pose.translation)
