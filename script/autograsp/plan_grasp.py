# Third Party
import numpy as np
import torch

from curobo.geom.transform import quaternion_to_matrix
from curobo.geom.transform import matrix_to_quaternion
from trimesh.transformations import quaternion_from_matrix
from wandb.env import DATA_DIR
from warp.types import Quaternion
from scipy.spatial.transform import Rotation
# cuRobo
from curobo.types.robot import JointState
import polyscope as ps
from robofab import ROBOFAB_DIR
from flipparts import RESOURCES_DIR
import polyscope.imgui as psim
from robofab.render import *
from robofab.place import *
from robofab.pick import *

import json

def interface_ik_solver():
    global kin_model, link_meshes, qtraj, itraj, select_part, place_poses, robot_id
    changed, itraj = psim.SliderInt(label="traj", v=itraj, v_min=0, v_max=max(qtraj.shape[0] - 1, 0))
    if changed or itraj == -1:
        itraj = max(itraj, 0)
        draw_robot(kin_model, link_meshes, qtraj[itraj, :])
        spheres = kin_model.get_robot_as_spheres(qtraj[itraj, :])[0]
        draw_spheres(robot_id, spheres)


if __name__ == "__main__":

    open_width = 0.04
    closed_width = 0.025
    ps.init()
    ps.set_up_dir("z_up")
    ps.set_front_dir("neg_y_front")
    ps.set_ground_plane_mode("shadow_only")

    robot_id = 435
    # draw
    kin_model = get_franka_kin_model(robot_id, 0.06)
    link_meshes = []
    for link_name in kin_model.kinematics_config.mesh_link_names:
        mesh = kin_model.get_link_mesh(link_name)
        link_meshes.append(mesh.get_trimesh_mesh())
    world_config = parse_assembly_from_files(ROBOFAB_DIR + "/assembly/tetris-byd-visual", position=[0.0,0.0,0.0])

    grasp_mesh = trimesh.load_mesh(ROBOFAB_DIR + "/assembly/tetris-byd-visual/2.obj")
    # to world
    with open(RESOURCES_DIR + "/transforms/2/object_transform_world.json", "r") as f:
        to_world = np.array(json.load(f)).reshape(4, 4)

    grasp_mat = to_world
    trans_mat = grasp_mat[:3, 3]
    quat = quaternion_from_matrix(grasp_mat)
    grasp_pose = [trans_mat[0],trans_mat[1],trans_mat[2], quat[0], quat[1], quat[2], quat[3]]

    list_obstacles = []
    for id in range(len(world_config.mesh)):
        mesh_obj = world_config.mesh[id]
        if "part" in mesh_obj.name:
            partID = int(mesh_obj.name.split("_")[1])
            pose = grasp_pose.copy()
            if partID == 2:
                print("part2")
                pose = pose.copy()
                obstacle = Mesh(
                    name=f"new_part_{partID}",
                    pose=pose,
                    file_path=mesh_obj.file_path,
                    scale=mesh_obj.scale.copy(),
                )
                list_obstacles.append(obstacle)
    list_obstacles.append(ground_mesh())
    world_config = WorldConfig(mesh=list_obstacles, cuboid=world_config.cuboid)

    assembly = get_trimesh_from_worldconfig(world_config)
    part_status = np.zeros(len(assembly))

    with open(RESOURCES_DIR + "/transforms/2/robot_joints.json", "r") as f:
        start_pose = np.array(json.load(f))

    with open(RESOURCES_DIR + "/transforms/2/pick_frames.json", "r") as f:
        approach_matrices = np.array(json.load(f))

    for i, mat in enumerate(approach_matrices):
        approach_matrix = np.array(mat).reshape(4, 4)
        trans = approach_matrix[:3, 3]

        trans_mat = approach_matrix[:3, 3]
        quat = quaternion_from_matrix(approach_matrix)
        # Convert trans to tensor using kin_model.tensor_args
        trans_tensor = torch.from_numpy(trans).to(**kin_model.tensor_args.as_torch_dict()).reshape(1, 3)
        quat_tensor = torch.from_numpy(quat).to(**kin_model.tensor_args.as_torch_dict()).reshape(1, 4)
        goal_poses = Pose(position=trans_tensor, quaternion=quat_tensor)

        # Visualize approach frame using point cloud and vectors
        frame_name = f"approach_frame"
        # Register origin point
        frame = ps.register_point_cloud(frame_name, trans_mat.reshape(1, 3))
        # Add X, Y, Z axes as vectors
        frame.add_vector_quantity(
            "X_axis", 
            approach_matrix[:3, 0].reshape(1, 3), 
            color=(1, 0, 0),  # Red for X
            length=0.1,
            radius=0.005,
            enabled=True
        )
        frame.add_vector_quantity(
            "Y_axis", 
            approach_matrix[:3, 1].reshape(1, 3), 
            color=(0, 1, 0),  # Green for Y
            length=0.1,
            radius=0.005,
            enabled=True
        )
        frame.add_vector_quantity(
            "Z_axis", 
            approach_matrix[:3, 2].reshape(1, 3), 
            color=(0, 0, 1),  # Blue for Z
            length=0.1,
            radius=0.005,
            enabled=True
        )

        start_state = JointState.from_position(torch.tensor(start_pose, **kin_model.tensor_args.as_torch_dict()).view(1, -1))
        # goal_state = JointState.from_position(torch.tensor(goal_state, **tensor_args.as_torch_dict()).view(1, -1))
        # result = plan_ik(robot_id=robot_id,
        #                  world_config=None,
        #                  place_poses = goal_poses,
        #                  gripper_width= 0.06)
        result = plan_joint_to_frame(robot_id,
                                     world_config,
                                     transfer_state=start_state,
                                     pick_poses=goal_poses,
                                     gripper_close_width=open_width)

        kin_model = get_franka_kin_model(robot_id, closed_width)
        draw_robot(kin_model = kin_model, link_meshes=link_meshes, q = start_state.position.view(-1))

        if result is None:
            draw_assembly(assembly, part_status, edge_width=1E-3)
            ps.reset_camera_to_home_view()

            draw_assembly(assembly, np.ones(len(assembly)))
            kin_model = get_franka_kin_model(robot_id, open_width)

        else:
            qtraj, itraj = result[0]
            data = np.array(qtraj.cpu()).tolist()
            data.reverse()
            with open(f"traj_{i}.json", "w") as f:
                json.dump(data, f)
            #part_status[select_part_id] = 0
            #draw_world()
            draw_assembly(assembly, part_status, edge_width=1E-3)
            ps.reset_camera_to_home_view()

            itraj = -1
            draw_assembly(assembly, np.ones(len(assembly)))
            kin_model = get_franka_kin_model(robot_id, open_width)


        ps.set_user_callback(interface_ik_solver)
        ps.show()
        #break
