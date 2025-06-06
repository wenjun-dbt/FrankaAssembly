from flipparts.grasp import draw_gripper, sample_pick_and_place_poses, _sample_pick_poses
from flipparts import RESOURCES_DIR
import polyscope.imgui as psim
import polyscope as ps
import numpy as np
import trimesh
import json

def gripper_interface():
    global frame_id, pick_frames, place_frames, open_widths
    enable, frame_id = psim.SliderInt("frame id", frame_id, v_min=0, v_max=pick_frames.shape[0] - 1)
    if psim.IsKeyPressed(psim.ImGuiKey_RightArrow):
        frame_id = np.clip(frame_id + 1, 0, pick_frames.shape[0] - 1)
        enable = True
    if psim.IsKeyPressed(psim.ImGuiKey_LeftArrow):
        frame_id = np.clip(frame_id - 1, 0, pick_frames.shape[0] - 1)
        enable = True
    if enable:
        draw_gripper(pick_frames[frame_id, :, :], open_widths[frame_id], "pick")
        draw_gripper(place_frames[frame_id, :, :], open_widths[frame_id], "place")


grasp_mesh = trimesh.load_mesh(RESOURCES_DIR + "/tetris-byd-visual/2.obj")
#to world
with open(RESOURCES_DIR + "/transforms/2/object_transform_world.json", "r") as f:
        to_world = np.array(json.load(f)).reshape(4,4)
print(to_world)

mat = to_world
grasp_mesh.apply_transform(mat)
mat = np.linalg.inv(mat)

assembly = trimesh.Scene()
for id in range(3):
    part_mesh = trimesh.load_mesh(RESOURCES_DIR + f"/tetris-byd-visual/{id}.obj")
    assembly.add_geometry(part_mesh)
assembly = assembly.to_mesh()

ps.init()
ps.set_ground_plane_height(0)
# ps.set_ground_plane_mode("none")
ps.set_up_dir("z_up")
ps.set_front_dir("neg_y_front")
ps.register_surface_mesh("grasp_mesh", grasp_mesh.vertices, grasp_mesh.faces)
ps.register_surface_mesh("assembly", assembly.vertices, assembly.faces)


pick_frames, place_frames, open_widths = sample_pick_and_place_poses(mesh=grasp_mesh,
                                                                     num_of_samples=1000,
                                                                     check_ground=True,
                                                                     place_assembly=assembly,
                                                                     place_transform=mat)

# _, pick_frames, open_widths = _sample_pick_poses(mesh=grasp_mesh,
#                                         num_of_samples=1000,
#                                         gripper_width= 0.08,
#                                         seed = None)
#
#
# pick_frames = pick_frames.numpy()
# open_widths = open_widths.numpy()

frame_id = 0
draw_gripper(pick_frames[frame_id, :, :], open_widths[frame_id], "pick")
# draw_gripper(place_frames[frame_id, :, :], open_widths[frame_id], "place")
with open(RESOURCES_DIR + "/transforms/2/pick_frames.json", "w") as f:
    json.dump(pick_frames.tolist(), f)
ps.set_user_callback(gripper_interface)
ps.show()
