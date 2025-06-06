import os
import json
import argparse
import numpy as np
import xml.etree.ElementTree as ET
import mujoco
from mujoco import viewer

joint_defs = [
    ("LeftToe", 0.025), ("RightToe", 0.025),
    ("LeftHeel", 0.03), ("RightHeel", 0.03),
    ("LeftAnkle", 0.03), ("RightAnkle", 0.03),
    ("LeftKnee", 0.05), ("RightKnee", 0.05),
    ("LeftHip", 0.09), ("RightHip", 0.09),
    ("LeftShoulder", 0.08), ("RightShoulder", 0.08),
    ("LeftElbow", 0.045), ("RightElbow", 0.045),
    ("LeftWrist", 0.02), ("RightWrist", 0.02),
    ("LeftHand", 0.02), ("RightHand", 0.02),
    ("LeftFingers", 0.02), ("RightFingers", 0.02),
    ("Core", 0.1), ("Neck", 0.05), ("Head", 0.11)
]
JOINT_NAMES = [name for name, _ in joint_defs]
NUM_JOINTS = len(JOINT_NAMES)

LIMB_JOINTS = [
    (0, 2, "hinge"), (1, 3, "hinge"), (2, 4, "hinge"), (3, 5, "hinge"),
    (4, 6, "hinge"), (5, 7, "hinge"), (6, 8, "hinge"), (7, 9, "hinge"),
    (8, 20, "ball"), (9, 20, "ball"), (20, 21, "ball"), (21, 22, "ball"),
    (10, 20, "ball"), (11, 20, "ball"), (10, 12, "hinge"), (11, 13, "hinge"),
    (12, 14, "hinge"), (13, 15, "hinge"), (14, 16, "hinge"), (15, 17, "hinge"),
    (16, 18, "hinge"), (17, 19, "hinge"), (21, 10, "ball"), (21, 11, "ball"),
    (8, 9, "ball"), (0, 4, "hinge"), (1, 5, "hinge")
]

def rotate_joints(joints, axis="x", degrees=90):
    theta = np.radians(degrees)
    if axis == "x":
        rot = np.array([[1, 0, 0],
                        [0, np.cos(theta), -np.sin(theta)],
                        [0, np.sin(theta),  np.cos(theta)]])
    elif axis == "y":
        rot = np.array([[ np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])
    elif axis == "z":
        rot = np.array([[np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta),  np.cos(theta), 0],
                        [0, 0, 1]])
    else:
        raise ValueError("axis must be 'x', 'y', or 'z'")
    return np.dot(joints, rot.T)

def vec_str(v):
    return " ".join([f"{x:.4f}" for x in v])

def create_stickman_xml_et(joints, filename):
    # Center at core
    core_idx = JOINT_NAMES.index("Core")
    joints = joints - joints[core_idx]
    ET.register_namespace('', "http://mujoco.org")

    mujoco_elem = ET.Element("mujoco", attrib={"model": "stickman"})
    option = ET.SubElement(mujoco_elem, "option", gravity="0 0 -9.81", timestep="0.002")
    asset = ET.SubElement(mujoco_elem, "asset")
    ET.SubElement(asset, "material", name="red", rgba="1 0 0 1")
    ET.SubElement(asset, "material", name="mat", rgba="0.8 0.8 0.8 1")
    ET.SubElement(asset, "texture", name="skybox", type="skybox", builtin="gradient",
                  rgb1="0.6 0.8 1", rgb2="0 0 0.3", width="256", height="256")
    worldbody = ET.SubElement(mujoco_elem, "worldbody")

    # Compute lowest Z of any joint for the floor
    min_z = joints[:, 2].min()
    floor_z = min_z - 0.02
    ET.SubElement(worldbody, "geom", type="plane", size="5 5 .1", pos=f"0 0 {floor_z:.4f}", material="mat")
    ET.SubElement(worldbody, "light", pos="0 0 8", dir="0 0 -1")

    # For visual debugging: spheres at joints
    for idx, (name, radius) in enumerate(joint_defs):
        ET.SubElement(worldbody, "geom", type="sphere", pos=vec_str(joints[idx]), size=f"{radius*1.2}", rgba="0 0 0 0.5")

    core_body = ET.SubElement(worldbody, "body", name="Core", pos=vec_str(joints[core_idx]))
    ET.SubElement(core_body, "freejoint")

    for child, parent, jtype in LIMB_JOINTS:
        from_pos = joints[parent]
        to_pos = joints[child]
        rad = joint_defs[child][1] * 0.9
        ET.SubElement(core_body, "geom", type="capsule",
                      fromto=f"{vec_str(from_pos)} {vec_str(to_pos)}",
                      size=f"{rad}", material="red")
    tree = ET.ElementTree(mujoco_elem)
    tree.write(filename, encoding="utf-8", xml_declaration=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pos_name", help="e.g. pos_61")
    parser.add_argument("--json", default="grapplemap_processed.json")
    args = parser.parse_args()
    xml_file = f"{args.pos_name}_stickman.xml"
    if not os.path.isfile(xml_file):
        with open(args.json) as f:
            data = json.load(f)
        if args.pos_name not in data["positions"]:
            raise ValueError(f"Position {args.pos_name} not found in {args.json}")
        joint_data = np.array(data["positions"][args.pos_name]["joints"])
        joint_data = rotate_joints(joint_data[:NUM_JOINTS], axis="x", degrees=90)
        create_stickman_xml_et(joint_data, xml_file)
    m = mujoco.MjModel.from_xml_path(xml_file)
    d = mujoco.MjData(m)
    with viewer.launch_passive(m, d) as v:
        print(f"Loaded {xml_file}. Viewer is paused. Click play to run physics. Close window to exit.")
        while v.is_running():
            v.sync()

if __name__ == "__main__":
    main()
