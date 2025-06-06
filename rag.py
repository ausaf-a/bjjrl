import os
import json
import argparse
import numpy as np
import xml.etree.ElementTree as ET
import mujoco
from mujoco import viewer
from collections import defaultdict

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

def build_tree():
    children = defaultdict(list)
    parent_info = {}
    for child, parent, jtype in LIMB_JOINTS:
        children[parent].append((child, jtype))
        parent_info[child] = (parent, jtype)
    return children, parent_info

def emit_ragdoll(idx, joints, name_prefix, color, children, parent_info, visited):
    if idx in visited:
        return None
    visited.add(idx)
    name, radius = joint_defs[idx]
    body = ET.Element("body", name=f"{name_prefix}_{name}", pos=vec_str(joints[idx]))
    # Assign a realistic mass for each body segment (approximate based on segment size)
    segment_mass = max(0.5, radius * 20)  # Heavier for bigger parts
    # Visualize joint as sphere, also assign mass here
    ET.SubElement(body, "geom", type="sphere", size=f"{radius*1.1}", rgba="0 0 0 0.3", mass=f"{segment_mass}")
    # If not root, add joint/capsule connecting to parent
    if idx in parent_info:
        parent, jtype = parent_info[idx]
        rel_pos = joints[idx] - joints[parent]
        joint_attrib = {
            "type": jtype,
            "name": f"{name_prefix}_{JOINT_NAMES[parent]}_to_{name_prefix}_{name}",
            "pos": "0 0 0"
        }
        if jtype == "hinge":
            joint_attrib["axis"] = "0 1 0"
        ET.SubElement(body, "joint", **joint_attrib)
        # Capsule from this joint to parent, in this local frame
        ET.SubElement(body, "geom",
            type="capsule",
            fromto=f"0 0 0 {vec_str(joints[parent]-joints[idx])}",
            size=f"{radius*0.9}",
            material=color,
            mass=f"{segment_mass}"
        )
    else:
        # Root: add freejoint so player is dynamic
        ET.SubElement(body, "freejoint")
    for child_idx, _ in children[idx]:
        child_body = emit_ragdoll(child_idx, joints, name_prefix, color, children, parent_info, visited)
        if child_body is not None:
            body.append(child_body)
    return body

def create_double_ragdoll_xml_et(joints1, joints2, filename):
    min_z = min(joints1[:,2].min(), joints2[:,2].min())
    floor_z = min_z - 0.02
    children, parent_info = build_tree()
    ET.register_namespace('', "http://mujoco.org")
    mujoco_elem = ET.Element("mujoco", attrib={"model": "double_ragdoll"})
    ET.SubElement(mujoco_elem, "option", gravity="0 0 -9.81", timestep="0.002")
    asset = ET.SubElement(mujoco_elem, "asset")
    ET.SubElement(asset, "material", name="red", rgba="1 0 0 1")
    ET.SubElement(asset, "material", name="blue", rgba="0 0 1 1")
    ET.SubElement(asset, "material", name="mat", rgba="0.8 0.8 0.8 1")
    ET.SubElement(asset, "texture", name="skybox", type="skybox", builtin="gradient",
                  rgb1="0.6 0.8 1", rgb2="0 0 0.3", width="256", height="256")
    worldbody = ET.SubElement(mujoco_elem, "worldbody")
    ET.SubElement(worldbody, "geom", type="plane", size="5 5 .1", pos=f"0 0 {floor_z:.4f}", material="mat")
    ET.SubElement(worldbody, "light", pos="0 0 8", dir="0 0 -1")
    # Player 1 (red)
    body1 = emit_ragdoll(JOINT_NAMES.index("Core"), joints1, "p1", "red", children, parent_info, set())
    worldbody.append(body1)
    # Player 2 (blue)
    body2 = emit_ragdoll(JOINT_NAMES.index("Core"), joints2, "p2", "blue", children, parent_info, set())
    worldbody.append(body2)
    tree = ET.ElementTree(mujoco_elem)
    tree.write(filename, encoding="utf-8", xml_declaration=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pos_name", help="e.g. pos_61")
    parser.add_argument("--json", default="grapplemap_processed.json")
    args = parser.parse_args()
    xml_file = f"{args.pos_name}_double_ragdoll.xml"
    if not os.path.isfile(xml_file):
        with open(args.json) as f:
            data = json.load(f)
        if args.pos_name not in data["positions"]:
            raise ValueError(f"Position {args.pos_name} not found in {args.json}")
        joint_data = np.array(data["positions"][args.pos_name]["joints"])
        joint_data1 = rotate_joints(joint_data[:NUM_JOINTS], axis="x", degrees=90)
        joint_data2 = rotate_joints(joint_data[NUM_JOINTS:NUM_JOINTS*2], axis="x", degrees=90)
        joint_data1[:,0] -= 0.1
        joint_data2[:,0] += 0.1
        create_double_ragdoll_xml_et(joint_data1, joint_data2, xml_file)
    m = mujoco.MjModel.from_xml_path(xml_file)
    d = mujoco.MjData(m)
    with viewer.launch_passive(m, d) as v:
        print(f"Loaded {xml_file}. Viewer is paused. Click play to run physics. Close window to exit.")
        while v.is_running():
            v.sync()

if __name__ == "__main__":
    main()
