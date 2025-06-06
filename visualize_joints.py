import os
import json
import argparse
import numpy as np
import mujoco
from mujoco import viewer
import xml.etree.ElementTree as ET

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

def joints_to_xml_et(joints, filename, color, material_name):
    ET.register_namespace('', "http://mujoco.org")
    mujoco_elem = ET.Element("mujoco", attrib={"model": "bjj_player"})
    asset = ET.SubElement(mujoco_elem, "asset")
    ET.SubElement(asset, "material", name=material_name, rgba=color)
    worldbody = ET.SubElement(mujoco_elem, "worldbody")
    for i, (name, radius) in enumerate(joint_defs):
        pos = joints[i]
        body = ET.SubElement(worldbody, "body", name=name, pos=f"{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}")
        ET.SubElement(body, "geom", type="capsule", size=f"{radius} {radius}", material=material_name)
    tree = ET.ElementTree(mujoco_elem)
    tree.write(filename, encoding="utf-8", xml_declaration=True)

def load_or_create_xml_et(joint_data, pos_name):
    fname = f"{pos_name}.xml"
    if os.path.isfile(fname):
        return fname
    # Assume shape (46, 3): first 23 = player 1, next 23 = player 2
    p1_file = f"{pos_name}_p1.xml"
    p2_file = f"{pos_name}_p2.xml"
    joints_to_xml_et(joint_data[0:NUM_JOINTS], p1_file, "1 0 0 1", "red")
    joints_to_xml_et(joint_data[NUM_JOINTS:NUM_JOINTS*2], p2_file, "0 0 1 1", "blue")
    # Combine both into one MJCF file (for viewing both players)
    ET.register_namespace('', "http://mujoco.org")
    mujoco_elem = ET.Element("mujoco", attrib={"model": "bjj_pair"})
    asset = ET.SubElement(mujoco_elem, "asset")
    ET.SubElement(asset, "material", name="red", rgba="1 0 0 1")
    ET.SubElement(asset, "material", name="blue", rgba="0 0 1 1")
    worldbody = ET.SubElement(mujoco_elem, "worldbody")
    for i, (name, radius) in enumerate(joint_defs):
        pos = joint_data[i]
        body = ET.SubElement(worldbody, "body", name=f"p1_{name}", pos=f"{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}")
        ET.SubElement(body, "geom", type="capsule", size=f"{radius} {radius}", material="red")
    for i, (name, radius) in enumerate(joint_defs):
        pos = joint_data[i+NUM_JOINTS]
        body = ET.SubElement(worldbody, "body", name=f"p2_{name}", pos=f"{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}")
        ET.SubElement(body, "geom", type="capsule", size=f"{radius} {radius}", material="blue")
    tree = ET.ElementTree(mujoco_elem)
    tree.write(fname, encoding="utf-8", xml_declaration=True)
    return fname

def rotate_joints(joints, axis="", degrees=90):
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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pos_name", help="e.g. pos_61")
    parser.add_argument("--json", default="grapplemap_processed.json", help="GrappleMap JSON file")
    args = parser.parse_args()
    xml_file = f"{args.pos_name}.xml"
    if not os.path.isfile(xml_file):
        with open(args.json) as f:
            data = json.load(f)
        if args.pos_name not in data["positions"]:
            raise ValueError(f"Position {args.pos_name} not found in {args.json}")
        joint_data = np.array(data["positions"][args.pos_name]["joints"])
        joint_data = rotate_joints(joint_data, axis="x", degrees=90)
        xml_file = load_or_create_xml_et(joint_data, args.pos_name)
    # Load and view
    m = mujoco.MjModel.from_xml_path(xml_file)
    d = mujoco.MjData(m)
    with viewer.launch_passive(m, d) as v:
        print(f"Loaded {xml_file}. Viewer is paused. Close window to exit.")
        while v.is_running():
            v.sync()

if __name__ == "__main__":
    main()
