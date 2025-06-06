import os
import json
import argparse
import numpy as np
import mujoco
from mujoco import viewer
import xml.etree.ElementTree as ET
from collections import defaultdict

# Joint radii only (spheres are visual, mass is almost zero)
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

# Define limb connections and assign realistic human limb masses (kg)
LIMB_MASS = {
    ("Core", "Neck"): 8.0,         # Upper torso
    ("Neck", "Head"): 4.0,         # Head
    ("Core", "LeftHip"): 7.0,     # Lower torso
    ("Core", "RightHip"): 7.0,    # Lower torso
    ("LeftHip", "LeftKnee"): 4.5, # Left thigh
    ("RightHip", "RightKnee"): 4.5, # Right thigh
    ("LeftKnee", "LeftAnkle"): 2.5, # Left shin
    ("RightKnee", "RightAnkle"): 2.5, # Right shin
    ("LeftAnkle", "LeftHeel"): 0.4,
    ("RightAnkle", "RightHeel"): 0.4,
    ("LeftHeel", "LeftToe"): 0.2,
    ("RightHeel", "RightToe"): 0.2,
    ("Core", "LeftShoulder"): 2.2, # Left clavicle
    ("Core", "RightShoulder"): 2.2, # Right clavicle
    ("LeftShoulder", "LeftElbow"): 1.7, # Left upper arm
    ("RightShoulder", "RightElbow"): 1.7, # Right upper arm
    ("LeftElbow", "LeftWrist"): 1.0, # Left forearm
    ("RightElbow", "RightWrist"): 1.0, # Right forearm
    ("LeftWrist", "LeftHand"): 0.4,
    ("RightWrist", "RightHand"): 0.4,
    ("LeftHand", "LeftFingers"): 0.2,
    ("RightHand", "RightFingers"): 0.2
}

# Kinematic tree (same as before)
kinematic_tree = {
    "LeftToe": "LeftHeel", "LeftHeel": "LeftAnkle", "LeftAnkle": "LeftKnee", "LeftKnee": "LeftHip", "LeftHip": "Core",
    "RightToe": "RightHeel", "RightHeel": "RightAnkle", "RightAnkle": "RightKnee", "RightKnee": "RightHip", "RightHip": "Core",
    "Neck": "Core", "Head": "Neck",
    "LeftShoulder": "Core", "LeftElbow": "LeftShoulder", "LeftWrist": "LeftElbow", "LeftHand": "LeftWrist", "LeftFingers": "LeftHand",
    "RightShoulder": "Core", "RightElbow": "RightShoulder", "RightWrist": "RightElbow", "RightHand": "RightWrist", "RightFingers": "RightHand"
}
child_map = defaultdict(list)
for child, parent in kinematic_tree.items():
    child_map[parent].append(child)

def create_body_recursive(joint_name, joints, player_name, color, material_name, joint_idx, joint_list, parent_pos=None, parent_joint=None):
    idx = joint_idx[joint_name]
    pos = joints[idx]
    rel_pos = pos if parent_pos is None else pos - parent_pos
    body = ET.Element("body", name=f"{player_name}_{joint_name}", pos=f"{rel_pos[0]:.4f} {rel_pos[1]:.4f} {rel_pos[2]:.4f}")
    # Spheres are visual (almost no mass)
    ET.SubElement(body, "geom", type="sphere", size=str(joint_defs[idx][1]), mass="0.01", material=material_name)
    # Capsule from parent to here: only if not root
    if parent_joint is not None:
        cap_size = min(joint_defs[idx][1], joint_defs[joint_idx[parent_joint]][1]) * 0.7
        limb_key = (parent_joint, joint_name)
        mass = LIMB_MASS.get(limb_key, 0.1)  # fallback to tiny mass
        rel = rel_pos
        if np.linalg.norm(rel) > 1e-4:
            ET.SubElement(body, "geom", type="capsule",
                fromto=f"0 0 0 {rel[0]:.4f} {rel[1]:.4f} {rel[2]:.4f}",
                size=f"{cap_size:.4f}", mass=f"{mass}", material=material_name)
    # Add joint (if not root)
    if joint_name != "Core":
        joint_name_full = f"{player_name}_{joint_name}_joint"
        joint_list.append(joint_name_full)
        if "Shoulder" in joint_name or "Hip" in joint_name:
            ET.SubElement(body, "joint", name=joint_name_full, type="ball", limited="false", damping="0.5")
        elif "Knee" in joint_name or "Elbow" in joint_name:
            ET.SubElement(body, "joint", name=joint_name_full, type="hinge", axis="0 1 0", limited="false", damping="0.5", ref="0")
        else:
            ET.SubElement(body, "joint", name=joint_name_full, type="hinge", axis="1 0 0", limited="false", damping="0.5")
    else:
        ET.SubElement(body, "freejoint", name=f"{player_name}_root")
    for child in child_map.get(joint_name, []):
        child_body = create_body_recursive(child, joints, player_name, color, material_name, joint_idx, joint_list, parent_pos=pos, parent_joint=joint_name)
        body.append(child_body)
    return body

def create_physics_humanoid_xml(joints, player_name, color, material_name, floor_z=-1.0):
    ET.register_namespace('', "http://mujoco.org")
    mujoco_elem = ET.Element("mujoco", attrib={"model": f"bjj_{player_name}"})
    compiler = ET.SubElement(mujoco_elem, "compiler", angle="radian", coordinate="local")
    default = ET.SubElement(mujoco_elem, "default")
    ET.SubElement(default, "joint", armature="0.01", damping="0.1", limited="true")
    ET.SubElement(default, "geom", contype="1", conaffinity="1", friction="1 0.5 0.5")
    asset = ET.SubElement(mujoco_elem, "asset")
    ET.SubElement(asset, "material", name=material_name, rgba=color)
    worldbody = ET.SubElement(mujoco_elem, "worldbody")
    ET.SubElement(worldbody, "geom", name="floor", type="plane", pos=f"0 0 {floor_z:.4f}", size="50 50 0.1", rgba="0.8 0.8 0.8 1")
    joint_idx = {name: i for i, (name, _) in enumerate(joint_defs)}
    joint_list = []
    core_body = create_body_recursive("Core", joints, player_name, color, material_name, joint_idx, joint_list)
    worldbody.append(core_body)
    actuator = ET.SubElement(mujoco_elem, "actuator")
    for jname in joint_list:
        ET.SubElement(actuator, "position", name=f"{jname}_act", joint=jname, kp="50")
    return mujoco_elem

def create_combined_scene(joint_data, pos_name):
    ET.register_namespace('', "http://mujoco.org")
    mujoco_elem = ET.Element("mujoco", attrib={"model": "bjj_scene"})
    compiler = ET.SubElement(mujoco_elem, "compiler", angle="radian", coordinate="local")
    option = ET.SubElement(mujoco_elem, "option", timestep="0.002", gravity="0 0 -.981")
    visual = ET.SubElement(mujoco_elem, "visual")
    ET.SubElement(visual, "rgba", haze="0.15 0.25 0.35 1")
    default = ET.SubElement(mujoco_elem, "default")
    ET.SubElement(default, "joint", armature="0.01", damping="0.1", limited="true")
    ET.SubElement(default, "geom", contype="1", conaffinity="1", friction="1 0.5 0.5", margin="0.001")
    asset = ET.SubElement(mujoco_elem, "asset")
    ET.SubElement(asset, "material", name="red", rgba="0.8 0.2 0.2 1")
    ET.SubElement(asset, "material", name="blue", rgba="0.2 0.2 0.8 1")
    ET.SubElement(asset, "material", name="mat_ground", rgba="0.5 0.5 0.5 1")
    worldbody = ET.SubElement(mujoco_elem, "worldbody")
    ET.SubElement(worldbody, "light", directional="true", pos="0 0 3", dir="0 0 -1")
    min_z = np.min(joint_data[:,2])
    floor_z = min_z - 0.05
    joint_idx = {name: i for i, (name, _) in enumerate(joint_defs)}
    p1_joints = []
    p2_joints = []
    player1_xml = create_body_recursive("Core", joint_data[0:NUM_JOINTS], "player1", "0.8 0.2 0.2 1", "red", joint_idx, p1_joints)
    player2_xml = create_body_recursive("Core", joint_data[NUM_JOINTS:NUM_JOINTS*2], "player2", "0.2 0.2 0.8 1", "blue", joint_idx, p2_joints)
    worldbody.append(player1_xml)
    worldbody.append(player2_xml)
    actuator = ET.SubElement(mujoco_elem, "actuator")
    for jname in p1_joints + p2_joints:
        ET.SubElement(actuator, "position", name=f"{jname}_act", joint=jname, kp="50")
    tree = ET.ElementTree(mujoco_elem)
    fname = f"{pos_name}_scene.xml"
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
    parser = argparse.ArgumentParser(description="Create physics-enabled BJJ humanoids from GrappleMap data")
    parser.add_argument("pos_name", help="Position name (e.g. pos_61)")
    parser.add_argument("--json", default="grapplemap_processed.json", help="GrappleMap JSON file")
    parser.add_argument("--player", type=int, choices=[1, 2], help="View only player 1 or 2")
    parser.add_argument("--scale", type=float, default=0.01, help="Scale factor for positions")
    args = parser.parse_args()
    with open(args.json) as f:
        data = json.load(f)
    if args.pos_name not in data["positions"]:
        raise ValueError(f"Position {args.pos_name} not found in {args.json}")
    joint_data = np.array(data["positions"][args.pos_name]["joints"])
    joint_data = joint_data * args.scale
    joint_data = rotate_joints(joint_data, axis="x", degrees=90)
    min_z = np.min(joint_data[:,2])
    floor_z = min_z - 0.05
    if args.player == 1:
        xml_elem = create_physics_humanoid_xml(joint_data[0:NUM_JOINTS], "player1", "0.8 0.2 0.2 1", "red", floor_z)
        xml_file = f"{args.pos_name}_player1.xml"
        tree = ET.ElementTree(xml_elem)
        tree.write(xml_file, encoding="utf-8", xml_declaration=True)
    elif args.player == 2:
        xml_elem = create_physics_humanoid_xml(joint_data[NUM_JOINTS:NUM_JOINTS*2], "player2", "0.2 0.2 0.8 1", "blue", floor_z)
        xml_file = f"{args.pos_name}_player2.xml"
        tree = ET.ElementTree(xml_elem)
        tree.write(xml_file, encoding="utf-8", xml_declaration=True)
    else:
        xml_file = create_combined_scene(joint_data, args.pos_name)
    print(f"Loading {xml_file}...")
    m = mujoco.MjModel.from_xml_path(xml_file)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    with viewer.launch(m, d) as v:
        print(f"Viewing {xml_file}")
        print("Controls:")
        print("  - Space: pause/unpause simulation")
        print("  - Right click + drag: rotate camera")
        print("  - Scroll: zoom")
        print("  - Shift + right click: pan")
        print("  - Close window to exit")
        while v.is_running():
            mujoco.mj_step(m, d)
            v.sync()

if __name__ == "__main__":
    main()
