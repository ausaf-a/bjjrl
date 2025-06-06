import numpy as np
import mujoco
import json
from typing import Dict, List, Tuple
import xml.etree.ElementTree as ET
from xml.dom import minidom


class GrappleMapToMuJoCo:
    """Converts GrappleMap positions to MuJoCo XML format"""
    
    # Joint mapping from GrappleMap to body parts
    JOINT_TO_BODY = {
        # Core body
        20: 'torso',      # Core
        21: 'neck',       # Neck  
        22: 'head',       # Head
        
        # Left side
        8: 'left_hip',    # LeftHip
        6: 'left_knee',   # LeftKnee
        4: 'left_ankle',  # LeftAnkle
        10: 'left_shoulder',  # LeftShoulder
        12: 'left_elbow',     # LeftElbow
        14: 'left_wrist',     # LeftWrist
        
        # Right side
        9: 'right_hip',    # RightHip
        7: 'right_knee',   # RightKnee
        5: 'right_ankle',  # RightAnkle
        11: 'right_shoulder',  # RightShoulder
        13: 'right_elbow',     # RightElbow
        15: 'right_wrist',     # RightWrist
    }
    
    # Define body connections for creating capsules
    BODY_CONNECTIONS = [
        # Spine
        ('torso', 'neck'),
        ('neck', 'head'),
        
        # Left leg
        ('torso', 'left_hip'),
        ('left_hip', 'left_knee'),
        ('left_knee', 'left_ankle'),
        
        # Right leg
        ('torso', 'right_hip'),
        ('right_hip', 'right_knee'),
        ('right_knee', 'right_ankle'),
        
        # Left arm
        ('neck', 'left_shoulder'),
        ('left_shoulder', 'left_elbow'),
        ('left_elbow', 'left_wrist'),
        
        # Right arm
        ('neck', 'right_shoulder'),
        ('right_shoulder', 'right_elbow'),
        ('right_elbow', 'right_wrist'),
    ]
    
    def __init__(self, position_scale: float = 0.001):
        self.position_scale = position_scale
        
    def load_position(self, json_path: str, position_index: int = 0) -> Dict:
        """Load a specific position from grapplemap_processed.json"""
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        if position_index >= len(data):
            raise ValueError(f"Position index {position_index} out of range")
            
        return data["positions"][f"pos_{position_index}"]
    
    def parse_pose_data(self, position_data: Dict) -> Tuple[Dict, Dict]:
        """Extract joint positions for both players from position data"""
        poses = position_data['joints']
        
        # First 23 joints are player 0, next 23 are player 1
        player0_joints = {}
        player1_joints = {}
        
        for i in range(46):
            joint_idx = i % 23
            joint_pos = np.array(poses[i]) * self.position_scale
            
            if i < 23:
                player0_joints[joint_idx] = joint_pos
            else:
                player1_joints[joint_idx] = joint_pos
                
        return player0_joints, player1_joints
    
    def create_mjcf_xml(self, position_data: Dict, save_path: str = "bjj_scene.xml"):
        """Generate MuJoCo XML file from position data"""
        
        # Parse joint positions
        p0_joints, p1_joints = self.parse_pose_data(position_data)
        
        # Create root element
        root = ET.Element('mujoco', model=f"bjj_{position_data.get('description', 'scene')}")
        
        # Add compiler settings
        compiler = ET.SubElement(root, 'compiler', angle="radian", coordinate="local", inertiafromgeom="true")
        
        # Add options
        option = ET.SubElement(root, 'option', timestep="0.005", gravity="0 0 -9.81")
        ET.SubElement(option, 'flag', sensornoise="enable", contact="enable")
        
        # Add size settings
        size = ET.SubElement(root, 'size', njmax="1000", nconmax="100")
        
        # Add default settings
        default = ET.SubElement(root, 'default')
        
        # Joint defaults
        joint_default = ET.SubElement(default, 'joint', 
                                    limited="true", damping="1", armature="0.01")
        
        # Geometry defaults
        geom_default = ET.SubElement(default, 'geom', 
                                   contype="1", conaffinity="1", 
                                   friction="1.0 0.5 0.5",
                                   rgba="0.8 0.6 0.4 1")
        
        # Motor defaults for muscle-like control
        motor_default = ET.SubElement(default, 'motor', 
                                    ctrllimited="true", ctrlrange="0 1")
        
        # Add assets
        assets = ET.SubElement(root, 'asset')
        ET.SubElement(assets, 'texture', name="grid", type="2d", 
                     builtin="checker", rgb1="0.1 0.2 0.3", 
                     rgb2="0.2 0.3 0.4", width="300", height="300")
        ET.SubElement(assets, 'material', name="grid", texture="grid", 
                     texrepeat="8 8", reflectance="0.2")
        
        # Add worldbody
        worldbody = ET.SubElement(root, 'worldbody')
        
        # Add ground
        ET.SubElement(worldbody, 'geom', name="floor", type="plane", 
                     size="5 5 0.1", material="grid", condim="3")
        
        # Add lighting
        ET.SubElement(worldbody, 'light', name="spotlight", mode="trackcom", 
                     pos="0 0 5", diffuse="0.8 0.8 0.8")
        
        # Add players
        self._add_player(worldbody, p0_joints, player_id=0, color="0.8 0.2 0.2 1")
        self._add_player(worldbody, p1_joints, player_id=1, color="0.2 0.2 0.8 1")
        
        # Add actuators
        actuator = ET.SubElement(root, 'actuator')
        self._add_muscle_actuators(actuator, player_id=0)
        self._add_muscle_actuators(actuator, player_id=1)
        
        # Add sensors
        sensor = ET.SubElement(root, 'sensor')
        self._add_sensors(sensor)
        
        # Pretty print and save
        xml_string = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
        with open(save_path, 'w') as f:
            f.write(xml_string)
            
        return save_path
    
    def _add_player(self, worldbody: ET.Element, joints: Dict, 
                   player_id: int, color: str):
        """Add a player body to the worldbody"""
        
        # Create player body
        player_body = ET.SubElement(worldbody, 'body', 
                                   name=f"player{player_id}_torso",
                                   pos=" ".join(map(str, joints[20])))  # Core position
        
        # Add torso geometry
        ET.SubElement(player_body, 'geom', name=f"p{player_id}_torso", 
                     type="capsule", size="0.08 0.15", rgba=color)
        ET.SubElement(player_body, 'joint', name=f"p{player_id}_root", 
                     type="free")
        
        # Build body hierarchy
        self._build_body_tree(player_body, joints, player_id, color)
    
    def _build_body_tree(self, parent_body: ET.Element, joints: Dict, 
                        player_id: int, color: str):
        """Recursively build body tree from joint positions"""
        
        # This is simplified - in practice you'd want a proper kinematic tree
        # For now, let's add major body parts
        
        # Head
        head_pos = joints[22] - joints[20]  # Relative to torso
        head_body = ET.SubElement(parent_body, 'body', 
                                 name=f"player{player_id}_head",
                                 pos=" ".join(map(str, head_pos)))
        ET.SubElement(head_body, 'geom', name=f"p{player_id}_head", 
                     type="sphere", size="0.08", rgba=color)
        ET.SubElement(head_body, 'joint', name=f"p{player_id}_neck", 
                     type="ball", limited="false")
        
        # Add limbs (simplified for brevity)
        # In practice, you'd create the full kinematic chain
        
    def _add_muscle_actuators(self, actuator: ET.Element, player_id: int):
        """Add muscle-based actuators for a player"""
        
        # Example muscle actuators - you'd want many more for full control
        muscles = [
            ("hip_flexor", f"p{player_id}_hip_flex", "80"),
            ("hip_extensor", f"p{player_id}_hip_ext", "120"),
            ("knee_flexor", f"p{player_id}_knee_flex", "60"),
            ("knee_extensor", f"p{player_id}_knee_ext", "100"),
            ("shoulder_flexor", f"p{player_id}_shoulder_flex", "50"),
            ("shoulder_extensor", f"p{player_id}_shoulder_ext", "70"),
        ]
        
        for muscle_name, joint_name, force in muscles:
            ET.SubElement(actuator, 'muscle', 
                         name=f"p{player_id}_{muscle_name}",
                         joint=joint_name,
                         force=force,
                         range="-1 1",
                         lengthrange="0.5 1.5")
    
    def _add_sensors(self, sensor: ET.Element):
        """Add sensors for state observation"""
        
        # Joint position sensors
        for player_id in [0, 1]:
            # Add joint position and velocity sensors
            ET.SubElement(sensor, 'jointpos', 
                         name=f"p{player_id}_jointpos",
                         joint=f"p{player_id}_root")
            
            # Contact force sensors
            ET.SubElement(sensor, 'touch', 
                         name=f"p{player_id}_touch_torso",
                         site=f"p{player_id}_torso_site")


# Example usage
if __name__ == "__main__":
    # Initialize converter
    converter = GrappleMapToMuJoCo(position_scale=0.001)
    
    # Load a position (e.g., side control)
    position_data = converter.load_position("grapplemap_processed.json", 
                                          position_index=0)
    
    # Generate MuJoCo XML
    xml_path = converter.create_mjcf_xml(position_data, "bjj_side_control.xml")
    
    print(f"Generated MuJoCo XML: {xml_path}")
    print(f"Position: {position_data['description']}")
    print(f"Tags: {position_data['tags']}")