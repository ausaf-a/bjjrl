import bpy
import json
import os
from mathutils import Matrix, Vector
from math import sin, cos

def ncross(a, b):
    return a.normalized().cross(b.normalized())

def ortho(a, b, c):  # returns a vector orthogonal to both (b-a) and (c-a)
    return ncross(b - a, c - a)

def lerp(a, b, c=0.5):
    return a * (1 - c) + b * c

def assignBone(poseBone, matrix, armaBone, scale):
    mi = armaBone.matrix.copy()
    mi.invert()
    poseBone.rotation_mode = 'XYZ'
    poseBone.rotation_euler = (mi * matrix).to_euler()
    poseBone.scale = Vector((scale, scale, scale))

class GrappleMapData:
    def __init__(self, json_file_path):
        self.json_file_path = json_file_path
        self.data = None
        self.positions = []
        self.load_data()
    
    def load_data(self):
        """Load the JSON data"""
        try:
            with open(self.json_file_path, 'r') as f:
                self.data = json.load(f)
            print(f"Loaded GrappleMap data with {len(self.data['positions'])} positions")
        except Exception as e:
            print(f"Error loading JSON: {e}")
            return False
        return True
    
    def decode_joints_from_position(self, position_data):
        """Convert position joint data to Vector format"""
        joints = []
        joint_data = position_data['joints']
        
        for joint in joint_data:
            if len(joint) >= 3:
                # Convert to Blender coordinate system (may need adjustment)
                vec = Vector((joint[0], joint[1], joint[2]))
                joints.append(vec)
        
        return joints
    
    def get_position_by_id(self, position_id):
        """Get a specific position by ID"""
        if self.data and 'positions' in self.data:
            return self.data['positions'].get(position_id)
        return None
    
    def get_all_positions(self):
        """Get all positions as a list"""
        if self.data and 'positions' in self.data:
            return list(self.data['positions'].values())
        return []
    
    def get_transition_sequence(self, transition_id):
        """Get a transition sequence"""
        if self.data and 'transitions' in self.data:
            transition = self.data['transitions'].get(transition_id)
            if transition and 'sequence' in transition:
                # Convert sequence to joint vectors
                sequence = []
                for frame in transition['sequence']:
                    frame_joints = []
                    for joint in frame:
                        if len(joint) >= 3:
                            vec = Vector((joint[0], joint[1], joint[2]))
                            frame_joints.append(vec)
                    sequence.append(frame_joints)
                return sequence
        return []

def readPlayer(joint_data, start_index=0):
    """Convert joint data to player position dict (matching original script format)"""
    pos = {}
    
    # Map from GrappleMap joint indices to bone names
    joint_mapping = [
        'lefttoe', 'righttoe', 'leftheel', 'rightheel', 
        'leftankle', 'rightankle', 'leftknee', 'rightknee',
        'lefthip', 'righthip', 'leftshoulder', 'rightshoulder',
        'leftelbow', 'rightelbow', 'leftwrist', 'rightwrist',
        'lefthand', 'righthand', 'leftfingers', 'rightfingers',
        'core', 'neck', 'head'
    ]
    
    # Extract 23 joints for this player
    for i, bone_name in enumerate(joint_mapping):
        joint_index = start_index + i
        if joint_index < len(joint_data):
            pos[bone_name] = joint_data[joint_index]
    
    # Apply the same adjustments as the original script
    if 'lefthip' in pos and 'righthip' in pos and 'core' in pos:
        sideways = (pos['righthip'] - pos['lefthip']).normalized()
        pos['lefthip'] -= sideways * 0.01
        pos['righthip'] += sideways * 0.01
        pos['lefthip'] = lerp(pos['lefthip'], pos['core'], 0.2)
        pos['righthip'] = lerp(pos['righthip'], pos['core'], 0.2)
        pos['hips'] = lerp(pos['lefthip'], pos['righthip'])
        pos['core'] = lerp(pos['core'], pos['neck'], 0.12)
        pos['neck'] = lerp(pos['neck'], pos['head'], 0.25)
    
    return pos

def cmu_enrich(p):
    """Add additional bone positions (from original script)"""
    if not all(key in p for key in ['lefthip', 'righthip', 'core', 'neck', 'head', 'leftshoulder', 'rightshoulder']):
        print("Missing required bones for cmu_enrich")
        return
    
    p['cmu_hips'] = lerp(p['lefthip'], p['righthip'])
    hipsfwd = ortho(p['cmu_hips'], p['core'], p['righthip'])
    corefwd = ortho(p['core'], p['leftshoulder'], p['rightshoulder'])

    p['cmu_lowerback'] = lerp(p['cmu_hips'], p['core'], 0.35) + hipsfwd * -0.04
    p['cmu_spine1'] = lerp(p['core'], p['neck'], 0.4) + corefwd * -0.07
    p['cmu_neck1'] = lerp(p['head'], p['neck'])

    # Continue with the rest of the original cmu_enrich function...
    # (This is a complex function - keeping the core parts for now)

def gm_to_cmu(p):
    """Convert GrappleMap position to CMU bone structure (from original script)"""
    cmu_enrich(p)
    
    cmu = {}
    
    # Implement the bone conversion logic from the original script
    # This is quite complex - starting with basic conversions
    
    if 'cmu_lowerback' in p and 'hips' in p:
        fwdR = ortho(p['cmu_lowerback'], p['righthip'], p['lefthip'])
        ll = ncross(p['cmu_lowerback'] - p['hips'], fwdR)
        cmu['Hips'] = (p['cmu_lowerback'] - p['hips'], ll)
    
    # Add more bone conversions as needed...
    
    return cmu

def create_armature_for_grapplemap():
    """Create a basic armature that matches GrappleMap bone structure"""
    
    # Clear existing mesh objects
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    
    # Create armature
    bpy.ops.object.armature_add(enter_editmode=True, location=(0, 0, 0))
    armature = bpy.context.object
    armature.name = "GrappleMapRig"
    
    # We'll create a simplified rig for now
    # In edit mode, we can add bones programmatically
    
    bpy.ops.object.mode_set(mode='OBJECT')
    return armature

def animate_position(grapplemap_data, position_id, armature_name="GrappleMapRig"):
    """Animate a single position"""
    
    position_data = grapplemap_data.get_position_by_id(position_id)
    if not position_data:
        print(f"Position {position_id} not found")
        return
    
    print(f"Animating position: {position_data.get('description', 'Unknown')}")
    
    # Decode joint data
    joint_data = grapplemap_data.decode_joints_from_position(position_data)
    
    if len(joint_data) < 46:  # Should be 46 joints (23 per player)
        print(f"Insufficient joint data: {len(joint_data)} joints")
        return
    
    # Extract player data
    red_player = readPlayer(joint_data, 0)    # First 23 joints
    blue_player = readPlayer(joint_data, 23)  # Next 23 joints
    
    print(f"Red player bones: {list(red_player.keys())}")
    print(f"Blue player bones: {list(blue_player.keys())}")
    
    # Apply to armatures (this would need actual armatures set up)
    # For now, just print the data
    print(f"Red player hip position: {red_player.get('hips', 'Not found')}")
    print(f"Blue player hip position: {blue_player.get('hips', 'Not found')}")

def animate_transition(grapplemap_data, transition_id):
    """Animate a transition sequence"""
    
    sequence = grapplemap_data.get_transition_sequence(transition_id)
    if not sequence:
        print(f"Transition {transition_id} not found")
        return
    
    print(f"Animating transition with {len(sequence)} keyframes")
    
    for frame_num, frame_joints in enumerate(sequence):
        print(f"Frame {frame_num}: {len(frame_joints)} joints")
        # Apply frame data to armatures...

def main():
    """Main function to run the importer"""
    
    # Path to your JSON file - adjust this path!
    json_path = "./grapplemap_processed.json"
    
    # Check if file exists
    if not os.path.exists(json_path):
        print(f"JSON file not found at: {json_path}")
        print("Please update the json_path variable with the correct path to your grapplemap_processed.json file")
        return
    
    # Load data
    grapplemap_data = GrappleMapData(json_path)
    
    if not grapplemap_data.data:
        print("Failed to load GrappleMap data")
        return
    
    # Create basic armature
    armature = create_armature_for_grapplemap()
    
    # Get first position for testing
    positions = grapplemap_data.get_all_positions()
    if positions:
        first_position = positions[0]
        print(f"Testing with first position: {first_position.get('description', 'Unknown')}")
        animate_position(grapplemap_data, first_position['id'])
    
    print("Import complete!")

if __name__ == "__main__":
    main()