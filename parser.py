import json
from typing import List, Dict, Optional, Union, Tuple
from pathlib import Path
from enum import IntEnum
import numpy as np
from pydantic import BaseModel, Field, validator


# Joint hierarchy and limb definitions
LIMBS = {
    "LeftToe": "LeftHeel",
    "LeftHeel": "LeftAnkle",
    "LeftAnkle": "LeftKnee",
    "LeftKnee": "LeftHip",
    "LeftHip": "Core",
    "RightToe": "RightHeel",
    "RightHeel": "RightAnkle",
    "RightAnkle": "RightKnee",
    "RightKnee": "RightHip",
    "RightHip": "Core",
    "Neck": "Core",
    "Head": "Neck",
    "LeftShoulder": "Neck",
    "LeftElbow": "LeftShoulder",
    "LeftWrist": "LeftElbow",
    "LeftHand": "LeftWrist",
    "LeftFingers": "LeftHand",
    "RightShoulder": "Neck",
    "RightElbow": "RightShoulder",
    "RightWrist": "RightElbow",
    "RightHand": "RightWrist",
    "RightFingers": "RightHand"
}

NAMED_LIMBS = {
    "LeftFoot": ["LeftToe", "LeftHeel"],
    "LeftAnkle": ["LeftHeel", "LeftAnkle"],
    "LeftShin": ["LeftAnkle", "LeftKnee"],
    "LeftThigh": ["LeftKnee", "LeftHip"],
    "RightFoot": ["RightToe", "RightHeel"],
    "RightAnkle": ["RightHeel", "RightAnkle"],
    "RightShin": ["RightAnkle", "RightKnee"],
    "RightThigh": ["RightKnee", "RightHip"],
    "LeftArm": ["LeftShoulder", "LeftElbow"],
    "LeftForearm": ["LeftElbow", "LeftWrist"],
    "LeftHand": ["LeftWrist", "LeftHand"],
    "LeftFingers": ["LeftHand", "LeftFingers"],
    "RightArm": ["RightShoulder", "RightElbow"],
    "RightForearm": ["RightElbow", "RightWrist"],
    "RightHand": ["RightWrist", "RightHand"],
    "RightFingers": ["RightHand", "RightFingers"],
    "Core": ["Core", "Neck"],
    "Neck": ["Neck", "Head"]
}


class JointName(IntEnum):
    """Enum for joint names with their corresponding indices"""
    LeftToe = 0
    RightToe = 1
    LeftHeel = 2
    RightHeel = 3
    LeftAnkle = 4
    RightAnkle = 5
    LeftKnee = 6
    RightKnee = 7
    LeftHip = 8
    RightHip = 9
    LeftShoulder = 10
    RightShoulder = 11
    LeftElbow = 12
    RightElbow = 13
    LeftWrist = 14
    RightWrist = 15
    LeftHand = 16
    RightHand = 17
    LeftFingers = 18
    RightFingers = 19
    Core = 20
    Neck = 21
    Head = 22
    
    @classmethod
    def get_joint_names(cls) -> List[str]:
        """Get all joint names in order"""
        return [member.name for member in cls]
    
    @classmethod
    def from_name(cls, name: str) -> 'JointName':
        """Get joint enum from string name"""
        return cls[name]


class Joint(BaseModel):
    """A 3D joint position"""
    x: float
    y: float 
    z: float
    
    @classmethod
    def from_list(cls, coords: List[float]) -> 'Joint':
        """Create Joint from [x, y, z] list"""
        return cls(x=coords[0], y=coords[1], z=coords[2])
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array"""
        return np.array([self.x, self.y, self.z])
    
    def __iter__(self):
        """Make Joint iterable for unpacking"""
        yield self.x
        yield self.y
        yield self.z


class Player(BaseModel):
    """A player with 23 joints"""
    joints: List[Joint] = Field(..., min_items=23, max_items=23)
    
    @classmethod
    def from_joint_data(cls, joint_data: List[List[float]]) -> 'Player':
        """Create Player from raw joint coordinate data"""
        joints = [Joint.from_list(coords) for coords in joint_data]
        return cls(joints=joints)
    
    def to_array(self) -> np.ndarray:
        """Convert all joints to numpy array of shape (23, 3)"""
        return np.array([joint.to_array() for joint in self.joints])
    
    def get_joint(self, joint: Union[int, str, JointName]) -> Joint:
        """Get joint by index, name string, or JointName enum"""
        if isinstance(joint, str):
            joint = JointName.from_name(joint)
        elif isinstance(joint, JointName):
            joint = joint.value
        return self.joints[joint]
    
    def get_joint_position(self, joint: Union[int, str, JointName]) -> np.ndarray:
        """Get joint position as numpy array"""
        return self.get_joint(joint).to_array()
    
    def get_limb_length(self, limb_name: str) -> float:
        """Calculate length of a named limb using NAMED_LIMBS"""
        if limb_name not in NAMED_LIMBS:
            raise ValueError(f"Unknown limb: {limb_name}. Available: {list(NAMED_LIMBS.keys())}")
        
        joint1_name, joint2_name = NAMED_LIMBS[limb_name]
        joint1_pos = self.get_joint_position(joint1_name)
        joint2_pos = self.get_joint_position(joint2_name)
        return np.linalg.norm(joint1_pos - joint2_pos)
    
    def get_all_limb_lengths(self) -> Dict[str, float]:
        """Get lengths of all named limbs"""
        return {limb_name: self.get_limb_length(limb_name) for limb_name in NAMED_LIMBS.keys()}
    
    def __getitem__(self, joint: Union[int, str, JointName]) -> Joint:
        """Allow indexing by joint name or index: player[JointName.Core] or player['Core']"""
        return self.get_joint(joint)
    
    # Convenience properties for common joints
    @property
    def core(self) -> Joint:
        return self.joints[JointName.Core]
    
    @property
    def head(self) -> Joint:
        return self.joints[JointName.Head]
    
    @property
    def left_hand(self) -> Joint:
        return self.joints[JointName.LeftHand]
    
    @property
    def right_hand(self) -> Joint:
        return self.joints[JointName.RightHand]
    
    @property
    def left_foot(self) -> Joint:
        return self.joints[JointName.LeftToe]
    
    @property
    def right_foot(self) -> Joint:
        return self.joints[JointName.RightToe]


class GrappleMapPosition(BaseModel):
    """A BJJ position with two players"""
    id: str
    description: str = ""
    tags: List[str] = Field(default_factory=list)
    player1: Player
    player2: Player
    
    @classmethod
    def from_raw_data(cls, position_id: str, data: Dict) -> 'GrappleMapPosition':
        """Create position from raw GrappleMap data"""
        joints = data.get('joints', [])
        if len(joints) < 46:  # 23 joints per player * 2 players
            raise ValueError(f"Position {position_id} has insufficient joint data: {len(joints)} joints")
        
        # Split joints between two players
        player1_joints = joints[:23]
        player2_joints = joints[23:46]
        
        return cls(
            id=position_id,
            description=data.get('description', ''),
            tags=data.get('tags', []),
            player1=Player.from_joint_data(player1_joints),
            player2=Player.from_joint_data(player2_joints)
        )
    
    def get_combined_joints(self) -> np.ndarray:
        """Get all joints as single numpy array (46, 3)"""
        return np.vstack([self.player1.to_array(), self.player2.to_array()])
    
    def contains_tag(self, tag: str) -> bool:
        """Check if position contains a specific tag"""
        return tag.lower() in [t.lower() for t in self.tags]
    
    def contains_description(self, query: str) -> bool:
        """Check if description contains query string"""
        return query.lower() in self.description.lower()


class Transition(BaseModel):
    """A transition between positions"""
    id: str
    from_position: str
    to_position: str
    description: str = ""
    
    @classmethod
    def from_raw_data(cls, transition_id: str, data: Dict) -> 'Transition':
        """Create transition from raw GrappleMap data"""
        return cls(
            id=transition_id,
            from_position=data.get('from', ''),
            to_position=data.get('to', ''),
            description=data.get('description', '')
        )


class GrappleMapLoader(BaseModel):
    """Main loader for GrappleMap data with clean interface"""
    
    positions: Dict[str, GrappleMapPosition] = Field(default_factory=dict)
    transitions: Dict[str, Transition] = Field(default_factory=dict)
    adjacency: Dict[str, Dict] = Field(default_factory=dict)
    
    model_config = {"validate_assignment": True}
    
    @classmethod
    def from_file(cls, file_path: Union[str, Path]) -> 'GrappleMapLoader':
        """Load GrappleMap data from JSON file"""
        path = Path(file_path)
        if not path.exists():
            raise FileNotFoundError(f"GrappleMap file not found: {path}")
        
        with open(path, 'r') as f:
            raw_data = json.load(f)
        
        return cls.from_dict(raw_data)
    
    @classmethod 
    def from_dict(cls, data: Dict) -> 'GrappleMapLoader':
        """Load GrappleMap data from dictionary"""
        positions = {}
        transitions = {}
        adjacency = data.get('adjacency', {})
        
        # Parse positions
        for pos_id, pos_data in data.get('positions', {}).items():
            try:
                positions[pos_id] = GrappleMapPosition.from_raw_data(pos_id, pos_data)
            except Exception as e:
                print(f"Warning: Failed to parse position {pos_id}: {e}")
        
        # Parse transitions
        for trans_id, trans_data in data.get('transitions', {}).items():
            try:
                transitions[trans_id] = Transition.from_raw_data(trans_id, trans_data)
            except Exception as e:
                print(f"Warning: Failed to parse transition {trans_id}: {e}")
        
        return cls(
            positions=positions,
            transitions=transitions, 
            adjacency=adjacency
        )
    
    def get_position(self, position_id: str) -> Optional[GrappleMapPosition]:
        """Get a specific position by ID"""
        return self.positions.get(position_id)
    
    def get_transition(self, transition_id: str) -> Optional[Transition]:
        """Get a specific transition by ID"""
        return self.transitions.get(transition_id)
    
    def search_positions(self, query: str) -> List[GrappleMapPosition]:
        """Search positions by description or tags"""
        results = []
        query_lower = query.lower()
        
        for position in self.positions.values():
            if (position.contains_description(query) or 
                any(query_lower in tag.lower() for tag in position.tags)):
                results.append(position)
        
        return results
    
    def get_positions_by_tag(self, tag: str) -> List[GrappleMapPosition]:
        """Get all positions with a specific tag"""
        return [pos for pos in self.positions.values() if pos.contains_tag(tag)]
    
    def get_position_transitions(self, position_id: str) -> Tuple[List[Transition], List[Transition]]:
        """Get incoming and outgoing transitions for a position"""
        incoming = []
        outgoing = []
        
        for transition in self.transitions.values():
            if transition.to_position == position_id:
                incoming.append(transition)
            elif transition.from_position == position_id:
                outgoing.append(transition)
        
        return incoming, outgoing
    
    def __len__(self) -> int:
        """Number of positions"""
        return len(self.positions)
    
    def __contains__(self, position_id: str) -> bool:
        """Check if position exists"""
        return position_id in self.positions
    
    def __getitem__(self, position_id: str) -> GrappleMapPosition:
        """Get position by ID with dict-like access"""
        if position_id not in self.positions:
            raise KeyError(f"Position {position_id} not found")
        return self.positions[position_id]


# Convenience functions for common use cases
def load_grapplemap(file_path: Union[str, Path]) -> GrappleMapLoader:
    """Simple function to load GrappleMap data"""
    return GrappleMapLoader.from_file(file_path)


def get_position_joints(loader: GrappleMapLoader, position_id: str) -> Optional[np.ndarray]:
    """Get joint data for a position as numpy array"""
    position = loader.get_position(position_id)
    return position.get_combined_joints() if position else None


# Example usage and testing
if __name__ == "__main__":
    try:
        # Load the data
        loader = load_grapplemap("grapplemap_processed.json")
        print(f"Loaded {len(loader)} positions")
        
        print("\n" + "="*80)
        print("LIMB LENGTH VARIATION ANALYSIS")
        print("="*80)
        
        # Collect limb length data
        position_ids = list(loader.positions.keys())[:50]  # Analyze first 50 positions for speed
        
        # Storage for analysis
        limb_data = {limb: {'p1': [], 'p2': [], 'positions': []} for limb in NAMED_LIMBS.keys()}
        player_consistency = {'p1': {}, 'p2': {}}
        
        print(f"Analyzing limb lengths across {len(position_ids)} positions...")
        
        # Collect data from positions
        for pos_id in position_ids:
            try:
                position = loader[pos_id]
                p1_limbs = position.player1.get_all_limb_lengths()
                p2_limbs = position.player2.get_all_limb_lengths()
                
                for limb_name in NAMED_LIMBS.keys():
                    limb_data[limb_name]['p1'].append(p1_limbs[limb_name])
                    limb_data[limb_name]['p2'].append(p2_limbs[limb_name])
                    limb_data[limb_name]['positions'].append(pos_id)
                
            except Exception as e:
                print(f"Skipping position {pos_id}: {e}")
        
        # Statistical Analysis
        print(f"\n1. OVERALL LIMB LENGTH STATISTICS")
        print("-" * 50)
        print(f"{'Limb':<15} {'P1 Mean':<10} {'P1 Std':<10} {'P2 Mean':<10} {'P2 Std':<10} {'P1-P2 Diff':<12}")
        print("-" * 80)
        
        bilateral_pairs = [
            ('LeftThigh', 'RightThigh'),
            ('LeftShin', 'RightShin'), 
            ('LeftArm', 'RightArm'),
            ('LeftForearm', 'RightForearm'),
            ('LeftFoot', 'RightFoot')
        ]
        
        for limb_name in NAMED_LIMBS.keys():
            p1_data = np.array(limb_data[limb_name]['p1'])
            p2_data = np.array(limb_data[limb_name]['p2'])
            
            p1_mean, p1_std = np.mean(p1_data), np.std(p1_data)
            p2_mean, p2_std = np.mean(p2_data), np.std(p2_data)
            diff_mean = np.mean(np.abs(p1_data - p2_data))
            
            print(f"{limb_name:<15} {p1_mean:<10.4f} {p1_std:<10.4f} {p2_mean:<10.4f} {p2_std:<10.4f} {diff_mean:<12.4f}")
        
        # Between-player analysis (P1 vs P2 in same positions)
        print(f"\n2. PLAYER DIFFERENCES (P1 vs P2 in same positions)")
        print("-" * 60)
        
        max_diff_limb = ""
        max_diff_value = 0
        
        for limb_name in NAMED_LIMBS.keys():
            p1_data = np.array(limb_data[limb_name]['p1'])
            p2_data = np.array(limb_data[limb_name]['p2'])
            
            differences = np.abs(p1_data - p2_data)
            mean_diff = np.mean(differences)
            max_diff = np.max(differences)
            
            if mean_diff > max_diff_value:
                max_diff_value = mean_diff
                max_diff_limb = limb_name
            
            print(f"{limb_name:<15} Mean diff: {mean_diff:.4f}, Max diff: {max_diff:.4f}")
        
        print(f"\nLargest player differences in: {max_diff_limb} (avg {max_diff_value:.4f})")
        
        # Bilateral symmetry analysis
        print(f"\n3. BILATERAL SYMMETRY (Left vs Right limbs)")
        print("-" * 60)
        
        for left_limb, right_limb in bilateral_pairs:
            if left_limb in NAMED_LIMBS and right_limb in NAMED_LIMBS:
                # Player 1 symmetry
                p1_left = np.array(limb_data[left_limb]['p1'])
                p1_right = np.array(limb_data[right_limb]['p1'])
                p1_asymmetry = np.mean(np.abs(p1_left - p1_right))
                
                # Player 2 symmetry  
                p2_left = np.array(limb_data[left_limb]['p2'])
                p2_right = np.array(limb_data[right_limb]['p2'])
                p2_asymmetry = np.mean(np.abs(p2_left - p2_right))
                
                print(f"{left_limb:<12} vs {right_limb:<12}: P1 asymmetry: {p1_asymmetry:.4f}, P2 asymmetry: {p2_asymmetry:.4f}")
        
        # Positional variation analysis
        print(f"\n4. POSITIONAL VARIATION (same player across positions)")
        print("-" * 60)
        
        most_variable_limb = ""
        max_variation = 0
        
        for limb_name in NAMED_LIMBS.keys():
            p1_data = np.array(limb_data[limb_name]['p1'])
            p2_data = np.array(limb_data[limb_name]['p2'])
            
            p1_variation = np.std(p1_data) / np.mean(p1_data) * 100  # Coefficient of variation
            p2_variation = np.std(p2_data) / np.mean(p2_data) * 100
            avg_variation = (p1_variation + p2_variation) / 2
            
            if avg_variation > max_variation:
                max_variation = avg_variation
                most_variable_limb = limb_name
            
            print(f"{limb_name:<15} P1 CV: {p1_variation:.2f}%, P2 CV: {p2_variation:.2f}%")
        
        print(f"\nMost variable limb: {most_variable_limb} (CV: {max_variation:.2f}%)")
        
        # Outlier detection
        print(f"\n5. OUTLIER DETECTION (positions with unusual limb lengths)")
        print("-" * 70)
        
        outlier_threshold = 2.5  # Standard deviations
        outlier_positions = set()
        
        for limb_name in ['LeftThigh', 'RightThigh', 'LeftShin', 'RightShin']:  # Focus on major limbs
            if limb_name in NAMED_LIMBS:
                p1_data = np.array(limb_data[limb_name]['p1'])
                p2_data = np.array(limb_data[limb_name]['p2'])
                positions = limb_data[limb_name]['positions']
                
                # Find outliers in P1
                p1_mean, p1_std = np.mean(p1_data), np.std(p1_data)
                p1_z_scores = np.abs((p1_data - p1_mean) / p1_std)
                p1_outliers = np.where(p1_z_scores > outlier_threshold)[0]
                
                # Find outliers in P2
                p2_mean, p2_std = np.mean(p2_data), np.std(p2_data)
                p2_z_scores = np.abs((p2_data - p2_mean) / p2_std)
                p2_outliers = np.where(p2_z_scores > outlier_threshold)[0]
                
                for idx in p1_outliers:
                    outlier_positions.add(positions[idx])
                    print(f"{positions[idx]}: P1 {limb_name} = {p1_data[idx]:.4f} (z={p1_z_scores[idx]:.2f})")
                
                for idx in p2_outliers:
                    outlier_positions.add(positions[idx])
                    print(f"{positions[idx]}: P2 {limb_name} = {p2_data[idx]:.4f} (z={p2_z_scores[idx]:.2f})")
        
        print(f"\nTotal positions with outlier limb lengths: {len(outlier_positions)}")
        
        # Summary and conclusions
        print(f"\n6. SUMMARY & DATA QUALITY ASSESSMENT")
        print("-" * 60)
        
        # Calculate overall statistics
        all_variations = []
        all_asymmetries = []
        all_player_diffs = []
        
        for limb_name in NAMED_LIMBS.keys():
            p1_data = np.array(limb_data[limb_name]['p1'])
            p2_data = np.array(limb_data[limb_name]['p2'])
            
            # Variation within player across positions
            p1_cv = np.std(p1_data) / np.mean(p1_data) * 100
            p2_cv = np.std(p2_data) / np.mean(p2_data) * 100
            all_variations.extend([p1_cv, p2_cv])
            
            # Player differences
            player_diff = np.mean(np.abs(p1_data - p2_data))
            all_player_diffs.append(player_diff)
        
        # Bilateral asymmetry
        for left_limb, right_limb in bilateral_pairs:
            if left_limb in NAMED_LIMBS and right_limb in NAMED_LIMBS:
                p1_left = np.array(limb_data[left_limb]['p1'])
                p1_right = np.array(limb_data[right_limb]['p1'])
                p2_left = np.array(limb_data[left_limb]['p2'])
                p2_right = np.array(limb_data[right_limb]['p2'])
                
                p1_asym = np.mean(np.abs(p1_left - p1_right))
                p2_asym = np.mean(np.abs(p2_left - p2_right))
                all_asymmetries.extend([p1_asym, p2_asym])
        
        avg_variation = np.mean(all_variations)
        avg_player_diff = np.mean(all_player_diffs)
        avg_asymmetry = np.mean(all_asymmetries)
        
        print(f"Average coefficient of variation: {avg_variation:.2f}%")
        print(f"Average player difference: {avg_player_diff:.4f}")
        print(f"Average bilateral asymmetry: {avg_asymmetry:.4f}")
        print(f"Outlier positions: {len(outlier_positions)} out of {len(position_ids)} ({len(outlier_positions)/len(position_ids)*100:.1f}%)")
        
        print(f"\nCONCLUSIONS:")
        if avg_variation > 5:
            print("- HIGH variation in limb lengths across positions suggests significant data inconsistency")
        elif avg_variation > 2:
            print("- MODERATE variation in limb lengths - could be pose-dependent compression/stretching")
        else:
            print("- LOW variation in limb lengths - data appears consistent")
            
        if avg_asymmetry > 0.1:
            print("- HIGH bilateral asymmetry - possible annotation errors or intentional modeling")
        else:
            print("- LOW bilateral asymmetry - good left/right consistency")
            
        if len(outlier_positions) > len(position_ids) * 0.1:
            print("- MANY outlier positions detected - recommend data quality review")
        else:
            print("- FEW outliers detected - data quality appears reasonable")
            
    except FileNotFoundError:
        print("GrappleMap file not found - this is expected when running as example")
    except Exception as e:
        print(f"Error during analysis: {e}")
        import traceback
        traceback.print_exc()