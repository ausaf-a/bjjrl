import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer
import json
import time
from enum import Enum
from typing import Dict, List, Tuple, Optional

class Joint(Enum):
    """Joint indices matching the grapplemap data structure"""
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

class GrappleMapBJJEnv(gym.Env):
    """
    BJJ Environment that loads positions from GrappleMap data.
    Uses simplified stick-figure representation for easier visualization and debugging.
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(self, 
                 grapplemap_data: Dict = None,
                 initial_position: str = "side_control",
                 render_mode: Optional[str] = None):
        super().__init__()
        
        self.render_mode = render_mode
        self.grapplemap_data = grapplemap_data
        self.initial_position = initial_position
        
        # Create the MuJoCo model
        self.model_xml = self._create_stick_figure_model()
        self.model = mujoco.MjModel.from_xml_string(self.model_xml)
        self.data = mujoco.MjData(self.model)
        
        # Define action and observation spaces
        # Actions: target positions for key joints (simplified control)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0,
            shape=(12,),  # 6 DOF for each player's movement
            dtype=np.float32
        )
        
        # Observations: joint positions and velocities
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(self.model.nq + self.model.nv,),
            dtype=np.float32
        )
        
        self.viewer = None
        self.joint_positions = None
    
    def _create_stick_figure_model(self) -> str:
        """Create a simplified stick-figure model for visualization"""
        return """
        <mujoco model="bjj_stick_figures">
            <option gravity="0 0 -9.81" timestep="0.002" iterations="50" solver="Newton"/>
            
            <visual>
                <global offwidth="1920" offheight="1080"/>
                <quality shadowsize="2048"/>
            </visual>
            
            <asset>
                <material name="red" rgba="0.8 0.1 0.1 1"/>
                <material name="blue" rgba="0.1 0.1 0.8 1"/>
                <material name="mat" rgba="0.9 0.9 0.9 1" reflectance="0.1"/>
            </asset>
            
            <worldbody>
                <light directional="true" diffuse="0.6 0.6 0.6" pos="0 0 4" dir="0 0 -1"/>
                <light directional="true" diffuse="0.4 0.4 0.4" pos="2 2 4" dir="-1 -1 -1"/>
                
                <!-- Ground/Mat -->
                <geom name="floor" type="plane" size="3 3 0.1" material="mat"/>
                
                <!-- Player 0 (Red) Stick Figure -->
                <body name="p0_root" pos="0 0 1">
                    <freejoint name="p0_root"/>
                    <geom name="p0_sphere" type="sphere" size="0.02" material="red"/>
                    
                    <!-- Core/Torso -->
                    <geom name="p0_spine" type="capsule" fromto="0 0 0 0 0 0.3" size="0.01" material="red"/>
                    
                    <!-- Head -->
                    <geom name="p0_neck" type="capsule" fromto="0 0 0.3 0 0 0.4" size="0.008" material="red"/>
                    <geom name="p0_head" type="sphere" pos="0 0 0.45" size="0.05" material="red"/>
                    
                    <!-- Left Arm -->
                    <geom name="p0_l_shoulder" type="capsule" fromto="0 0 0.25 0.1 0 0.25" size="0.008" material="red"/>
                    <geom name="p0_l_upper_arm" type="capsule" fromto="0.1 0 0.25 0.2 0 0.15" size="0.008" material="red"/>
                    <geom name="p0_l_forearm" type="capsule" fromto="0.2 0 0.15 0.3 0 0.05" size="0.008" material="red"/>
                    
                    <!-- Right Arm -->
                    <geom name="p0_r_shoulder" type="capsule" fromto="0 0 0.25 -0.1 0 0.25" size="0.008" material="red"/>
                    <geom name="p0_r_upper_arm" type="capsule" fromto="-0.1 0 0.25 -0.2 0 0.15" size="0.008" material="red"/>
                    <geom name="p0_r_forearm" type="capsule" fromto="-0.2 0 0.15 -0.3 0 0.05" size="0.008" material="red"/>
                    
                    <!-- Left Leg -->
                    <geom name="p0_l_hip" type="capsule" fromto="0 0 0 0.05 0 0" size="0.008" material="red"/>
                    <geom name="p0_l_thigh" type="capsule" fromto="0.05 0 0 0.1 0 -0.2" size="0.01" material="red"/>
                    <geom name="p0_l_shin" type="capsule" fromto="0.1 0 -0.2 0.1 0 -0.4" size="0.008" material="red"/>
                    
                    <!-- Right Leg -->
                    <geom name="p0_r_hip" type="capsule" fromto="0 0 0 -0.05 0 0" size="0.008" material="red"/>
                    <geom name="p0_r_thigh" type="capsule" fromto="-0.05 0 0 -0.1 0 -0.2" size="0.01" material="red"/>
                    <geom name="p0_r_shin" type="capsule" fromto="-0.1 0 -0.2 -0.1 0 -0.4" size="0.008" material="red"/>
                </body>
                
                <!-- Player 1 (Blue) Stick Figure -->
                <body name="p1_root" pos="0.5 0 1">
                    <freejoint name="p1_root"/>
                    <geom name="p1_sphere" type="sphere" size="0.02" material="blue"/>
                    
                    <!-- Same structure as player 0 but with blue material -->
                    <geom name="p1_spine" type="capsule" fromto="0 0 0 0 0 0.3" size="0.01" material="blue"/>
                    <geom name="p1_neck" type="capsule" fromto="0 0 0.3 0 0 0.4" size="0.008" material="blue"/>
                    <geom name="p1_head" type="sphere" pos="0 0 0.45" size="0.05" material="blue"/>
                    
                    <!-- Arms -->
                    <geom name="p1_l_shoulder" type="capsule" fromto="0 0 0.25 0.1 0 0.25" size="0.008" material="blue"/>
                    <geom name="p1_l_upper_arm" type="capsule" fromto="0.1 0 0.25 0.2 0 0.15" size="0.008" material="blue"/>
                    <geom name="p1_l_forearm" type="capsule" fromto="0.2 0 0.15 0.3 0 0.05" size="0.008" material="blue"/>
                    
                    <geom name="p1_r_shoulder" type="capsule" fromto="0 0 0.25 -0.1 0 0.25" size="0.008" material="blue"/>
                    <geom name="p1_r_upper_arm" type="capsule" fromto="-0.1 0 0.25 -0.2 0 0.15" size="0.008" material="blue"/>
                    <geom name="p1_r_forearm" type="capsule" fromto="-0.2 0 0.15 -0.3 0 0.05" size="0.008" material="blue"/>
                    
                    <!-- Legs -->
                    <geom name="p1_l_hip" type="capsule" fromto="0 0 0 0.05 0 0" size="0.008" material="blue"/>
                    <geom name="p1_l_thigh" type="capsule" fromto="0.05 0 0 0.1 0 -0.2" size="0.01" material="blue"/>
                    <geom name="p1_l_shin" type="capsule" fromto="0.1 0 -0.2 0.1 0 -0.4" size="0.008" material="blue"/>
                    
                    <geom name="p1_r_hip" type="capsule" fromto="0 0 0 -0.05 0 0" size="0.008" material="blue"/>
                    <geom name="p1_r_thigh" type="capsule" fromto="-0.05 0 0 -0.1 0 -0.2" size="0.01" material="blue"/>
                    <geom name="p1_r_shin" type="capsule" fromto="-0.1 0 -0.2 -0.1 0 -0.4" size="0.008" material="blue"/>
                </body>
            </worldbody>
            
            <actuator>
                <!-- Simple position control for each player's root body -->
                <position name="p0_x" joint="p0_root" kp="500" ctrlrange="-2 2" ctrllimited="true" gear="1 0 0 0 0 0"/>
                <position name="p0_y" joint="p0_root" kp="500" ctrlrange="-2 2" ctrllimited="true" gear="0 1 0 0 0 0"/>
                <position name="p0_z" joint="p0_root" kp="500" ctrlrange="0 2" ctrllimited="true" gear="0 0 1 0 0 0"/>
                <position name="p0_rx" joint="p0_root" kp="100" ctrlrange="-3.14 3.14" ctrllimited="true" gear="0 0 0 1 0 0"/>
                <position name="p0_ry" joint="p0_root" kp="100" ctrlrange="-3.14 3.14" ctrllimited="true" gear="0 0 0 0 1 0"/>
                <position name="p0_rz" joint="p0_root" kp="100" ctrlrange="-3.14 3.14" ctrllimited="true" gear="0 0 0 0 0 1"/>
                
                <position name="p1_x" joint="p1_root" kp="500" ctrlrange="-2 2" ctrllimited="true" gear="1 0 0 0 0 0"/>
                <position name="p1_y" joint="p1_root" kp="500" ctrlrange="-2 2" ctrllimited="true" gear="0 1 0 0 0 0"/>
                <position name="p1_z" joint="p1_root" kp="500" ctrlrange="0 2" ctrllimited="true" gear="0 0 1 0 0 0"/>
                <position name="p1_rx" joint="p1_root" kp="100" ctrlrange="-3.14 3.14" ctrllimited="true" gear="0 0 0 1 0 0"/>
                <position name="p1_ry" joint="p1_root" kp="100" ctrlrange="-3.14 3.14" ctrllimited="true" gear="0 0 0 0 1 0"/>
                <position name="p1_rz" joint="p1_root" kp="100" ctrlrange="-3.14 3.14" ctrllimited="true" gear="0 0 0 0 0 1"/>
            </actuator>
        </mujoco>
        """
    
    def _load_position_from_grapplemap(self, position_name: str):
        """Load a position from grapplemap data"""
        if self.grapplemap_data is None:
            # Use default positions if no data provided
            return self._get_default_position(position_name)
        
        # Find position by name/tag
        for pos_id, pos_data in self.grapplemap_data["positions"].items():
            if position_name in pos_data.get("tags", []) or position_name in pos_data.get("description", ""):
                return np.array(pos_data["joints"])
        
        return self._get_default_position(position_name)
    
    def _get_default_position(self, position_name: str):
        """Get default positions for common BJJ positions"""
        if position_name == "side_control":
            # Simplified side control position
            joints = np.zeros((46, 3))
            # Player 0 (top) - roughly on top
            joints[Joint.Core.value] = [0.0, 0.0, 0.3]
            joints[Joint.Head.value] = [0.0, 0.2, 0.4]
            # Player 1 (bottom) - on back
            joints[23 + Joint.Core.value] = [0.0, 0.0, 0.1]
            joints[23 + Joint.Head.value] = [0.0, 0.2, 0.1]
            return joints
        else:
            # Default standing position
            joints = np.zeros((46, 3))
            joints[Joint.Core.value] = [-0.3, 0.0, 0.5]
            joints[23 + Joint.Core.value] = [0.3, 0.0, 0.5]
            return joints
    
    def reset(self, seed=None, options=None):
        """Reset the environment"""
        super().reset(seed=seed)
        
        # Reset MuJoCo state
        mujoco.mj_resetData(self.model, self.data)
        
        # Load position from grapplemap
        self.joint_positions = self._load_position_from_grapplemap(self.initial_position)
        
        # Set player positions based on core joint positions
        scale = 0.1  # Scale down from grapplemap units
        
        # Player 0
        p0_core = self.joint_positions[Joint.Core.value] * scale
        self.data.qpos[0:3] = p0_core  # x, y, z position
        
        # Player 1
        p1_core = self.joint_positions[23 + Joint.Core.value] * scale
        self.data.qpos[7:10] = p1_core  # x, y, z position
        
        # Set initial orientations (quaternions) - identity for now
        self.data.qpos[3:7] = [1, 0, 0, 0]   # Player 0 quaternion
        self.data.qpos[10:14] = [1, 0, 0, 0] # Player 1 quaternion
        
        # Forward dynamics
        mujoco.mj_forward(self.model, self.data)
        
        return self._get_obs(), {}
    
    def step(self, action):
        """Execute one time step"""
        # Apply actions as target positions
        self.data.ctrl[:] = action
        
        # Step simulation multiple times for stability
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)
        
        # Get observation
        observation = self._get_obs()
        
        # Calculate reward based on maintaining contact and position
        reward = self._calculate_reward()
        
        # Check termination
        terminated = self._is_terminated()
        
        return observation, reward, terminated, False, {}
    
    def _get_obs(self):
        """Get current observation"""
        return np.concatenate([self.data.qpos, self.data.qvel])
    
    def _calculate_reward(self):
        """Calculate reward - encourage maintaining grappling positions"""
        # Distance between players (should be close for grappling)
        p0_pos = self.data.qpos[0:3]
        p1_pos = self.data.qpos[7:10]
        distance = np.linalg.norm(p0_pos - p1_pos)
        
        # Penalize large distances
        distance_reward = -max(0, distance - 0.5)
        
        # Penalize excessive movement
        velocity_penalty = -0.01 * np.sum(np.square(self.data.qvel))
        
        return distance_reward + velocity_penalty
    
    def _is_terminated(self):
        """Check if episode should terminate"""
        # Terminate if players are too far apart
        p0_pos = self.data.qpos[0:3]
        p1_pos = self.data.qpos[7:10]
        distance = np.linalg.norm(p0_pos - p1_pos)
        
        # Or if someone falls too low
        min_z = min(p0_pos[2], p1_pos[2])
        
        return distance > 2.0 or min_z < -0.5
    
    def render(self):
        """Render the environment"""
        if self.render_mode == "human":
            if self.viewer is None:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                # Set camera position for better view
                self.viewer.cam.azimuth = 45
                self.viewer.cam.elevation = -20
                self.viewer.cam.distance = 3.0
                self.viewer.cam.lookat[:] = [0, 0, 0.5]
            else:
                self.viewer.sync()
    
    def close(self):
        """Clean up"""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


def visualize_grapplemap_position(position_data: Dict):
    """Visualize a specific position from grapplemap data"""
    print(f"\nVisualizing position: {position_data.get('description', 'Unknown')}")
    print(f"Tags: {', '.join(position_data.get('tags', []))}")
    
    # Create environment with this position
    env = GrappleMapBJJEnv(
        grapplemap_data={"positions": {"current": position_data}},
        initial_position="current",
        render_mode="human"
    )
    
    obs, _ = env.reset()
    
    print("\nControls:")
    print("- The view should update automatically")
    print("- Press Ctrl+C to stop")
    print("- Players are shown as stick figures (red vs blue)")
    
    try:
        # Just hold the position with small movements
        for i in range(1000):
            # Keep current positions with small variations
            current_pos = env.data.qpos[[0,1,2,7,8,9]]  # x,y,z for both players
            action = np.concatenate([
                current_pos[:3] + np.random.normal(0, 0.01, 3),  # Player 0 position
                [0, 0, 0],  # Player 0 rotation (unchanged)
                current_pos[3:] + np.random.normal(0, 0.01, 3),  # Player 1 position
                [0, 0, 0],  # Player 1 rotation (unchanged)
            ])
            
            obs, reward, terminated, truncated, _ = env.step(action)
            env.render()
            
            if i % 100 == 0:
                print(f"Step {i}, Reward: {reward:.3f}")
            
            if terminated:
                print("Position lost - resetting")
                obs, _ = env.reset()
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nStopping visualization...")
    finally:
        env.close()


# Example usage
if __name__ == "__main__":
    # Test with sample grapplemap data
    sample_position = {
        "id": "pos_0",
        "description": "side control with crossface",
        "tags": ["side_control", "top_pressure"],
        "joints": [
            # Sample joint positions (simplified)
            [0.5, 0.2, 0.3],   # LeftToe
            [0.5, -0.2, 0.3],  # RightToe
            [0.4, 0.2, 0.3],   # LeftHeel
            [0.4, -0.2, 0.3],  # RightHeel
            [0.3, 0.2, 0.4],   # LeftAnkle
            [0.3, -0.2, 0.4],  # RightAnkle
            [0.2, 0.15, 0.6],  # LeftKnee
            [0.2, -0.15, 0.6], # RightKnee
            [0.1, 0.1, 0.8],   # LeftHip
            [0.1, -0.1, 0.8],  # RightHip
            [0.0, 0.2, 1.2],   # LeftShoulder
            [0.0, -0.2, 1.2],  # RightShoulder
            [0.2, 0.3, 1.0],   # LeftElbow
            [0.2, -0.3, 1.0],  # RightElbow
            [0.4, 0.3, 0.8],   # LeftWrist
            [0.4, -0.3, 0.8],  # RightWrist
            [0.5, 0.3, 0.7],   # LeftHand
            [0.5, -0.3, 0.7],  # RightHand
            [0.6, 0.3, 0.7],   # LeftFingers
            [0.6, -0.3, 0.7],  # RightFingers
            [0.0, 0.0, 1.0],   # Core
            [0.0, 0.0, 1.3],   # Neck
            [0.0, 0.0, 1.5],   # Head
            # Player 2 (on bottom)
            [0.2, 0.2, 0.1],   # LeftToe
            [0.2, -0.2, 0.1],  # RightToe
            [0.1, 0.2, 0.1],   # LeftHeel
            [0.1, -0.2, 0.1],  # RightHeel
            [0.0, 0.2, 0.1],   # LeftAnkle
            [0.0, -0.2, 0.1],  # RightAnkle
            [-0.1, 0.15, 0.1], # LeftKnee
            [-0.1, -0.15, 0.1],# RightKnee
            [-0.2, 0.1, 0.1],  # LeftHip
            [-0.2, -0.1, 0.1], # RightHip
            [-0.1, 0.2, 0.1],  # LeftShoulder
            [-0.1, -0.2, 0.1], # RightShoulder
            [0.1, 0.3, 0.1],   # LeftElbow
            [0.1, -0.3, 0.1],  # RightElbow
            [0.3, 0.3, 0.1],   # LeftWrist
            [0.3, -0.3, 0.1],  # RightWrist
            [0.4, 0.3, 0.1],   # LeftHand
            [0.4, -0.3, 0.1],  # RightHand
            [0.5, 0.3, 0.1],   # LeftFingers
            [0.5, -0.3, 0.1],  # RightFingers
            [0.0, 0.0, 0.1],   # Core
            [0.0, 0.1, 0.1],   # Neck
            [0.0, 0.2, 0.1],   # Head
        ]
    }
    
    # If you have the actual grapplemap_processed.json file, load it:
    try:
        with open("grapplemap_processed.json", "r") as f:
            grapplemap_data = json.load(f)
            # Use the first position
            first_pos = list(grapplemap_data["positions"].values())[0]
            visualize_grapplemap_position(first_pos)
    except FileNotFoundError:
        print("grapplemap_processed.json not found, using sample data")
        visualize_grapplemap_position(sample_position)