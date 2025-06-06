
---

### `PRD.md` (Product Requirements Document)

# Product Requirements Document: BJJ Physics Humanoid

## Overview

**Goal:**  
Enable interactive and physically accurate visualization of Brazilian Jiu Jitsu positions and transitions, using real data from GrappleMap, by building articulated ragdoll models in MuJoCo.

## Functional Requirements

1. **GrappleMap Integration:**
   - Parse joint positions from the processed GrappleMap JSON.
   - Support both single-player and dual-player visualization.

2. **Humanoid Construction:**
   - Automatically build a humanoid with correct joint hierarchy (parent/child).
   - Use realistic mass, radii, and joint types (ball, hinge) for each limb.
   - Assign appropriate joint limits (e.g., knees only bend backward).
   - Create actuators for key joints for future RL or control tasks.

3. **Physics & Visualization:**
   - Place both players in the loaded position, correctly scaled and oriented.
   - Add floor, lighting, and materials for clear visualization.
   - Adjust the ground plane to always be just below the lowest joint.
   - Expose CLI arguments for loading any position and scaling.

4. **Simulation:**
   - Support launching in MuJoCo's viewer for inspection.
   - Optionally allow running the simulation, observing realistic collapse.

5. **Extensibility:**
   - The code should be modular, with functions for MJCF generation, data loading, and joint hierarchy definition.
   - Designed for easy integration with reinforcement learning, e.g., for pose imitation.

## Non-Functional Requirements

- **Performance:** Scene generation and load time should be <2 seconds.
- **Code Quality:** Use clear structure and comments. Prefer standard Python libraries.
- **Data Integrity:** Never overwrite GrappleMap data. Generated MJCF files should be clearly named by position.
- **Documentation:** README and CLI help must explain usage, credits, and expected outputs.

## Dependencies

- Python 3.8+
- [MuJoCo](https://mujoco.readthedocs.io/) (>=2.3)
- NumPy

## Credits

- [GrappleMap](https://github.com/Eelis/GrappleMap): Original BJJ position data.
- [JuiJitsu-RL](https://github.com/amorsi1/JuiJitsu-RL): Parser and example code inspiration.
