# Worklog 

## 6-5-25
Moved from unity to MuJoCo. Created scripts to load the grapplemap JSON and construct player bodies in mujoco using the grapplemap joints 


# Backlog

1. joint limits need to be set accurately. currently, there is no "T-pose" in the grapplemap dataset. When creating joints in mujoco, the joint limits are set based off of the inital angle. So if we load a position where joints are already bent, then the proper range of motion is not set. 

To solve this issue, we need to
- compute the limb lengths by taking the distance between connected joints in the grapplemap 
- create a T pose reference pose. 
- when loading, initialize the player in the T pose first, and then apply the selected position by manually setting the joint positions   