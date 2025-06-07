# Worklog 

## 6-5-25
Moved from unity to MuJoCo. Created scripts to load the grapplemap JSON and construct player bodies in mujoco using the grapplemap joints 

## 6-6-25


As part of backlog #1, created an improved parser for the GrappleMap. Also assessed the variation in bone lengths across the dataset. Seems to be within a very tight margin of error, so we can build a T-pose using the average bone length for each bone. Still some concerns. 

- neck joint movement seems unnatural. Not sure if the neck joint itself should be movable. Head joint movement also just rotates the head. 

Still need to figure out how to correctly initialize players as T pose, we need to figure out where to create the T posed players first, and refine how to apply IK in order to configure them in the correct grappling poses. Maybe shift the players based on the target core positions, and then set IK targets based on the joints. THen use something like ikpy to get the joints to move correctly, and start physics sim after the poses are actually aligned. 

## 6-7-25

Still wokring on #1, figuring out the details noted in the last part. Should do some research on similar projects to clarify the project roadmap and find inspiration for RL techniques we should try. 

# Backlog

1. joint limits need to be set accurately. currently, there is no "T-pose" in the grapplemap dataset. When creating joints in mujoco, the joint limits are set based off of the inital angle. So if we load a position where joints are already bent, then the proper range of motion is not set. 

To solve this issue, we need to
- compute the limb lengths by taking the distance between connected joints in the grapplemap 
- create a T pose reference pose. 
- when loading, initialize the player in the T pose first, and then apply the selected position by applying IK.    
