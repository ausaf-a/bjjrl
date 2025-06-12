using System.Collections.Generic;
using UnityEngine;

public class YBotRetargeter : MonoBehaviour
{
    [Header("Configuration")]
    public int playerIndex = 0;
    
    [Header("Debug")]
    public bool showBoneGizmos = false;
    public bool logMappingErrors = true;
    
    private Animator animator;
    private Dictionary<HumanBodyBones, Transform> boneTransforms;
    private Vector3[] currentJointPositions = new Vector3[23];
    
    // GrappleMap joint indices
    private const int LEFT_TOE = 0, RIGHT_TOE = 1, LEFT_HEEL = 2, RIGHT_HEEL = 3;
    private const int LEFT_ANKLE = 4, RIGHT_ANKLE = 5, LEFT_KNEE = 6, RIGHT_KNEE = 7;
    private const int LEFT_HIP = 8, RIGHT_HIP = 9, LEFT_SHOULDER = 10, RIGHT_SHOULDER = 11;
    private const int LEFT_ELBOW = 12, RIGHT_ELBOW = 13, LEFT_WRIST = 14, RIGHT_WRIST = 15;
    private const int LEFT_HAND = 16, RIGHT_HAND = 17, LEFT_FINGERS = 18, RIGHT_FINGERS = 19;
    private const int CORE = 20, NECK = 21, HEAD = 22;
    
    public void Initialize(int player)
    {
        playerIndex = player;
        animator = GetComponent<Animator>();
        
        if (animator == null)
        {
            Debug.LogError($"No Animator found on Y Bot player {player}!");
            return;
        }
        
        if (animator.avatar == null || !animator.avatar.isHuman)
        {
            Debug.LogError($"Y Bot player {player} doesn't have a humanoid avatar!");
            return;
        }
        
        CacheBoneTransforms();
        Debug.Log($"Y Bot Retargeter initialized for player {player}");
    }
    
    void CacheBoneTransforms()
    {
        boneTransforms = new Dictionary<HumanBodyBones, Transform>();
        
        foreach (HumanBodyBones bone in System.Enum.GetValues(typeof(HumanBodyBones)))
        {
            if (bone != HumanBodyBones.LastBone)
            {
                Transform boneTransform = animator.GetBoneTransform(bone);
                if (boneTransform != null)
                {
                    boneTransforms[bone] = boneTransform;
                }
            }
        }
        
        Debug.Log($"Cached {boneTransforms.Count} bone transforms for Y Bot");
    }
    
    public void ApplyPose(Vector3[] jointPositions)
    {
        if (jointPositions.Length != 23)
        {
            Debug.LogError($"Expected 23 joint positions, got {jointPositions.Length}");
            return;
        }
        
        // Apply coordinate system fix to all joint positions
        currentJointPositions = new Vector3[23];
        for (int i = 0; i < 23; i++)
        {
            currentJointPositions[i] = jointPositions[i];
            currentJointPositions[i].x = -currentJointPositions[i].x; // Fix X-axis reflection
        }
        
        // Position the root transform first
        PositionRoot();
        
        // Apply rotations to bones based on joint positions
        ApplyBoneRotations();
    }
    
    void PositionRoot()
    {
        // Calculate hip position from GrappleMap data
        Vector3 leftHip = currentJointPositions[LEFT_HIP];
        Vector3 rightHip = currentJointPositions[RIGHT_HIP];
        Vector3 core = currentJointPositions[CORE];
        
        // Position root at hip center
        Vector3 hipCenter = (leftHip + rightHip) / 2f;
        
        // Fix X-axis reflection
        hipCenter.x = -hipCenter.x;
        
        transform.position = hipCenter;
        
        // Calculate body orientation from hip and shoulder data
        Vector3 leftShoulder = currentJointPositions[LEFT_SHOULDER];
        Vector3 rightShoulder = currentJointPositions[RIGHT_SHOULDER];
        Vector3 shoulderCenter = (leftShoulder + rightShoulder) / 2f;
        
        // Apply X-axis fix to shoulders too
        leftShoulder.x = -leftShoulder.x;
        rightShoulder.x = -rightShoulder.x;
        shoulderCenter.x = -shoulderCenter.x;
        
        // Body forward direction (corrected)
        Vector3 hipLine = (rightShoulder - leftShoulder).normalized; // Note: using shoulders not hips
        Vector3 spineDirection = (shoulderCenter - hipCenter).normalized;
        
        // Ensure we have valid directions
        if (hipLine == Vector3.zero || spineDirection == Vector3.zero)
        {
            transform.rotation = Quaternion.identity;
            return;
        }
        
        // Calculate forward direction (reversed to fix backwards rotation)
        Vector3 forward = -Vector3.Cross(hipLine, spineDirection).normalized;
        
        if (forward != Vector3.zero)
        {
            transform.rotation = Quaternion.LookRotation(forward, spineDirection);
        }
        else
        {
            // Fallback: just use spine direction as up
            transform.rotation = Quaternion.LookRotation(-Vector3.forward, spineDirection);
        }
    }
    
    void ApplyBoneRotations()
    {
        // Apply rotations to key bones
        ApplySpineRotation();
        ApplyArmRotations();
        ApplyLegRotations();
        ApplyHandRotations();
    }
    
    void ApplySpineRotation()
    {
        // Spine rotation based on core to neck direction
        if (boneTransforms.ContainsKey(HumanBodyBones.Spine))
        {
            Vector3 spineDirection = currentJointPositions[NECK] - currentJointPositions[CORE];
            SetBoneRotation(HumanBodyBones.Spine, spineDirection, Vector3.up);
        }
        
        // Neck rotation based on neck to head direction
        if (boneTransforms.ContainsKey(HumanBodyBones.Neck))
        {
            Vector3 neckDirection = currentJointPositions[HEAD] - currentJointPositions[NECK];
            SetBoneRotation(HumanBodyBones.Neck, neckDirection, Vector3.up);
        }
    }
    
    void ApplyArmRotations()
    {
        // Left arm
        SetLimbRotation(HumanBodyBones.LeftUpperArm, LEFT_SHOULDER, LEFT_ELBOW);
        SetLimbRotation(HumanBodyBones.LeftLowerArm, LEFT_ELBOW, LEFT_WRIST);
        
        // Right arm  
        SetLimbRotation(HumanBodyBones.RightUpperArm, RIGHT_SHOULDER, RIGHT_ELBOW);
        SetLimbRotation(HumanBodyBones.RightLowerArm, RIGHT_ELBOW, RIGHT_WRIST);
    }
    
    void ApplyLegRotations()
    {
        // Left leg
        SetLimbRotation(HumanBodyBones.LeftUpperLeg, LEFT_HIP, LEFT_KNEE);
        SetLimbRotation(HumanBodyBones.LeftLowerLeg, LEFT_KNEE, LEFT_ANKLE);
        SetFootRotation(HumanBodyBones.LeftFoot, LEFT_ANKLE, LEFT_TOE, LEFT_HEEL);
        
        // Right leg
        SetLimbRotation(HumanBodyBones.RightUpperLeg, RIGHT_HIP, RIGHT_KNEE);
        SetLimbRotation(HumanBodyBones.RightLowerLeg, RIGHT_KNEE, RIGHT_ANKLE);
        SetFootRotation(HumanBodyBones.RightFoot, RIGHT_ANKLE, RIGHT_TOE, RIGHT_HEEL);
    }
    
    void ApplyHandRotations()
    {
        // Calculate hand orientations from wrist to fingers
        if (boneTransforms.ContainsKey(HumanBodyBones.LeftHand))
        {
            Vector3 handDirection = currentJointPositions[LEFT_FINGERS] - currentJointPositions[LEFT_WRIST];
            SetBoneRotation(HumanBodyBones.LeftHand, handDirection, Vector3.up);
        }
        
        if (boneTransforms.ContainsKey(HumanBodyBones.RightHand))
        {
            Vector3 handDirection = currentJointPositions[RIGHT_FINGERS] - currentJointPositions[RIGHT_WRIST];
            SetBoneRotation(HumanBodyBones.RightHand, handDirection, Vector3.up);
        }
    }
    
    void SetLimbRotation(HumanBodyBones bone, int fromJoint, int toJoint)
    {
        if (!boneTransforms.ContainsKey(bone)) return;
        
        Vector3 direction = currentJointPositions[toJoint] - currentJointPositions[fromJoint];
        SetBoneRotation(bone, direction, Vector3.up);
    }
    
    void SetFootRotation(HumanBodyBones bone, int ankle, int toe, int heel)
    {
        if (!boneTransforms.ContainsKey(bone)) return;
        
        Vector3 forward = currentJointPositions[toe] - currentJointPositions[heel];
        Vector3 up = currentJointPositions[ankle] - currentJointPositions[heel];
        
        SetBoneRotation(bone, forward, up);
    }
    
    void SetBoneRotation(HumanBodyBones bone, Vector3 direction, Vector3 upDirection)
    {
        if (!boneTransforms.ContainsKey(bone) || direction == Vector3.zero) return;
        
        Transform boneTransform = boneTransforms[bone];
        
        // Convert world direction to local space relative to the character root
        Vector3 localDirection = transform.InverseTransformDirection(direction.normalized);
        Vector3 localUp = transform.InverseTransformDirection(upDirection.normalized);
        
        // Fix backwards rotation for shoulder and hip joints
        if (bone == HumanBodyBones.LeftUpperArm || bone == HumanBodyBones.RightUpperArm ||
            bone == HumanBodyBones.LeftUpperLeg || bone == HumanBodyBones.RightUpperLeg)
        {
            localDirection = -localDirection; // Reverse direction for these joints
        }
        
        // Calculate target rotation
        Quaternion targetRotation = Quaternion.LookRotation(localDirection, localUp);
        
        // Apply relative to the character's root rotation
        boneTransform.rotation = transform.rotation * targetRotation;
    }
    
    void OnDrawGizmos()
    {
        if (!showBoneGizmos || currentJointPositions == null) return;
        
        // Draw joint positions
        Gizmos.color = playerIndex == 0 ? Color.red : Color.blue;
        for (int i = 0; i < currentJointPositions.Length; i++)
        {
            Gizmos.DrawWireSphere(currentJointPositions[i], 0.02f);
        }
        
        // Draw major bone connections
        Gizmos.color = Color.yellow;
        DrawGizmoBone(LEFT_SHOULDER, LEFT_ELBOW);
        DrawGizmoBone(LEFT_ELBOW, LEFT_WRIST);
        DrawGizmoBone(RIGHT_SHOULDER, RIGHT_ELBOW);
        DrawGizmoBone(RIGHT_ELBOW, RIGHT_WRIST);
        DrawGizmoBone(LEFT_HIP, LEFT_KNEE);
        DrawGizmoBone(LEFT_KNEE, LEFT_ANKLE);
        DrawGizmoBone(RIGHT_HIP, RIGHT_KNEE);
        DrawGizmoBone(RIGHT_KNEE, RIGHT_ANKLE);
        DrawGizmoBone(CORE, NECK);
        DrawGizmoBone(NECK, HEAD);
    }
    
    void DrawGizmoBone(int joint1, int joint2)
    {
        if (joint1 < currentJointPositions.Length && joint2 < currentJointPositions.Length)
        {
            Gizmos.DrawLine(currentJointPositions[joint1], currentJointPositions[joint2]);
        }
    }
}