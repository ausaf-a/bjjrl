using System.Collections.Generic;
using UnityEngine;

public class ControlRigRetargeter : MonoBehaviour
{
    [Header("Configuration")]
    public int playerIndex = 0;
    
    [Header("Control Targets")]
    [Tooltip("Assign these manually in the editor - create empty GameObjects as children")]
    public Transform leftHandTarget;
    public Transform rightHandTarget;
    public Transform leftFootTarget;
    public Transform rightFootTarget;
    public Transform headTarget;
    public Transform rootTarget; // For overall positioning
    
    [Header("Manual Setup")]
    public bool createTargetsAutomatically = false;
    [Tooltip("Click this button in the editor to create target GameObjects")]
    public bool createTargetsButton = false;
    
    [Header("IK Weights")]
    [Range(0f, 1f)] public float leftHandIKWeight = 1f;
    [Range(0f, 1f)] public float rightHandIKWeight = 1f;
    [Range(0f, 1f)] public float leftFootIKWeight = 1f;
    [Range(0f, 1f)] public float rightFootIKWeight = 1f;
    [Range(0f, 1f)] public float headIKWeight = 0.8f;
    
    [Header("Debug")]
    public bool showTargets = true;
    public bool logMappingErrors = true;
    
    private Animator animator;
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
        
        // Only create targets if they don't exist and auto-creation is enabled
        if (createTargetsAutomatically && AreTargetsMissing())
        {
            CreateControlTargets();
        }
        
        Debug.Log($"Control Rig Retargeter initialized for player {player}");
    }
    
    bool AreTargetsMissing()
    {
        return leftHandTarget == null || rightHandTarget == null || 
               leftFootTarget == null || rightFootTarget == null || 
               headTarget == null || rootTarget == null;
    }
    
    void OnValidate()
    {
        // Editor button functionality
        if (createTargetsButton)
        {
            createTargetsButton = false;
            CreateControlTargets();
        }
    }
    
    void CreateControlTargets()
    {
        // Create target objects as children of this transform
        leftHandTarget = CreateTarget("LeftHandTarget", Color.red);
        rightHandTarget = CreateTarget("RightHandTarget", Color.red);
        leftFootTarget = CreateTarget("LeftFootTarget", Color.blue);
        rightFootTarget = CreateTarget("RightFootTarget", Color.blue);
        headTarget = CreateTarget("HeadTarget", Color.yellow);
        rootTarget = CreateTarget("RootTarget", Color.green);
    }
    
    Transform CreateTarget(string name, Color color)
    {
        GameObject targetGO = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        targetGO.name = name;
        targetGO.transform.SetParent(transform);
        targetGO.transform.localScale = Vector3.one * 0.05f; // Small spheres
        targetGO.transform.localPosition = Vector3.zero; // Start at character center
        
        // Remove collider to avoid physics interference
        Collider collider = targetGO.GetComponent<Collider>();
        if (collider != null) DestroyImmediate(collider);
        
        // Use a simple material approach that works better
        Renderer renderer = targetGO.GetComponent<Renderer>();
        if (renderer != null)
        {
            // Try to use a built-in material first
            Material mat = renderer.material; // Get default material
            mat.color = color;
            renderer.material = mat;
        }
        
        // Make them visible/invisible based on debug setting
        targetGO.SetActive(showTargets);
        
        Debug.Log($"Created target: {name} at {targetGO.transform.position}");
        
        return targetGO.transform;
    }
    
    public void ApplyPose(Vector3[] jointPositions)
    {
        if (jointPositions.Length != 23)
        {
            Debug.LogError($"Expected 23 joint positions, got {jointPositions.Length}");
            return;
        }
        
        // Apply coordinate system fix to all joint positions (from previous work)
        currentJointPositions = new Vector3[23];
        for (int i = 0; i < 23; i++)
        {
            currentJointPositions[i] = jointPositions[i];
            currentJointPositions[i].x = -currentJointPositions[i].x; // Fix X-axis reflection
        }
        
        // Position the character root
        PositionRoot();
        
        // Update control targets
        UpdateControlTargets();
    }
    
    void PositionRoot()
    {
        // Calculate hip center for root positioning
        Vector3 leftHip = currentJointPositions[LEFT_HIP];
        Vector3 rightHip = currentJointPositions[RIGHT_HIP];
        Vector3 hipCenter = (leftHip + rightHip) / 2f;
        
        // Position the character at hip level
        transform.position = hipCenter;
        
        // Set root target for reference
        if (rootTarget != null)
        {
            rootTarget.position = hipCenter;
        }
        
        // Calculate basic orientation (simplified for now)
        Vector3 leftShoulder = currentJointPositions[LEFT_SHOULDER];
        Vector3 rightShoulder = currentJointPositions[RIGHT_SHOULDER];
        Vector3 shoulderLine = (rightShoulder - leftShoulder).normalized;
        Vector3 spineDirection = ((leftShoulder + rightShoulder) / 2f - hipCenter).normalized;
        
        if (shoulderLine != Vector3.zero && spineDirection != Vector3.zero)
        {
            Vector3 forward = Vector3.Cross(shoulderLine, spineDirection).normalized;
            if (forward != Vector3.zero)
            {
                transform.rotation = Quaternion.LookRotation(forward, spineDirection);
            }
        }
    }
    
    void UpdateControlTargets()
    {
        // Update target positions based on GrappleMap joints
        if (leftHandTarget != null)
        {
            // Use wrist position as primary target, but consider fingers for direction
            Vector3 handPos = currentJointPositions[LEFT_WRIST];
            Vector3 fingerDirection = (currentJointPositions[LEFT_FINGERS] - handPos).normalized;
            leftHandTarget.position = handPos + fingerDirection * 0.1f; // Offset slightly toward fingers
        }
        
        if (rightHandTarget != null)
        {
            Vector3 handPos = currentJointPositions[RIGHT_WRIST];
            Vector3 fingerDirection = (currentJointPositions[RIGHT_FINGERS] - handPos).normalized;
            rightHandTarget.position = handPos + fingerDirection * 0.1f;
        }
        
        if (leftFootTarget != null)
        {
            // Use ankle position but consider toe direction
            Vector3 anklePos = currentJointPositions[LEFT_ANKLE];
            Vector3 toeDirection = (currentJointPositions[LEFT_TOE] - anklePos).normalized;
            leftFootTarget.position = anklePos + toeDirection * 0.05f;
        }
        
        if (rightFootTarget != null)
        {
            Vector3 anklePos = currentJointPositions[RIGHT_ANKLE];
            Vector3 toeDirection = (currentJointPositions[RIGHT_TOE] - anklePos).normalized;
            rightFootTarget.position = anklePos + toeDirection * 0.05f;
        }
        
        if (headTarget != null)
        {
            headTarget.position = currentJointPositions[HEAD];
        }
    }
    
    void OnAnimatorIK(int layerIndex)
    {
        if (animator == null) return;
        
        // Apply IK for hands
        if (leftHandTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.LeftHand, leftHandIKWeight);
            animator.SetIKPosition(AvatarIKGoal.LeftHand, leftHandTarget.position);
            
            // Calculate hand rotation based on finger direction
            Vector3 handDirection = (currentJointPositions[LEFT_FINGERS] - currentJointPositions[LEFT_WRIST]).normalized;
            if (handDirection != Vector3.zero)
            {
                Quaternion handRotation = Quaternion.LookRotation(handDirection, Vector3.up);
                animator.SetIKRotationWeight(AvatarIKGoal.LeftHand, leftHandIKWeight * 0.5f);
                animator.SetIKRotation(AvatarIKGoal.LeftHand, handRotation);
            }
        }
        
        if (rightHandTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.RightHand, rightHandIKWeight);
            animator.SetIKPosition(AvatarIKGoal.RightHand, rightHandTarget.position);
            
            Vector3 handDirection = (currentJointPositions[RIGHT_FINGERS] - currentJointPositions[RIGHT_WRIST]).normalized;
            if (handDirection != Vector3.zero)
            {
                Quaternion handRotation = Quaternion.LookRotation(handDirection, Vector3.up);
                animator.SetIKRotationWeight(AvatarIKGoal.RightHand, rightHandIKWeight * 0.5f);
                animator.SetIKRotation(AvatarIKGoal.RightHand, handRotation);
            }
        }
        
        // Apply IK for feet
        if (leftFootTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.LeftFoot, leftFootIKWeight);
            animator.SetIKPosition(AvatarIKGoal.LeftFoot, leftFootTarget.position);
            
            // Calculate foot rotation based on toe direction
            Vector3 footDirection = (currentJointPositions[LEFT_TOE] - currentJointPositions[LEFT_ANKLE]).normalized;
            if (footDirection != Vector3.zero)
            {
                Quaternion footRotation = Quaternion.LookRotation(footDirection, Vector3.up);
                animator.SetIKRotationWeight(AvatarIKGoal.LeftFoot, leftFootIKWeight * 0.7f);
                animator.SetIKRotation(AvatarIKGoal.LeftFoot, footRotation);
            }
        }
        
        if (rightFootTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.RightFoot, rightFootIKWeight);
            animator.SetIKPosition(AvatarIKGoal.RightFoot, rightFootTarget.position);
            
            Vector3 footDirection = (currentJointPositions[RIGHT_TOE] - currentJointPositions[RIGHT_ANKLE]).normalized;
            if (footDirection != Vector3.zero)
            {
                Quaternion footRotation = Quaternion.LookRotation(footDirection, Vector3.up);
                animator.SetIKRotationWeight(AvatarIKGoal.RightFoot, rightFootIKWeight * 0.7f);
                animator.SetIKRotation(AvatarIKGoal.RightFoot, footRotation);
            }
        }
        
        // Apply look-at for head
        if (headTarget != null && headIKWeight > 0f)
        {
            animator.SetLookAtWeight(headIKWeight, 0.3f, 0.8f, 0.5f, 0.7f);
            animator.SetLookAtPosition(headTarget.position + transform.forward * 2f); // Look forward from head
        }
    }
    
    void Update()
    {
        // Toggle target visibility
        if (Input.GetKeyDown(KeyCode.G))
        {
            showTargets = !showTargets;
            ToggleTargetVisibility();
        }
    }
    
    void ToggleTargetVisibility()
    {
        if (leftHandTarget != null) leftHandTarget.gameObject.SetActive(showTargets);
        if (rightHandTarget != null) rightHandTarget.gameObject.SetActive(showTargets);
        if (leftFootTarget != null) leftFootTarget.gameObject.SetActive(showTargets);
        if (rightFootTarget != null) rightFootTarget.gameObject.SetActive(showTargets);
        if (headTarget != null) headTarget.gameObject.SetActive(showTargets);
        if (rootTarget != null) rootTarget.gameObject.SetActive(showTargets);
    }
    
    void OnDrawGizmos()
    {
        if (currentJointPositions == null || currentJointPositions.Length == 0) return;
        
        // Draw connections from humanoid bones to targets
        Gizmos.color = Color.cyan;
        
        if (animator != null)
        {
            // Draw lines from actual bone positions to target positions
            if (leftHandTarget != null)
            {
                Transform leftHand = animator.GetBoneTransform(HumanBodyBones.LeftHand);
                if (leftHand != null)
                    Gizmos.DrawLine(leftHand.position, leftHandTarget.position);
            }
            
            if (rightHandTarget != null)
            {
                Transform rightHand = animator.GetBoneTransform(HumanBodyBones.RightHand);
                if (rightHand != null)
                    Gizmos.DrawLine(rightHand.position, rightHandTarget.position);
            }
            
            if (leftFootTarget != null)
            {
                Transform leftFoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
                if (leftFoot != null)
                    Gizmos.DrawLine(leftFoot.position, leftFootTarget.position);
            }
            
            if (rightFootTarget != null)
            {
                Transform rightFoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
                if (rightFoot != null)
                    Gizmos.DrawLine(rightFoot.position, rightFootTarget.position);
            }
        }
    }
}