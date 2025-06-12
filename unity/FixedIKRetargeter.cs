using UnityEngine;

public class FixedIKRetargeter : MonoBehaviour
{
    [Header("IK Targets")]
    public Transform leftHandTarget;
    public Transform rightHandTarget;
    public Transform leftFootTarget;
    public Transform rightFootTarget;
    public Transform headTarget;
    public Transform rootTarget; // For hips/root motion
    
    [Header("IK Settings")]
    [Range(0f, 1f)] public float handIKWeight = 1f;
    [Range(0f, 1f)] public float footIKWeight = 1f;
    [Range(0f, 1f)] public float headIKWeight = 1f;
    
    [Header("Debug")]
    public bool enableDebugLogs = false;
    public bool enableDebugGizmos = true;
    
    private Animator animator;
    private bool isInitialized = false;
    
    void Start()
    {
        InitializeIK();
    }
    
    void InitializeIK()
    {
        animator = GetComponent<Animator>();
        
        if (animator == null)
        {
            Debug.LogError($"No Animator found on {gameObject.name}!");
            return;
        }
        
        // Critical: Ensure animator is enabled and has proper setup
        animator.enabled = true;
        
        // Check if we have a humanoid avatar
        if (!animator.isHuman)
        {
            Debug.LogError($"{gameObject.name} is not set up as Humanoid! Check import settings.");
            return;
        }
        
        // Check if we have an animator controller
        if (animator.runtimeAnimatorController == null)
        {
            Debug.LogWarning($"{gameObject.name} has no Animator Controller. Creating basic one...");
            CreateBasicAnimatorController();
        }
        
        isInitialized = true;
        
        if (enableDebugLogs)
        {
            Debug.Log($"IK initialized for {gameObject.name}");
            Debug.Log($"Avatar: {animator.avatar?.name}");
            Debug.Log($"Controller: {animator.runtimeAnimatorController?.name}");
        }
    }
    
    void CreateBasicAnimatorController()
    {
        // Create a minimal animator controller to ensure OnAnimatorIK gets called
        var controller = new UnityEditor.Animations.AnimatorController();
        controller.name = "BasicIKController";
        
        // Add a default state
        var rootStateMachine = controller.layers[0].stateMachine;
        var idleState = rootStateMachine.AddState("Idle");
        rootStateMachine.defaultState = idleState;
        
        // Assign to animator
        animator.runtimeAnimatorController = controller;
        
        Debug.Log($"Created basic animator controller for {gameObject.name}");
    }
    
    void OnAnimatorIK(int layerIndex)
    {
        if (!isInitialized || animator == null) return;
        
        if (enableDebugLogs)
        {
            Debug.Log($"OnAnimatorIK called for {gameObject.name}");
        }
        
        // Apply hand IK
        if (leftHandTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.LeftHand, handIKWeight);
            animator.SetIKPosition(AvatarIKGoal.LeftHand, leftHandTarget.position);
            
            animator.SetIKRotationWeight(AvatarIKGoal.LeftHand, handIKWeight);
            animator.SetIKRotation(AvatarIKGoal.LeftHand, leftHandTarget.rotation);
        }
        
        if (rightHandTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.RightHand, handIKWeight);
            animator.SetIKPosition(AvatarIKGoal.RightHand, rightHandTarget.position);
            
            animator.SetIKRotationWeight(AvatarIKGoal.RightHand, handIKWeight);
            animator.SetIKRotation(AvatarIKGoal.RightHand, rightHandTarget.rotation);
        }
        
        // Apply foot IK
        if (leftFootTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.LeftFoot, footIKWeight);
            animator.SetIKPosition(AvatarIKGoal.LeftFoot, leftFootTarget.position);
            
            animator.SetIKRotationWeight(AvatarIKGoal.LeftFoot, footIKWeight);
            animator.SetIKRotation(AvatarIKGoal.LeftFoot, leftFootTarget.rotation);
        }
        
        if (rightFootTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.RightFoot, footIKWeight);
            animator.SetIKPosition(AvatarIKGoal.RightFoot, rightFootTarget.position);
            
            animator.SetIKRotationWeight(AvatarIKGoal.RightFoot, footIKWeight);
            animator.SetIKRotation(AvatarIKGoal.RightFoot, rightFootTarget.rotation);
        }
        
        // Apply head/look IK
        if (headTarget != null)
        {
            animator.SetLookAtWeight(headIKWeight);
            animator.SetLookAtPosition(headTarget.position);
        }
        
        // Root motion for hips
        if (rootTarget != null)
        {
            // For hip positioning, we might need to move the entire character
            Vector3 hipOffset = rootTarget.position - animator.GetBoneTransform(HumanBodyBones.Hips).position;
            transform.position += hipOffset;
        }
    }
    
    // Alternative approach: Direct bone manipulation (fallback if IK fails)
    public void ApplyDirectBonePositioning()
    {
        if (!animator.isHuman) return;
        
        // This is a more direct approach that bypasses Unity's IK system
        if (leftHandTarget != null)
        {
            Transform leftHand = animator.GetBoneTransform(HumanBodyBones.LeftHand);
            if (leftHand != null)
            {
                leftHand.position = leftHandTarget.position;
                leftHand.rotation = leftHandTarget.rotation;
            }
        }
        
        if (rightHandTarget != null)
        {
            Transform rightHand = animator.GetBoneTransform(HumanBodyBones.RightHand);
            if (rightHand != null)
            {
                rightHand.position = rightHandTarget.position;
                rightHand.rotation = rightHandTarget.rotation;
            }
        }
        
        // Similar for feet, head, etc.
    }
    
    void OnDrawGizmos()
    {
        if (!enableDebugGizmos) return;
        
        // Draw connections between character and targets
        if (animator != null && animator.isHuman)
        {
            DrawTargetConnection(animator.GetBoneTransform(HumanBodyBones.LeftHand), leftHandTarget, Color.red);
            DrawTargetConnection(animator.GetBoneTransform(HumanBodyBones.RightHand), rightHandTarget, Color.blue);
            DrawTargetConnection(animator.GetBoneTransform(HumanBodyBones.LeftFoot), leftFootTarget, Color.green);
            DrawTargetConnection(animator.GetBoneTransform(HumanBodyBones.RightFoot), rightFootTarget, Color.yellow);
            DrawTargetConnection(animator.GetBoneTransform(HumanBodyBones.Head), headTarget, Color.magenta);
        }
    }
    
    void DrawTargetConnection(Transform bone, Transform target, Color color)
    {
        if (bone != null && target != null)
        {
            Gizmos.color = color;
            Gizmos.DrawLine(bone.position, target.position);
            Gizmos.DrawWireSphere(target.position, 0.05f);
        }
    }
    
    // Manual test function you can call from inspector
    [ContextMenu("Test IK Movement")]
    public void TestIKMovement()
    {
        if (leftHandTarget != null)
        {
            leftHandTarget.position = transform.position + Vector3.forward * 2f + Vector3.up * 1.5f;
        }
    }
}

// USAGE INSTRUCTIONS:
// 1. Add this script to your Y Bot
// 2. Assign the target transforms in inspector
// 3. Make sure Y Bot is imported as Humanoid
// 4. Check that "Optimize Game Objects" is OFF in import settings
// 5. Test with "Test IK Movement" context menu option