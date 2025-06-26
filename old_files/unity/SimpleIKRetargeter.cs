using UnityEngine;

public class SimpleIKRetargeter : MonoBehaviour
{
    [Header("Player Configuration")]
    public int playerIndex = 0;
    
    [Header("IK Target Objects")]
    public Transform leftHandTarget;
    public Transform rightHandTarget;
    public Transform leftFootTarget;
    public Transform rightFootTarget;
    
    [Header("Auto Setup")]
    public bool createTargetsAutomatically = true;
    public float targetSize = 0.05f;
    
    [Header("IK Weights")]
    [Range(0f, 1f)] public float leftHandWeight = 1f;
    [Range(0f, 1f)] public float rightHandWeight = 1f;
    [Range(0f, 1f)] public float leftFootWeight = 1f;
    [Range(0f, 1f)] public float rightFootWeight = 1f;
    
    [Header("Debug")]
    public bool showTargets = true;
    public bool logSetup = true;
    
    private Animator animator;
    
    // GrappleMap joint indices
    private const int LEFT_WRIST = 14, RIGHT_WRIST = 15;
    private const int LEFT_ANKLE = 4, RIGHT_ANKLE = 5;
    private const int HEAD = 22, CORE = 20;
    
    void Start()
    {
        SetupIKSystem();
    }
    
    public void SetupIKSystem()
    {
        if (logSetup) Debug.Log($"Setting up simple IK system for player {playerIndex}");
        
        // Get required components
        animator = GetComponent<Animator>();
        if (animator == null)
        {
            Debug.LogError("No Animator found! Make sure Y Bot has Animator component.");
            return;
        }
        
        // Check if it's a humanoid
        if (animator.avatar == null || !animator.avatar.isHuman)
        {
            Debug.LogError("Y Bot must be set up as Humanoid in Import Settings!");
            return;
        }
        
        // Create a simple animation controller if none exists
        if (animator.runtimeAnimatorController == null)
        {
            CreateSimpleAnimatorController();
        }
        
        // Create targets if needed
        if (createTargetsAutomatically)
        {
            CreateIKTargets();
        }
        
        if (logSetup) Debug.Log("Simple IK system setup complete!");
    }
    
    [Header("Animation Controller")]
    public RuntimeAnimatorController animatorController; // Assign in GrappleMapVisualizer
    
    void CreateSimpleAnimatorController()
    {
        // Try to load the controller if it exists
        if (animatorController == null)
        {
            animatorController = Resources.Load<RuntimeAnimatorController>("SimpleIK_Controller");
        }
        
        if (animatorController != null)
        {
            animator.runtimeAnimatorController = animatorController;
            if (logSetup) Debug.Log("Assigned animator controller for IK");
        }
        else
        {
            if (logSetup) Debug.LogWarning("No animator controller found - IK may not work. Please create SimpleIK_Controller in Resources folder.");
        }
    }
    
    void CreateIKTargets()
    {
        if (logSetup) Debug.Log("Creating IK targets...");
        
        // Create target container
        GameObject targetContainer = new GameObject($"Player{playerIndex + 1}_IKTargets");
        targetContainer.transform.SetParent(transform.parent);
        
        // Define target colors for easy identification
        Color handColor = playerIndex == 0 ? Color.red : Color.blue;
        Color footColor = playerIndex == 0 ? new Color(1f, 0.5f, 0.5f) : new Color(0.5f, 0.5f, 1f);
        
        // Create individual targets at reasonable default positions
        Transform rightHand = animator.GetBoneTransform(HumanBodyBones.RightHand);
        Transform leftHand = animator.GetBoneTransform(HumanBodyBones.LeftHand);
        Transform rightFoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
        Transform leftFoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
        
        // Create targets at current bone positions
        leftHandTarget = CreateTarget("LeftHandTarget", targetContainer.transform, handColor, 
            leftHand != null ? leftHand.position : transform.position + Vector3.left);
        rightHandTarget = CreateTarget("RightHandTarget", targetContainer.transform, handColor,
            rightHand != null ? rightHand.position : transform.position + Vector3.right);
        leftFootTarget = CreateTarget("LeftFootTarget", targetContainer.transform, footColor,
            leftFoot != null ? leftFoot.position : transform.position + Vector3.left + Vector3.down);
        rightFootTarget = CreateTarget("RightFootTarget", targetContainer.transform, footColor,
            rightFoot != null ? rightFoot.position : transform.position + Vector3.right + Vector3.down);
        
        if (logSetup) Debug.Log($"Created {4} IK targets");
    }
    
    Transform CreateTarget(string name, Transform parent, Color color, Vector3 position)
    {
        // Create target GameObject
        GameObject targetGO = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        targetGO.name = $"P{playerIndex + 1}_{name}";
        targetGO.transform.SetParent(parent);
        targetGO.transform.position = position;
        targetGO.transform.localScale = Vector3.one * targetSize;
        
        // Remove collider to avoid physics interference
        Collider collider = targetGO.GetComponent<Collider>();
        if (collider != null) DestroyImmediate(collider);
        
        // Set color
        Renderer renderer = targetGO.GetComponent<Renderer>();
        Material mat = new Material(Shader.Find("Standard"));
        mat.color = color;
        renderer.material = mat;
        
        // Control visibility
        targetGO.SetActive(showTargets);
        
        return targetGO.transform;
    }
    
    void OnAnimatorIK(int layerIndex)
    {
        if (animator == null) return;
        
        // Apply IK for hands
        if (leftHandTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.LeftHand, leftHandWeight);
            animator.SetIKPosition(AvatarIKGoal.LeftHand, leftHandTarget.position);
        }
        
        if (rightHandTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.RightHand, rightHandWeight);
            animator.SetIKPosition(AvatarIKGoal.RightHand, rightHandTarget.position);
        }
        
        // Apply IK for feet
        if (leftFootTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.LeftFoot, leftFootWeight);
            animator.SetIKPosition(AvatarIKGoal.LeftFoot, leftFootTarget.position);
        }
        
        if (rightFootTarget != null)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.RightFoot, rightFootWeight);
            animator.SetIKPosition(AvatarIKGoal.RightFoot, rightFootTarget.position);
        }
    }
    
    public void ApplyPose(Vector3[] jointPositions)
    {
        if (jointPositions.Length < 23)
        {
            Debug.LogError($"Expected at least 23 joint positions, got {jointPositions.Length}");
            return;
        }
        
        // Move IK targets to GrappleMap joint positions
        if (leftHandTarget != null)
            leftHandTarget.position = jointPositions[LEFT_WRIST];
        
        if (rightHandTarget != null)
            rightHandTarget.position = jointPositions[RIGHT_WRIST];
        
        if (leftFootTarget != null)
            leftFootTarget.position = jointPositions[LEFT_ANKLE];
        
        if (rightFootTarget != null)
            rightFootTarget.position = jointPositions[RIGHT_ANKLE];
        
        // Position the character root at core position
        Vector3 corePos = jointPositions[CORE];
        transform.position = corePos;
        
        if (logSetup) Debug.Log($"Applied pose for player {playerIndex}");
    }
    
    public void ToggleTargetVisibility()
    {
        showTargets = !showTargets;
        
        if (leftHandTarget != null) leftHandTarget.gameObject.SetActive(showTargets);
        if (rightHandTarget != null) rightHandTarget.gameObject.SetActive(showTargets);
        if (leftFootTarget != null) leftFootTarget.gameObject.SetActive(showTargets);
        if (rightFootTarget != null) rightFootTarget.gameObject.SetActive(showTargets);
    }
    
    void Update()
    {
        // Toggle target visibility with G key
        if (Input.GetKeyDown(KeyCode.G))
        {
            ToggleTargetVisibility();
        }
    }
    
    void OnDrawGizmos()
    {
        // Draw connections from bones to targets for debugging
        if (animator == null || !showTargets) return;
        
        Gizmos.color = Color.yellow;
        
        // Draw lines from actual bone positions to target positions
        DrawGizmoConnection(HumanBodyBones.LeftHand, leftHandTarget);
        DrawGizmoConnection(HumanBodyBones.RightHand, rightHandTarget);
        DrawGizmoConnection(HumanBodyBones.LeftFoot, leftFootTarget);
        DrawGizmoConnection(HumanBodyBones.RightFoot, rightFootTarget);
    }
    
    void DrawGizmoConnection(HumanBodyBones bone, Transform target)
    {
        if (target == null) return;
        
        Transform boneTransform = animator.GetBoneTransform(bone);
        if (boneTransform != null)
        {
            Gizmos.DrawLine(boneTransform.position, target.position);
        }
    }
}