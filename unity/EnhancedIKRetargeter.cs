using UnityEngine;
using UnityEngine.Animations.Rigging;

public class EnhancedIKRetargeter : MonoBehaviour
{
    [Header("Player Configuration")]
    public int playerIndex = 0;
    
    [Header("IK Target Objects")]
    [Tooltip("These will be created automatically if not assigned")]
    public Transform leftHandTarget;
    public Transform rightHandTarget;
    public Transform leftFootTarget;
    public Transform rightFootTarget;
    public Transform headTarget;
    public Transform hipTarget;
    
    [Header("Auto Setup")]
    public bool createTargetsAutomatically = true;
    public float targetSize = 0.05f;
    
    [Header("IK Components")]
    [Tooltip("These will be set up automatically")]
    public TwoBoneIKConstraint leftArmIK;
    public TwoBoneIKConstraint rightArmIK;
    public TwoBoneIKConstraint leftLegIK;
    public TwoBoneIKConstraint rightLegIK;
    public MultiAimConstraint headConstraint;
    
    [Header("Debug")]
    public bool showTargets = true;
    public bool logSetup = true;
    
    private Animator animator;
    private RigBuilder rigBuilder;
    private Rig mainRig;
    
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
        if (logSetup) Debug.Log($"Setting up IK system for player {playerIndex}");
        
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
        
        // Set up rig builder
        SetupRigBuilder();
        
        // Create targets if needed
        if (createTargetsAutomatically)
        {
            CreateIKTargets();
        }
        
        // Set up IK constraints
        SetupIKConstraints();
        
        if (logSetup) Debug.Log("IK system setup complete!");
    }
    
    void SetupRigBuilder()
    {
        // Add RigBuilder if not present
        rigBuilder = GetComponent<RigBuilder>();
        if (rigBuilder == null)
        {
            rigBuilder = gameObject.AddComponent<RigBuilder>();
        }
        
        // Create main rig GameObject
        GameObject rigGO = new GameObject("MainRig");
        rigGO.transform.SetParent(transform);
        mainRig = rigGO.AddComponent<Rig>();
        mainRig.weight = 1f;
        
        // Add to rig builder
        rigBuilder.layers.Add(new RigLayer(mainRig, true));
    }
    
    void CreateIKTargets()
    {
        if (logSetup) Debug.Log("Creating IK targets...");
        
        // Create target container
        GameObject targetContainer = new GameObject($"Player{playerIndex + 1}_IKTargets");
        targetContainer.transform.SetParent(transform.parent);
        
        // Define target colors for easy identification
        Color player1Color = Color.red;
        Color player2Color = Color.blue;
        Color currentColor = playerIndex == 0 ? player1Color : player2Color;
        
        // Create individual targets
        leftHandTarget = CreateTarget("LeftHandTarget", targetContainer.transform, currentColor);
        rightHandTarget = CreateTarget("RightHandTarget", targetContainer.transform, currentColor);
        leftFootTarget = CreateTarget("LeftFootTarget", targetContainer.transform, Color.cyan);
        rightFootTarget = CreateTarget("RightFootTarget", targetContainer.transform, Color.cyan);
        headTarget = CreateTarget("HeadTarget", targetContainer.transform, Color.yellow);
        hipTarget = CreateTarget("HipTarget", targetContainer.transform, Color.green);
        
        if (logSetup) Debug.Log($"Created {6} IK targets");
    }
    
    Transform CreateTarget(string name, Transform parent, Color color)
    {
        // Create target GameObject
        GameObject targetGO = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        targetGO.name = $"P{playerIndex + 1}_{name}";
        targetGO.transform.SetParent(parent);
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
    
    void SetupIKConstraints()
    {
        if (logSetup) Debug.Log("Setting up IK constraints...");
        
        // Set up left arm IK
        leftArmIK = SetupTwoBoneIK("LeftArmIK", 
            HumanBodyBones.LeftShoulder, 
            HumanBodyBones.LeftUpperArm, 
            HumanBodyBones.LeftLowerArm, 
            HumanBodyBones.LeftHand,
            leftHandTarget);
        
        // Set up right arm IK
        rightArmIK = SetupTwoBoneIK("RightArmIK",
            HumanBodyBones.RightShoulder,
            HumanBodyBones.RightUpperArm,
            HumanBodyBones.RightLowerArm,
            HumanBodyBones.RightHand,
            rightHandTarget);
        
        // Set up left leg IK
        leftLegIK = SetupTwoBoneIK("LeftLegIK",
            HumanBodyBones.LeftUpperLeg,
            HumanBodyBones.LeftUpperLeg,
            HumanBodyBones.LeftLowerLeg,
            HumanBodyBones.LeftFoot,
            leftFootTarget);
        
        // Set up right leg IK
        rightLegIK = SetupTwoBoneIK("RightLegIK",
            HumanBodyBones.RightUpperLeg,
            HumanBodyBones.RightUpperLeg,
            HumanBodyBones.RightLowerLeg,
            HumanBodyBones.RightFoot,
            rightFootTarget);
        
        // Set up head constraint
        SetupHeadConstraint();
        
        if (logSetup) Debug.Log("All IK constraints set up!");
    }
    
    TwoBoneIKConstraint SetupTwoBoneIK(string name, HumanBodyBones rootBone, HumanBodyBones midBone, HumanBodyBones tipBone, HumanBodyBones targetBone, Transform target)
    {
        // Create constraint GameObject
        GameObject constraintGO = new GameObject(name);
        constraintGO.transform.SetParent(mainRig.transform);
        
        // Add TwoBoneIK constraint
        TwoBoneIKConstraint constraint = constraintGO.AddComponent<TwoBoneIKConstraint>();
        
        // Set up constraint data
        constraint.data.root = animator.GetBoneTransform(rootBone);
        constraint.data.mid = animator.GetBoneTransform(midBone);
        constraint.data.tip = animator.GetBoneTransform(tipBone);
        constraint.data.target = target;
        
        // Use the correct property names for newer Animation Rigging versions
        constraint.weight = 1f;
        
        // Create hint target (for elbow/knee positioning)
        GameObject hintGO = new GameObject($"{name}_Hint");
        hintGO.transform.SetParent(constraintGO.transform);
        constraint.data.hint = hintGO.transform;
        
        // Position hint appropriately
        if (constraint.data.mid != null)
        {
            hintGO.transform.position = constraint.data.mid.position;
        }
        
        return constraint;
    }
    
    void SetupHeadConstraint()
    {
        // Create head constraint GameObject
        GameObject headConstraintGO = new GameObject("HeadConstraint");
        headConstraintGO.transform.SetParent(mainRig.transform);
        
        // Add MultiAim constraint for head
        headConstraint = headConstraintGO.AddComponent<MultiAimConstraint>();
        
        // Set up constraint data
        Transform headBone = animator.GetBoneTransform(HumanBodyBones.Head);
        if (headBone != null)
        {
            headConstraint.data.constrainedObject = headBone;
            
            // Add head target - use simpler approach for newer versions
            var sourceObjects = new WeightedTransformArray(1);
            sourceObjects[0] = new WeightedTransform(headTarget, 1f);
            headConstraint.data.sourceObjects = sourceObjects;
            
            // Set weight
            headConstraint.weight = 0.5f; // Lighter weight for head to avoid neck strain
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
        
        if (headTarget != null)
            headTarget.position = jointPositions[HEAD];
        
        if (hipTarget != null)
        {
            // Calculate hip center from hip joints
            Vector3 leftHip = jointPositions[8];  // LEFT_HIP
            Vector3 rightHip = jointPositions[9]; // RIGHT_HIP
            hipTarget.position = (leftHip + rightHip) / 2f;
        }
        
        // Position the character root
        Vector3 corePos = jointPositions[CORE];
        transform.position = corePos;
    }
    
    public void ToggleTargetVisibility()
    {
        showTargets = !showTargets;
        
        if (leftHandTarget != null) leftHandTarget.gameObject.SetActive(showTargets);
        if (rightHandTarget != null) rightHandTarget.gameObject.SetActive(showTargets);
        if (leftFootTarget != null) leftFootTarget.gameObject.SetActive(showTargets);
        if (rightFootTarget != null) rightFootTarget.gameObject.SetActive(showTargets);
        if (headTarget != null) headTarget.gameObject.SetActive(showTargets);
        if (hipTarget != null) hipTarget.gameObject.SetActive(showTargets);
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
        
        Gizmos.color = Color.cyan;
        
        // Draw lines from actual bone positions to target positions
        DrawGizmoConnection(HumanBodyBones.LeftHand, leftHandTarget);
        DrawGizmoConnection(HumanBodyBones.RightHand, rightHandTarget);
        DrawGizmoConnection(HumanBodyBones.LeftFoot, leftFootTarget);
        DrawGizmoConnection(HumanBodyBones.RightFoot, rightFootTarget);
        DrawGizmoConnection(HumanBodyBones.Head, headTarget);
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