using System.Collections.Generic;
using UnityEngine;

public class Player : MonoBehaviour
{
    [Header("Physics Settings")]
    public float totalMass = 70f;
    public float jointStrength = 1000f;
    public float jointDamping = 100f;

    [Header("Visual Settings")]
    public Color playerColor = Color.red;
    public float jointScaleFactor = 1.0f;

    private Dictionary<int, ArticulationBody> joints = new Dictionary<int, ArticulationBody>();
    private Dictionary<int, GameObject> jointVisuals = new Dictionary<int, GameObject>();
    private ArticulationBody rootBody;

    // Joint definitions from GrappleMap
    private readonly string[] jointNames = {
        "LeftToe", "RightToe", "LeftHeel", "RightHeel", "LeftAnkle", "RightAnkle",
        "LeftKnee", "RightKnee", "LeftHip", "RightHip", "LeftShoulder", "RightShoulder",
        "LeftElbow", "RightElbow", "LeftWrist", "RightWrist", "LeftHand", "RightHand",
        "LeftFingers", "RightFingers", "Core", "Neck", "Head"
    };

    private readonly float[] jointRadii = {
        0.025f, 0.025f, 0.03f, 0.03f, 0.03f, 0.03f,     // toes, heels, ankles
        0.05f, 0.05f, 0.09f, 0.09f, 0.08f, 0.08f,       // knees, hips, shoulders
        0.045f, 0.045f, 0.02f, 0.02f, 0.02f, 0.02f,     // elbows, wrists, hands
        0.02f, 0.02f, 0.1f, 0.05f, 0.11f                // fingers, core, neck, head
    };

    // Skeleton hierarchy (parent -> child relationships)
    private readonly Dictionary<int, int> parentJoints = new Dictionary<int, int>
    {
        // Core is root (20)
        {21, 20}, // Neck -> Core
        {22, 21}, // Head -> Neck
        {8, 20},  // LeftHip -> Core  
        {9, 20},  // RightHip -> Core
        {10, 20}, // LeftShoulder -> Core
        {11, 20}, // RightShoulder -> Core
        
        // Left leg chain
        {6, 8},   // LeftKnee -> LeftHip
        {4, 6},   // LeftAnkle -> LeftKnee
        {2, 4},   // LeftHeel -> LeftAnkle
        {0, 4},   // LeftToe -> LeftAnkle
        
        // Right leg chain  
        {7, 9},   // RightKnee -> RightHip
        {5, 7},   // RightAnkle -> RightKnee
        {3, 5},   // RightHeel -> RightAnkle
        {1, 5},   // RightToe -> RightAnkle
        
        // Left arm chain
        {12, 10}, // LeftElbow -> LeftShoulder
        {14, 12}, // LeftWrist -> LeftElbow
        {16, 14}, // LeftHand -> LeftWrist
        {18, 16}, // LeftFingers -> LeftHand
        
        // Right arm chain
        {13, 11}, // RightElbow -> RightShoulder
        {15, 13}, // RightWrist -> RightElbow
        {17, 15}, // RightHand -> RightWrist
        {19, 17}, // RightFingers -> RightHand
    };

    public void Initialize()
    {
        CreatePhysicsSkeleton();
        CreateVisuals();
    }

    public void SetPoseFromGrappleMap(GrappleMapPosition position, int playerIndex, float scale = 1.0f)
    {
        // Get target positions
        Vector3[] targetPositions = new Vector3[23];
        for (int i = 0; i < 23; i++)
        {
            // You'll need to pass your poseLoader here or make it static
            // targetPositions[i] = poseLoader.GetJointPosition(position, playerIndex, i) * scale;
        }

        // Apply positions to physics bodies
        ApplyTargetPose(targetPositions);
    }

    void CreatePhysicsSkeleton()
    {
        // Create root body (Core)
        rootBody = CreateJoint(20, Vector3.zero, null);
        joints[20] = rootBody;

        // Create child joints in hierarchy order
        foreach (var kvp in parentJoints)
        {
            int childJoint = kvp.Key;
            int parentJoint = kvp.Value;
            
            ArticulationBody parentBody = joints[parentJoint];
            Vector3 offset = Vector3.up * 0.2f; // Temporary offset, will be set by pose
            
            ArticulationBody childBody = CreateJoint(childJoint, offset, parentBody);
            joints[childJoint] = childBody;
        }
    }

    ArticulationBody CreateJoint(int jointIndex, Vector3 localPosition, ArticulationBody parent)
    {
        GameObject jointObj = new GameObject(jointNames[jointIndex]);
        jointObj.transform.SetParent(transform);
        
        ArticulationBody body = jointObj.AddComponent<ArticulationBody>();
        
        if (parent == null)
        {
            // Root body
            // body.isRoot = true;
            
            body.immovable = false;
        }
        else
        {
            // Child body
            body.parentAnchorPosition = Vector3.zero;
            body.anchorPosition = Vector3.zero;
            jointObj.transform.SetParent(parent.transform);
        }

        jointObj.transform.localPosition = localPosition;
        
        // Set up joint properties
        body.mass = totalMass / 23f; // Distribute mass evenly for now
        body.jointType = ArticulationJointType.SphericalJoint;
        
        // Set joint limits (can be refined per joint type)
        var drive = body.xDrive;
        drive.stiffness = jointStrength;
        drive.damping = jointDamping;
        drive.forceLimit = float.MaxValue;
        body.xDrive = body.yDrive = body.zDrive = drive;

        // Add collider
        SphereCollider collider = jointObj.AddComponent<SphereCollider>();
        collider.radius = jointRadii[jointIndex] * jointScaleFactor;

        return body;
    }

    void CreateVisuals()
    {
        // Create visual representations for each joint
        for (int i = 0; i < 23; i++)
        {
            if (joints.ContainsKey(i))
            {
                GameObject visual = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                visual.transform.SetParent(joints[i].transform);
                visual.transform.localPosition = Vector3.zero;
                visual.transform.localScale = Vector3.one * jointRadii[i] * jointScaleFactor * 2f;
                
                // Remove collider from visual (physics handled by ArticulationBody)
                DestroyImmediate(visual.GetComponent<SphereCollider>());
                
                var renderer = visual.GetComponent<Renderer>();
                Material material = new Material(Shader.Find("Standard"));
                material.color = playerColor;
                renderer.material = material;
                
                jointVisuals[i] = visual;
            }
        }
    }

    void ApplyTargetPose(Vector3[] targetPositions)
    {
        // This is where we'd apply forces/torques to reach target positions
        // For now, just move them directly (we can make this physics-based later)
        for (int i = 0; i < targetPositions.Length; i++)
        {
            if (joints.ContainsKey(i))
            {
                joints[i].transform.position = transform.position + targetPositions[i];
            }
        }
    }

    public ArticulationBody GetJoint(int jointIndex)
    {
        return joints.ContainsKey(jointIndex) ? joints[jointIndex] : null;
    }

    public void SetJointTarget(int jointIndex, Vector3 targetPosition)
    {
        if (joints.ContainsKey(jointIndex))
        {
            // Apply force toward target (implement physics-based movement)
            Vector3 currentPos = joints[jointIndex].transform.position;
            Vector3 force = (targetPosition - currentPos) * jointStrength;
            joints[jointIndex].AddForce(force);
        }
    }
}