using System;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEditor;
using UnityEditor.Animations;
using UnityEngine;
using UnityEngine.Animations.Rigging;

public class DebugVisualizer : MonoBehaviour
{
    [Header("Core Setup")]
    public EnhancedPoseLoader poseLoader;
    public Transform container;
    public Transform yc;

    [Header("Visualization Objects")]
    public GameObject yBotPrefab;
    public GameObject jointTarget;

    [Header("Visualization Parameters")]
    [Range(0.1f, 50.0f)] public float globalScale = 0.18f;
    [Range(1.0f, 10.0f)] public float jointScaleFactor = 3.0f;

    [Header("Physics Test")]
    public bool enablePhysicsTest = true;
    public int testSphereCount = 5;

    private GameObject yBot;
    
    private readonly string[] jointNames = {
        "LeftToe", "RightToe", "LeftHeel", "RightHeel", "LeftAnkle", "RightAnkle",
        "LeftKnee", "RightKnee", "LeftHip", "RightHip", "LeftShoulder", "RightShoulder",
        "LeftElbow", "RightElbow", "LeftWrist", "RightWrist", "LeftHand", "RightHand",
        "LeftFingers", "RightFingers", "Core", "Neck", "Head"
    };

    // Joint radii from GrappleMap jointDefs
    private readonly float[] jointRadii = {
        0.025f, 0.025f, 0.03f, 0.03f, 0.03f, 0.03f,     // toes, heels, ankles
        0.05f, 0.05f, 0.09f, 0.09f, 0.08f, 0.08f,       // knees, hips, shoulders
        0.045f, 0.045f, 0.02f, 0.02f, 0.02f, 0.02f,     // elbows, wrists, hands
        0.02f, 0.02f, 0.1f, 0.05f, 0.11f                // fingers, core, neck, head
    };

    // Limb connections (joint pairs that should be connected with cylinders)
    private readonly int[,] limbs = {
        {0, 2}, {1, 3},     // toe to heel
        {2, 4}, {3, 5},     // heel to ankle
        {4, 6}, {5, 7},     // ankle to knee
        {6, 8}, {7, 9},     // knee to hip
        {8, 20}, {9, 20},   // hip to core
        {20, 21},           // core to neck
        {21, 22},           // neck to head
        {10, 20}, {11, 20}, // shoulder to core
        {10, 12}, {11, 13}, // shoulder to elbow
        {12, 14}, {13, 15}, // elbow to wrist
        {14, 16}, {15, 17}, // wrist to hand
        {16, 18}, {17, 19}, // hand to fingers
        {21, 10}, {21, 11}, // neck to shoulders
        {8, 9},             // hip to hip
        {0, 4}, {1, 5}      // toe to ankle
    };

    void Start()
    {
        if (poseLoader == null)
        {
            poseLoader = FindFirstObjectByType<EnhancedPoseLoader>();
        }

        GrappleMapPosition currentPos = poseLoader.GetPosition("pos_454");
        Debug.Log($"Set position: {currentPos.description}");

        GameObject player1Obj = new GameObject("PhysicsPlayer1");
        GameObject player2Obj = new GameObject("PhysicsPlayer2");
        
        player1Obj.transform.SetParent(container);
        player2Obj.transform.SetParent(container);
        
        Player player1 = player1Obj.AddComponent<Player>();
        Player player2 = player2Obj.AddComponent<Player>();
        
        player1.playerColor = Color.red;
        player2.playerColor = Color.blue;
        
        player1.Initialize();
        player2.Initialize();
        player1.SetPoseFromGrappleMap(currentPos, 0, globalScale);
        player2.SetPoseFromGrappleMap(currentPos, 1, globalScale);
    }


    Material CreateMaterial(Color color)
    {
        Material material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        if (material.shader == null)
        {
            material = new Material(Shader.Find("Standard"));
        }
        if (material.shader == null)
        {
            material = new Material(Shader.Find("Legacy Shaders/Diffuse"));
        }
        material.color = color;
        return material;
    }

    void Update()
    {
        
    }
}