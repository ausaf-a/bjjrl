using System;
using System.Linq;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;

public class DebugVisualizerOld : MonoBehaviour
{

    [Header("Core Setup")]
    public EnhancedPoseLoader poseLoader;
    public Transform container;

    public Transform yc;  

    [Header("Visualization Objects")]
    public GameObject yBotPrefab; // Assign your Y Bot FBX here
    public GameObject jointTarget; // Assign your joint sphere here



    [Header("Visualization Parameters")]
    [Range(0.1f, 50.0f)] public float globalScale = 0.18f;  // Increased default scale

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        if (poseLoader == null)
        {
            poseLoader = FindFirstObjectByType<EnhancedPoseLoader>();
        }

        GrappleMapPosition currentPos = poseLoader.GetPosition("pos_454");
        Debug.Log($"Set position: {currentPos.description}");

        for (int player = 0; player < 2; player++)
        {
            for (int joint = 0; joint < 23; joint++)
            {
                Vector3 jointPos = poseLoader.GetJointPosition(currentPos, player, joint);
                jointPos *= globalScale;

                GameObject sphere = Instantiate(jointTarget);
                sphere.transform.SetParent(container);
                sphere.transform.localPosition = jointPos;

                var renderer = sphere.GetComponent<Renderer>();
                {
                    renderer.material.color = player == 0 ? Color.blue : Color.green;
                }

            }
        }
        

        GameObject yBot = Instantiate(yBotPrefab);
        yBot.transform.SetParent(yc);
        yBot.transform.localPosition = Vector3.zero;

        // Get all joints from the Y Bot
        Transform[] yBotJoints = yBot.GetComponentsInChildren<Transform>();
        string[] reduced = {
            "LeftLeg", "RightLeg", // Knees
            "LeftFoot", "RightFoot", // Ankles
            "LeftToe_End", "RightToe_End", // Toes
            "LeftUpLeg", "RightUpLeg", // Hips
            "Spine1", // Core
            "Neck", // Neck, 
            "HeadTop_End", // Head, 
            "LeftArm", "RightArm", // Shoulders
            "LeftForeArm", "RightForeArm", // Elbows
            "LeftHand", "RightHand", // Wrists
            "LeftHandMiddle1", "RightHandMiddle1", // Hands
            "LeftHandMiddle4", "RightHandMiddle4", // Fingertips
        };
        foreach (Transform joint in yBotJoints)
        {
            // Skip the root transform
            if (joint == yBot.transform) continue;

            string[] sp = joint.name.Split(':');
            

            if (sp.Length < 2 || !reduced.Contains(sp[1])) continue; 



            GameObject jointViz = Instantiate(jointTarget);
            jointViz.transform.SetParent(yc);
            jointViz.transform.position = joint.position;
            jointViz.name = $"YBot_{joint.name}";

            var renderer = jointViz.GetComponent<Renderer>();
            renderer.material.color = Color.red; // Red for Y Bot joints

            // Make them slightly larger to distinguish from pose joints
            jointViz.transform.localScale *= 1.5f;
        }
         
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
