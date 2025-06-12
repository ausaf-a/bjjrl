using System.Collections.Generic;
using UnityEngine;

public class MannequinBuilder : MonoBehaviour
{
    public PoseLoader poseLoader; // Assign in Inspector
    public float positionScale = 0.001f;
    public Transform container; // Set to an empty GameObject in scene

    private Dictionary<(int, int), GameObject> jointObjects = new();
    private Dictionary<int, string> jointNames = new();
    private Dictionary<int, float> jointScales = new();

    private readonly int[] bones = new int[]
    {
        0, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 7,
        6, 8, 7, 9, 8, 20, 9, 20, 10, 20, 11, 20,
        10, 12, 11, 13, 12, 14, 13, 15, 14, 16, 15, 17,
        20, 21, 21, 22
    };

    void Awake()
    {
        InitJointNames();
        InitJointScales();
    }

    void Start()
    {
        ClearOld();
        BuildMannequins();
    }

    void InitJointNames()
    {
        string[] names = {
            "LeftToe", "RightToe", "LeftHeel", "RightHeel", "LeftAnkle", "RightAnkle",
            "LeftKnee", "RightKnee", "LeftHip", "RightHip", "LeftShoulder", "RightShoulder",
            "LeftElbow", "RightElbow", "LeftWrist", "RightWrist", "LeftHand", "RightHand",
            "LeftFingers", "RightFingers", "Core", "Neck", "Head"
        };
        for (int i = 0; i < names.Length; i++) jointNames[i] = names[i];
    }

    void InitJointScales()
    {
        float[] sizes = {
            0.03f, 0.03f, 0.03f, 0.03f, 0.05f, 0.05f,
            0.07f, 0.07f, 0.08f, 0.08f, 0.08f, 0.08f,
            0.07f, 0.07f, 0.06f, 0.06f, 0.05f, 0.05f,
            0.04f, 0.04f, 0.1f, 0.08f, 0.09f
        };
        for (int i = 0; i < sizes.Length; i++) jointScales[i] = sizes[i];
    }

    void ClearOld()
    {
        foreach (Transform child in container) Destroy(child.gameObject);
        jointObjects.Clear();
    }

    void BuildMannequins()
    {
        var pose = poseLoader.loadedPoses[0].poses;

        for (int i = 0; i < pose.Count; i++)
        {
            int player = i < 23 ? 0 : 1;
            int jointIndex = i % 23;
            Vector3 pos = new Vector3(pose[i][0], pose[i][1], pose[i][2]) * positionScale;

            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.SetParent(container);
            sphere.transform.localPosition = pos;
            float scale = jointScales[jointIndex];
            sphere.transform.localScale = Vector3.one * scale;
            sphere.name = $"P{player}_{jointNames[jointIndex]}";

            // var mat = new Material(Shader.Find("Standard"));
            // mat.color = player == 0 ? Color.red : Color.blue;
            // sphere.GetComponent<Renderer>().material = mat;

            jointObjects[(player, jointIndex)] = sphere;
        }

        // Bones
        for (int i = 0; i < bones.Length; i += 2)
        {
            int j1 = bones[i];
            int j2 = bones[i + 1];

            foreach (int player in new[] { 0, 1 })
            {
                if (!jointObjects.ContainsKey((player, j1)) || !jointObjects.ContainsKey((player, j2))) continue;

                Vector3 p1 = jointObjects[(player, j1)].transform.localPosition;
                Vector3 p2 = jointObjects[(player, j2)].transform.localPosition;

                GameObject bone = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                bone.transform.SetParent(container);
                bone.transform.localScale = new Vector3(
                    (jointScales[j1] + jointScales[j2]) / 2 * 0.5f,
                    Vector3.Distance(p1, p2) / 2,
                    (jointScales[j1] + jointScales[j2]) / 2 * 0.5f);

                bone.transform.localPosition = (p1 + p2) / 2;
                bone.transform.up = (p2 - p1).normalized;
                bone.name = $"P{player}_Bone_{jointNames[j1]}_{jointNames[j2]}";

                var mat = new Material(Shader.Find("Standard"));
                mat.color = player == 0 ? Color.red : Color.blue;
                bone.GetComponent<Renderer>().material = mat;
            }
        }
    }
}
