using System.Collections.Generic;
using UnityEngine;

public class ApplyGrapplePose : MonoBehaviour
{
    public Transform[] joints = new Transform[23];  // Assign in Inspector
    public PoseLoader poseLoader;
    public float scale = 0.05f;                   // Drag your PoseLoader GameObject here

    void Start()
    {
        var pose3D = poseLoader.loadedPoses[0].poses;

        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null || i >= pose3D.Count) continue;

            Vector3 pos = new Vector3(pose3D[i][0], pose3D[i][1], pose3D[i][2]) * scale;
            joints[i].localPosition = pos;
        }
    }
}