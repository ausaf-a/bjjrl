using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;

public class PoseLoader : MonoBehaviour
{
    [System.Serializable]
    public class PoseData
    {
        public string description;
        public string tags;
        public int is_position;
        public int is_transition;
        public List<List<float>> poses;
    }

    public List<PoseData> loadedPoses;

    void Awake()
    {
        TextAsset jsonFile = Resources.Load<TextAsset>("positions");
        loadedPoses = JsonConvert.DeserializeObject<List<PoseData>>(jsonFile.text);

        Debug.Log($"Loaded {loadedPoses.Count} poses");
        Debug.Log($"First pose description: {loadedPoses[0].description}");
        Debug.Log($"First pose poses: {loadedPoses[0].poses}");
    }

}
