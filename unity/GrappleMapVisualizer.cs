using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public enum VisualizationMode
{
    YBotHumanoids,
    DebugMannequins,
    Both
}

public class GrappleMapVisualizer : MonoBehaviour
{
    [Header("Core Setup")]
    public EnhancedPoseLoader poseLoader;
    public VisualizationMode visualizationMode = VisualizationMode.YBotHumanoids;
    
    [Header("Y Bot Humanoids")]
    public GameObject yBotPrefab; // Assign your Y Bot FBX here
    public Transform humanoidContainer;
    public Material player1Material;
    public Material player2Material;
    
    [Header("Debug Mannequins")]
    public Transform mannequinContainer;
    public Material mannequinPlayer1Material;
    public Material mannequinPlayer2Material;
    
    [Header("Visualization Parameters")]
    [Range(0.1f, 50.0f)] public float globalScale = 0.18f;  // Increased default scale
    [Range(0.01f, 0.2f)] public float jointScale = 0.05f;
    [Range(0.005f, 0.1f)] public float boneRadius = 0.02f;
    
    [Header("Current Position")]
    public string currentPositionId;
    [Range(0, 1)] public float animationTime = 0f; // For transition playback
    
    [Header("Debug")]
    public bool showJointLabels = false;
    public bool logPositionChanges = true;
    
    // Runtime data
    private GameObject[] yBotInstances = new GameObject[2];
    private SimpleIKRetargeter[] retargeters = new SimpleIKRetargeter[2];
    
    // Debug mannequin data
    private Dictionary<(int, int), GameObject> jointObjects = new Dictionary<(int, int), GameObject>();
    private Dictionary<(int, int), GameObject> boneObjects = new Dictionary<(int, int), GameObject>();
    
    // Joint definitions from GrappleMap
    private readonly string[] jointNames = {
        "LeftToe", "RightToe", "LeftHeel", "RightHeel", "LeftAnkle", "RightAnkle",
        "LeftKnee", "RightKnee", "LeftHip", "RightHip", "LeftShoulder", "RightShoulder",
        "LeftElbow", "RightElbow", "LeftWrist", "RightWrist", "LeftHand", "RightHand",
        "LeftFingers", "RightFingers", "Core", "Neck", "Head"
    };
    
    private readonly float[] jointSizes = {
        0.03f, 0.03f, 0.03f, 0.03f, 0.05f, 0.05f,
        0.07f, 0.07f, 0.08f, 0.08f, 0.08f, 0.08f,
        0.07f, 0.07f, 0.06f, 0.06f, 0.05f, 0.05f,
        0.04f, 0.04f, 0.1f, 0.08f, 0.09f
    };
    
    // Bone connections (joint index pairs)
    private readonly int[] boneConnections = {
        0, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 7,
        6, 8, 7, 9, 8, 20, 9, 20, 10, 20, 11, 20,
        10, 12, 11, 13, 12, 14, 13, 15, 14, 16, 15, 17,
        20, 21, 21, 22
    };
    
    void Start()
    {
        if (poseLoader == null)
        {
            poseLoader = FindObjectOfType<EnhancedPoseLoader>();
        }
        
        SetupVisualization();
        
        // Load first position if available
        if (string.IsNullOrEmpty(currentPositionId) && poseLoader.totalPositions > 0)
        {
            currentPositionId = poseLoader.positions.Keys.First();
        }
        
        UpdateVisualization();
    }
    
    void Update()
    {
        // Simple keyboard controls
        HandleKeyboardInput();
        
        // Real-time parameter updates
        if (HasVisualizationChanged())
        {
            UpdateVisualization();
        }
    }
    
    void HandleKeyboardInput()
    {
        if (poseLoader == null || poseLoader.positions == null) return;
        
        // Get all position IDs as array for navigation
        var positionIds = poseLoader.positions.Keys.ToArray();
        if (positionIds.Length == 0) return;
        
        // Find current index
        int currentIndex = System.Array.IndexOf(positionIds, currentPositionId);
        if (currentIndex == -1) currentIndex = 0;
        
        bool positionChanged = false;
        
        // Navigation controls
        if (Input.GetKeyDown(KeyCode.LeftArrow))
        {
            currentIndex = (currentIndex - 1 + positionIds.Length) % positionIds.Length;
            positionChanged = true;
        }
        else if (Input.GetKeyDown(KeyCode.RightArrow))
        {
            currentIndex = (currentIndex + 1) % positionIds.Length;
            positionChanged = true;
        }
        else if (Input.GetKeyDown(KeyCode.Space))
        {
            currentIndex = Random.Range(0, positionIds.Length);
            positionChanged = true;
        }
        else if (Input.GetKeyDown(KeyCode.R))
        {
            // Random position with specific tag
            var guardPositions = poseLoader.FindPositionsByTag("guard");
            if (guardPositions.Count > 0)
            {
                var randomGuard = guardPositions[Random.Range(0, guardPositions.Count)];
                currentPositionId = randomGuard.id;
                UpdateVisualization();
                if (logPositionChanges)
                    Debug.Log($"Random guard position: {randomGuard.description}");
                return;
            }
        }
        else if (Input.GetKeyDown(KeyCode.T))
        {
            // Toggle visualization mode
            ToggleVisualizationMode();
            return;
        }
        
        if (positionChanged)
        {
            currentPositionId = positionIds[currentIndex];
            UpdateVisualization();
            
            if (logPositionChanges)
            {
                var position = poseLoader.GetPosition(currentPositionId);
                Debug.Log($"Position {currentIndex + 1}/{positionIds.Length}: {position?.description}");
            }
        }
    }
    
    bool HasVisualizationChanged()
    {
        // Check if we need to update the visualization
        // This could be expanded to track parameter changes
        return false; // For now, manual updates only
    }
    
    void SetupVisualization()
    {
        ClearOldVisualization();
        
        switch (visualizationMode)
        {
            case VisualizationMode.YBotHumanoids:
                SetupYBots();
                break;
                
            case VisualizationMode.DebugMannequins:
                // Mannequins will be created per-position
                break;
                
            case VisualizationMode.Both:
                SetupYBots();
                break;
        }
    }
    
    void SetupYBots()
    {
        if (yBotPrefab == null)
        {
            Debug.LogError("Y Bot prefab not assigned!");
            return;
        }
        
        for (int player = 0; player < 2; player++)
        {
            // Instantiate Y Bot
            yBotInstances[player] = Instantiate(yBotPrefab, humanoidContainer);
            yBotInstances[player].name = $"Player{player + 1}_YBot";
            
            Vector3 position = Vector3.zero;
            yBotInstances[player].transform.localPosition = position;
            yBotInstances[player].transform.localScale = Vector3.one; // No scaling here
            
            // Apply materials
            Material playerMaterial = player == 0 ? player1Material : player2Material;
            if (playerMaterial != null)
            {
                ApplyMaterialToYBot(yBotInstances[player], playerMaterial);
            }
            
            // Set up retargeter
            retargeters[player] = yBotInstances[player].GetComponent<SimpleIKRetargeter>();
            if (retargeters[player] == null)
            {
                retargeters[player] = yBotInstances[player].AddComponent<SimpleIKRetargeter>();
                retargeters[player].playerIndex = player;
                retargeters[player].SetupIKSystem();
            }
        }
        
        Debug.Log("Y Bot humanoids set up successfully");
    }
    
    void ApplyMaterialToYBot(GameObject yBot, Material material)
    {
        var renderers = yBot.GetComponentsInChildren<Renderer>();
        foreach (var renderer in renderers)
        {
            var materials = new Material[renderer.materials.Length];
            for (int i = 0; i < materials.Length; i++)
            {
                materials[i] = material;
            }
            renderer.materials = materials;
        }
    }
    
    void ClearOldVisualization()
    {
        // Clear Y Bots
        for (int i = 0; i < yBotInstances.Length; i++)
        {
            if (yBotInstances[i] != null)
            {
                DestroyImmediate(yBotInstances[i]);
                yBotInstances[i] = null;
                retargeters[i] = null;
            }
        }
        
        // Clear debug mannequins
        ClearDebugMannequins();
    }
    
    void ClearDebugMannequins()
    {
        foreach (var joint in jointObjects.Values)
        {
            if (joint != null) DestroyImmediate(joint);
        }
        
        foreach (var bone in boneObjects.Values)
        {
            if (bone != null) DestroyImmediate(bone);
        }
        
        jointObjects.Clear();
        boneObjects.Clear();
    }
    
    public void UpdateVisualization()
    {
        if (poseLoader == null || string.IsNullOrEmpty(currentPositionId))
            return;
            
        var position = poseLoader.GetPosition(currentPositionId);
        if (position == null)
        {
            Debug.LogWarning($"Position {currentPositionId} not found!");
            return;
        }
        
        if (logPositionChanges)
        {
            Debug.Log($"Updating visualization to position: {position.description}");
        }
        
        // Update Y Bots
        if (visualizationMode == VisualizationMode.YBotHumanoids || 
            visualizationMode == VisualizationMode.Both)
        {
            UpdateYBots(position);
        }
        
        // Update debug mannequins
        if (visualizationMode == VisualizationMode.DebugMannequins || 
            visualizationMode == VisualizationMode.Both)
        {
            UpdateDebugMannequins(position);
        }
    }
    
    void UpdateYBots(GrappleMapPosition position)
    {
        for (int player = 0; player < 2; player++)
        {
            if (retargeters[player] != null)
            {
                var playerJoints = poseLoader.GetPlayerJoints(position, player);
                
                // Apply global scale to all joint positions
                for (int i = 0; i < playerJoints.Length; i++)
                {
                    playerJoints[i] *= globalScale;
                }
                
                retargeters[player].ApplyPose(playerJoints);
            }
        }
    }
    
    void UpdateDebugMannequins(GrappleMapPosition position)
    {
        ClearDebugMannequins();
        
        // Create joints
        for (int player = 0; player < 2; player++)
        {
            for (int joint = 0; joint < 23; joint++)
            {
                Vector3 jointPos = poseLoader.GetJointPosition(position, player, joint);
                
                // Apply the SAME scaling and separation as humanoids
                jointPos *= globalScale;
                
                CreateDebugJoint(player, joint, jointPos);
            }
        }
        
        // Create bones
        CreateDebugBones();
    }
    
    void CreateDebugJoint(int player, int joint, Vector3 position)
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.SetParent(mannequinContainer);
        sphere.transform.localPosition = position;
        
        float size = jointSizes[joint] * jointScale;
        sphere.transform.localScale = Vector3.one * size;
        
        sphere.name = $"P{player}_{jointNames[joint]}";
        
        // Apply material
        var renderer = sphere.GetComponent<Renderer>();
        Material mat = player == 0 ? mannequinPlayer1Material : mannequinPlayer2Material;
        if (mat != null)
        {
            renderer.material = mat;
        }
        else
        {
            renderer.material.color = player == 0 ? Color.red : Color.blue;
        }
        
        jointObjects[(player, joint)] = sphere;
        
        // Add label if enabled
        if (showJointLabels)
        {
            AddJointLabel(sphere, $"P{player}_{jointNames[joint]}");
        }
    }
    
    void CreateDebugBones()
    {
        for (int i = 0; i < boneConnections.Length; i += 2)
        {
            int joint1 = boneConnections[i];
            int joint2 = boneConnections[i + 1];
            
            for (int player = 0; player < 2; player++)
            {
                if (jointObjects.ContainsKey((player, joint1)) && 
                    jointObjects.ContainsKey((player, joint2)))
                {
                    CreateDebugBone(player, joint1, joint2);
                }
            }
        }
    }
    
    void CreateDebugBone(int player, int joint1, int joint2)
    {
        var joint1Obj = jointObjects[(player, joint1)];
        var joint2Obj = jointObjects[(player, joint2)];
        
        Vector3 pos1 = joint1Obj.transform.localPosition;
        Vector3 pos2 = joint2Obj.transform.localPosition;
        
        GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        cylinder.transform.SetParent(mannequinContainer);
        
        // Position and scale the bone
        cylinder.transform.localPosition = (pos1 + pos2) / 2;
        float distance = Vector3.Distance(pos1, pos2);
        cylinder.transform.localScale = new Vector3(boneRadius, distance / 2, boneRadius);
        
        // Orient the bone
        cylinder.transform.up = (pos2 - pos1).normalized;
        
        cylinder.name = $"P{player}_Bone_{jointNames[joint1]}_{jointNames[joint2]}";
        
        // Apply material
        var renderer = cylinder.GetComponent<Renderer>();
        Material mat = player == 0 ? mannequinPlayer1Material : mannequinPlayer2Material;
        if (mat != null)
        {
            renderer.material = mat;
        }
        else
        {
            renderer.material.color = player == 0 ? Color.red : Color.blue;
        }
        
        boneObjects[(player, joint1 * 100 + joint2)] = cylinder;
    }
    
    void AddJointLabel(GameObject joint, string label)
    {
        // Create a simple text mesh for joint labels
        var textGO = new GameObject($"{label}_Label");
        textGO.transform.SetParent(joint.transform);
        textGO.transform.localPosition = Vector3.up * 0.1f;
        
        var textMesh = textGO.AddComponent<TextMesh>();
        textMesh.text = label;
        textMesh.fontSize = 10;
        textMesh.color = Color.white;
        textMesh.anchor = TextAnchor.MiddleCenter;
        
        // Make text face camera
        textGO.transform.LookAt(Camera.main.transform);
        textGO.transform.Rotate(0, 180, 0);
    }
    
    // === PUBLIC API ===
    
    public void SetPosition(string positionId)
    {
        if (poseLoader.GetPosition(positionId) != null)
        {
            currentPositionId = positionId;
            UpdateVisualization();
        }
        else
        {
            Debug.LogWarning($"Position {positionId} not found!");
        }
    }
    
    public void SetRandomPosition()
    {
        var randomPos = poseLoader.GetRandomPosition();
        if (randomPos != null)
        {
            SetPosition(randomPos.id);
        }
    }
    
    public void SetRandomPositionWithTag(string tag)
    {
        var randomPos = poseLoader.GetRandomPositionWithTag(tag);
        if (randomPos != null)
        {
            SetPosition(randomPos.id);
        }
    }
    
    public void ToggleVisualizationMode()
    {
        visualizationMode = (VisualizationMode)(((int)visualizationMode + 1) % 3);
        SetupVisualization();
        UpdateVisualization();
    }
}