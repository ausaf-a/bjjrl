using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using System.Linq;

[System.Serializable]
public class GrappleMapPosition
{
    public string id;
    public float[][] joints; // [46][3] - 46 joints, 3 coordinates each
    public string description;
    public string[] tags;
    public string code;
}

[System.Serializable]
public class GrappleMapTransition
{
    public string id;
    public string from;
    public string to;
    public string description;
    public string properties;
    public float[][][] sequence; // [keyframes][46][3]
    public int keyframes;
    public string reverse_transition_id;
}

[System.Serializable]
public class GrappleMapAdjacency
{
    public string[] outgoing;
    public string[] incoming;
}

[System.Serializable]
public class GrappleMapData
{
    public Dictionary<string, GrappleMapPosition> positions;
    public Dictionary<string, GrappleMapTransition> transitions;
    public Dictionary<string, GrappleMapAdjacency> adjacency;
}

public class EnhancedPoseLoader : MonoBehaviour
{
    [Header("Data Source")]
    public string jsonFileName = "grapplemap_processed";
    
    [Header("Debug")]
    public bool showDebugInfo = true;
    
    // Loaded data
    public GrappleMapData grappleMapData { get; private set; }
    
    // Quick access collections
    public Dictionary<string, GrappleMapPosition> positions { get; private set; }
    public Dictionary<string, GrappleMapTransition> transitions { get; private set; }
    public Dictionary<string, GrappleMapAdjacency> adjacency { get; private set; }
    
    // Statistics
    public int totalPositions { get; private set; }
    public int totalTransitions { get; private set; }
    public int connectedPositions { get; private set; }
    
    void Awake()
    {
        LoadGrappleMapData();
    }
    
    void LoadGrappleMapData()
    {
        try
        {
            Debug.Log($"Loading GrappleMap data from {jsonFileName}...");
            
            TextAsset jsonFile = Resources.Load<TextAsset>(jsonFileName);
            if (jsonFile == null)
            {
                Debug.LogError($"Could not find {jsonFileName}.json in Resources folder!");
                return;
            }
            
            grappleMapData = JsonConvert.DeserializeObject<GrappleMapData>(jsonFile.text);
            
            if (grappleMapData == null)
            {
                Debug.LogError("Failed to deserialize GrappleMap data!");
                return;
            }
            
            // Cache references for easy access
            positions = grappleMapData.positions;
            transitions = grappleMapData.transitions;
            adjacency = grappleMapData.adjacency;
            
            // Calculate statistics
            totalPositions = positions?.Count ?? 0;
            totalTransitions = transitions?.Count ?? 0;
            connectedPositions = adjacency?.Count ?? 0;
            
            if (showDebugInfo)
            {
                LogDataStatistics();
                LogSampleData();
            }
            
            Debug.Log($"✓ GrappleMap data loaded successfully!");
            
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error loading GrappleMap data: {e.Message}");
        }
    }
    
    void LogDataStatistics()
    {
        Debug.Log($"=== GRAPPLEMAP DATA STATISTICS ===");
        Debug.Log($"Positions: {totalPositions}");
        Debug.Log($"Transitions: {totalTransitions}");
        Debug.Log($"Connected Positions: {connectedPositions}");
        Debug.Log($"Average transitions per position: {(float)totalTransitions / totalPositions:F2}");
        
        // Find positions with most connections
        if (adjacency != null && adjacency.Count > 0)
        {
            var mostConnected = adjacency
                .OrderByDescending(kvp => kvp.Value.outgoing.Length + kvp.Value.incoming.Length)
                .Take(3);
            
            Debug.Log("Most connected positions:");
            foreach (var pos in mostConnected)
            {
                int totalConnections = pos.Value.outgoing.Length + pos.Value.incoming.Length;
                string desc = positions.ContainsKey(pos.Key) ? positions[pos.Key].description : "Unknown";
                Debug.Log($"  {pos.Key}: {totalConnections} connections - {desc}");
            }
        }
    }
    
    void LogSampleData()
    {
        Debug.Log($"=== SAMPLE DATA ===");
        
        // Show sample position
        if (positions != null && positions.Count > 0)
        {
            var samplePos = positions.First();
            Debug.Log($"Sample Position: {samplePos.Key}");
            Debug.Log($"  Description: {samplePos.Value.description}");
            Debug.Log($"  Tags: [{string.Join(", ", samplePos.Value.tags)}]");
            Debug.Log($"  Joints: {samplePos.Value.joints.Length} (first joint: [{string.Join(", ", samplePos.Value.joints[0])}])");
        }
        
        // Show sample transition
        if (transitions != null && transitions.Count > 0)
        {
            var sampleTrans = transitions.First();
            Debug.Log($"Sample Transition: {sampleTrans.Key}");
            Debug.Log($"  Description: {sampleTrans.Value.description}");
            Debug.Log($"  From: {sampleTrans.Value.from} → To: {sampleTrans.Value.to}");
            Debug.Log($"  Keyframes: {sampleTrans.Value.keyframes}");
            Debug.Log($"  Reversible: {!string.IsNullOrEmpty(sampleTrans.Value.reverse_transition_id)}");
        }
    }
    
    // === PUBLIC API METHODS ===
    
    /// <summary>
    /// Get a position by ID
    /// </summary>
    public GrappleMapPosition GetPosition(string positionId)
    {
        return positions?.ContainsKey(positionId) == true ? positions[positionId] : null;
    }
    
    /// <summary>
    /// Get a transition by ID
    /// </summary>
    public GrappleMapTransition GetTransition(string transitionId)
    {
        return transitions?.ContainsKey(transitionId) == true ? transitions[transitionId] : null;
    }
    
    /// <summary>
    /// Get all available transitions from a position
    /// </summary>
    public List<GrappleMapTransition> GetAvailableTransitions(string positionId)
    {
        var result = new List<GrappleMapTransition>();
        
        if (adjacency?.ContainsKey(positionId) == true)
        {
            foreach (string transId in adjacency[positionId].outgoing)
            {
                var transition = GetTransition(transId);
                if (transition != null)
                    result.Add(transition);
            }
        }
        
        return result;
    }
    
    /// <summary>
    /// Find positions by tag
    /// </summary>
    public List<GrappleMapPosition> FindPositionsByTag(string tag)
    {
        var result = new List<GrappleMapPosition>();
        
        if (positions != null)
        {
            foreach (var pos in positions.Values)
            {
                if (pos.tags != null && pos.tags.Contains(tag))
                    result.Add(pos);
            }
        }
        
        return result;
    }
    
    /// <summary>
    /// Find positions by description (partial match)
    /// </summary>
    public List<GrappleMapPosition> FindPositionsByDescription(string searchTerm)
    {
        var result = new List<GrappleMapPosition>();
        
        if (positions != null)
        {
            foreach (var pos in positions.Values)
            {
                if (!string.IsNullOrEmpty(pos.description) && 
                    pos.description.ToLower().Contains(searchTerm.ToLower()))
                    result.Add(pos);
            }
        }
        
        return result;
    }
    
    /// <summary>
    /// Get joint position for specific player and joint index
    /// </summary>
    public Vector3 GetJointPosition(GrappleMapPosition position, int player, int jointIndex)
    {
        if (position?.joints == null) return Vector3.zero;
        
        int index = player * 23 + jointIndex;
        if (index >= 0 && index < position.joints.Length && position.joints[index] != null && position.joints[index].Length >= 3)
        {
            return new Vector3(
                position.joints[index][0],
                position.joints[index][1], 
                position.joints[index][2]
            );
        }
        
        return Vector3.zero;
    }
    
    /// <summary>
    /// Get all joint positions for a specific player
    /// </summary>
    public Vector3[] GetPlayerJoints(GrappleMapPosition position, int player)
    {
        var joints = new Vector3[23];
        
        for (int i = 0; i < 23; i++)
        {
            joints[i] = GetJointPosition(position, player, i);
        }
        
        return joints;
    }
    
    /// <summary>
    /// Check if transition is bidirectional
    /// </summary>
    public bool IsTransitionBidirectional(string transitionId)
    {
        var transition = GetTransition(transitionId);
        return transition != null && !string.IsNullOrEmpty(transition.reverse_transition_id);
    }
    
    /// <summary>
    /// Get random position (useful for testing)
    /// </summary>
    public GrappleMapPosition GetRandomPosition()
    {
        if (positions == null || positions.Count == 0) return null;
        
        var keys = positions.Keys.ToArray();
        string randomKey = keys[Random.Range(0, keys.Length)];
        return positions[randomKey];
    }
    
    /// <summary>
    /// Get random position with specific tag
    /// </summary>
    public GrappleMapPosition GetRandomPositionWithTag(string tag)
    {
        var taggedPositions = FindPositionsByTag(tag);
        return taggedPositions.Count > 0 ? taggedPositions[Random.Range(0, taggedPositions.Count)] : null;
    }
}