using UnityEngine;

public class FlyCameraController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 5.0f;
    public float fastMoveSpeed = 10.0f;
    public float mouseSensitivity = 2.0f;
    
    [Header("Input Settings")]
    public KeyCode fastMoveKey = KeyCode.LeftShift;
    public KeyCode upKey = KeyCode.E;
    public KeyCode downKey = KeyCode.Q;
    
    [Header("Constraints")]
    public bool constrainPitch = true;
    public float minPitch = -90f;
    public float maxPitch = 90f;
    
    private float pitch = 0f;
    private float yaw = 0f;
    private bool isMouseLocked = false;
    
    void Start()
    {
        // Initialize rotation based on current transform
        Vector3 eulerAngles = transform.eulerAngles;
        yaw = eulerAngles.y;
        pitch = eulerAngles.x;
        
        // Handle pitch wraparound
        if (pitch > 180f)
            pitch -= 360f;
    }
    
    void Update()
    {
        HandleMouseLook();
        HandleMovement();
        HandleMouseLockToggle();
    }
    
    void HandleMouseLockToggle()
    {
        // Toggle mouse lock with right mouse button
        if (Input.GetMouseButtonDown(1))
        {
            ToggleMouseLock();
        }
        
        // Also unlock with Escape
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            UnlockMouse();
        }
    }
    
    void ToggleMouseLock()
    {
        if (isMouseLocked)
        {
            UnlockMouse();
        }
        else
        {
            LockMouse();
        }
    }
    
    void LockMouse()
    {
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
        isMouseLocked = true;
    }
    
    void UnlockMouse()
    {
        Cursor.lockState = CursorLockMode.None;
        Cursor.visible = true;
        isMouseLocked = false;
    }
    
    void HandleMouseLook()
    {
        if (!isMouseLocked) return;
        
        // Get mouse input
        float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity;
        float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity;
        
        // Update yaw and pitch
        yaw += mouseX;
        pitch -= mouseY;
        
        // Constrain pitch if enabled
        if (constrainPitch)
        {
            pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
        }
        
        // Apply rotation
        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
    }
    
    void HandleMovement()
    {
        // Determine current move speed
        float currentSpeed = Input.GetKey(fastMoveKey) ? fastMoveSpeed : moveSpeed;
        
        // Get input
        float horizontal = Input.GetAxis("Horizontal"); // A/D
        float vertical = Input.GetAxis("Vertical");     // W/S
        float upDown = 0f;
        
        // Handle up/down movement
        if (Input.GetKey(upKey))
            upDown = 1f;
        else if (Input.GetKey(downKey))
            upDown = -1f;
        
        // Calculate movement vector in local space
        Vector3 movement = new Vector3(horizontal, upDown, vertical);
        
        // Apply speed and time
        movement *= currentSpeed * Time.deltaTime;
        
        // Transform to world space and apply
        transform.Translate(movement);
    }
    
    void OnGUI()
    {
        if (!isMouseLocked)
        {
            // Show instructions when mouse is not locked
            GUIStyle style = new GUIStyle();
            style.normal.textColor = Color.white;
            style.fontSize = 14;
            
            GUI.Label(new Rect(10, 10, 300, 100), 
                "Right-click to enable mouse look\n" +
                "WASD: Move\n" +
                "E/Q: Up/Down\n" +
                "Shift: Fast move\n" +
                "Escape: Unlock mouse", style);
        }
    }
}