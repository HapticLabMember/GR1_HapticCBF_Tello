using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Float32MultiArrayMsg

public class UnifiedCommandSender : MonoBehaviour
{
    ROSConnection ros;
    public string unifiedTopic = "/unity_cmd";

    [Header("Speeds (-100..100 for Tello)")]
    public float linearSpeed = 2f;
    public float yawSpeed = 5f;

    [Header("Publish rate")]
    public float publishRateHz = 5f;
    float publishInterval;
    float lastPublishTime;

    // Latched command flags
    bool pendingTakeoff = false;
    bool pendingLand = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(unifiedTopic);

        publishInterval = 1f / publishRateHz;
        lastPublishTime = Time.time;
    }

    void Update()
    {
        // 1) Update control intents from input every frame

        float forward = 0f;
        float yaw = 0f;

        if (Input.GetKey(KeyCode.UpArrow))
            forward = linearSpeed;

        if (Input.GetKey(KeyCode.LeftArrow))
            yaw = yawSpeed;

        if (Input.GetKey(KeyCode.RightArrow))
            yaw = -yawSpeed;

        // Latch takeoff / land on key down
        if (Input.GetKeyDown(KeyCode.T))
        {
            pendingTakeoff = true;
            pendingLand = false; // optional, avoid conflicting
            Debug.Log("Takeoff requested (latched)");
        }

        if (Input.GetKeyDown(KeyCode.L))
        {
            pendingLand = true;
            pendingTakeoff = false; // optional
            Debug.Log("Land requested (latched)");
        }

        // 2) Publish at fixed rate
        if (Time.time - lastPublishTime >= publishInterval)
        {
            float takeoffFlag = pendingTakeoff ? 1f : 0f;
            float landFlag = pendingLand ? 1f : 0f;

            var msg = new Float32MultiArrayMsg();
            msg.data = new float[] { forward, yaw, takeoffFlag, landFlag, 0f };
            ros.Publish(unifiedTopic, msg);

            lastPublishTime = Time.time;

            // Optional: auto-clear after sending once
            // If you prefer ROS side to handle edge detection, comment these out.
            if (takeoffFlag > 0f) pendingTakeoff = false;
            if (landFlag > 0f)    pendingLand = false;
        }
    }
}
