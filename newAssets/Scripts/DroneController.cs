using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Float32MultiArrayMsg

public class DroneController : MonoBehaviour
{
    ROSConnection ros;
    public string unifiedTopic = "/unity_cmd";

    [Header("ROS References")]
    public HapticPlugin haptic;
    public Transform drone;  // Only for reading rotation
    public DroneCommandSubscriber pixelThresholds;

    // Pixel thresholds for obstacle detection
    float maxThreshold = 130f;
    float minThreshold = 70f;

    [Header("Speeds (-100..100 for Tello)")]
    public float linearSpeed = 4f;
    public float yawSpeed = 5f;

    [Header("Tuning")]
    public float rotateGain = 2.0f;
    public float alignmentThreshold = 0.99f;

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
        if (drone == null || pixelThresholds == null) return;

        // Keyboard overrides (T/L for takeoff/land) - FIRST
        if (Input.GetKeyDown(KeyCode.T))
        {
            pendingTakeoff = true;
            pendingLand = false;
            Debug.Log("Takeoff requested (latched)");
        }
        if (Input.GetKeyDown(KeyCode.L))
        {
            pendingLand = true;
            pendingTakeoff = false;
            Debug.Log("Land requested (latched)");
        }

        // 1) Stylus ¡ú desired direction
        Vector3 rawInputDir = GetStylusPosition();
        float inputMagnitude = rawInputDir.magnitude;
        if (inputMagnitude <= 50f)
        {
            pendingTakeoff = false;
            pendingLand = false;
            PublishCommands(0f, 0f, 0f, 0f);
            return;
        }

        Vector3 inputDir = new Vector3(-rawInputDir.z, 0f, rawInputDir.x).normalized;
        Vector3 forwardDir = drone.forward.normalized;
        float dotProd = Vector3.Dot(inputDir, forwardDir);

        // 2) Compute commands
        float forward = 0f;
        float yaw = 0f;

        if (dotProd <= alignmentThreshold)
        {
            float targetYaw = Mathf.Atan2(inputDir.x, inputDir.z);
            float currentYaw = drone.eulerAngles.y * Mathf.Deg2Rad;
            float yawError = Mathf.DeltaAngle(drone.eulerAngles.y, targetYaw * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            yaw = Mathf.Clamp(yawError * rotateGain, -1f, 1f) * yawSpeed;
        }
        else
        {
            float frontPixels = pixelThresholds.msg.data[0];
            if (frontPixels < minThreshold)
            {
                forward = linearSpeed;
            }
            else
            {
                float lerp = (frontPixels - minThreshold) / (maxThreshold - minThreshold);
                forward = Mathf.Max(0f, linearSpeed * (1f - lerp));
            }
        }

        // 3) Publish at fixed rate
        if (Time.time - lastPublishTime >= publishInterval)
        {
            float takeoffFlag = pendingTakeoff ? 1f : 0f;
            float landFlag = pendingLand ? 1f : 0f;
            PublishCommands(forward, yaw, takeoffFlag, landFlag);
            lastPublishTime = Time.time;

            if (takeoffFlag > 0f) pendingTakeoff = false;
            if (landFlag > 0f) pendingLand = false;
        }
    }


    void PublishCommands(float forward, float yaw, float takeoffFlag, float landFlag)
    {
        var msg = new Float32MultiArrayMsg();
        msg.data = new float[] { forward, yaw, takeoffFlag, landFlag, 0f };
        Debug.Log(forward);
        ros.Publish(unifiedTopic, msg);
    }


    private Vector3 GetStylusPosition()
    {
        double[] position = new double[3];
        HapticPlugin.getPosition("Default Device", position);
        return new Vector3((float)position[0], (float)position[1], (float)position[2]);
    }
}




/*
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class DroneController : MonoBehaviour
{
    [Header("ROS Setup")]
    ROSConnection ros;
    public string cmdVelTopic = "/unity_cmd";

    public Transform drone;  // Only for reading rotation
    public DroneCommandSubscriber pixelThresholds;

    // Pixel thresholds for obstacle detection
    float maxThreshold = 130f;
    float minThreshold = 70f;

    [Header("Tello RC Commands (-100..100)")]
    public float maxRotateSpeed = 50f;  // angular.z 
    public float maxMoveSpeed = 40f;    // linear.x

    [Header("Tuning")]
    public float rotateGain = 2.0f;
    public float alignmentThreshold = 0.99f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
    }

    void Update()
    {
        if (drone == null || pixelThresholds == null) return;

        // 1) Read stylus ¡ú desired direction (XZ plane only)
        Vector3 rawInputDir = GetStylusPosition();
        float inputMagnitude = rawInputDir.magnitude;
        if (inputMagnitude <= 50f)
        {
            SendStop();  // No input ¡ú stop
            return;
        }

        Vector3 inputDir = new Vector3(-rawInputDir.z, 0f, rawInputDir.x).normalized;
        Vector3 forwardDir = drone.forward.normalized;
        float dotProd = Vector3.Dot(inputDir, forwardDir);

        // 2) Compute commands
        float linearX = 0f;
        float angularZ = 0f;

        if (dotProd <= alignmentThreshold)
        {
            // Rotate to face input direction
            float targetYaw = Mathf.Atan2(inputDir.x, inputDir.z);
            float currentYaw = drone.eulerAngles.y * Mathf.Deg2Rad;
            float yawError = Mathf.DeltaAngle(drone.eulerAngles.y, targetYaw * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            angularZ = Mathf.Clamp(yawError * rotateGain, -1f, 1f) * maxRotateSpeed;
        }
        else
        {
            // Aligned ¡ú check obstacles
            float frontPixels = pixelThresholds.msg.data[0];
            if (frontPixels < minThreshold)
            {
                linearX = maxMoveSpeed;  // Clear path
            }
            else
            {
                // Scale by obstacle proximity
                float lerp = (frontPixels - minThreshold) / (maxThreshold - minThreshold);
                linearX = Mathf.Max(0f, maxMoveSpeed * (1f - lerp));
            }
        }

        // 3) Send
        SendTwist(linearX, angularZ);
    }

    void SendTwist(float linearX, float angularZ)
    {
        TwistMsg twist = new TwistMsg
        {
            linear = new Vector3Msg(linearX, 0f, 0f),
            angular = new Vector3Msg(0f, 0f, angularZ)
        };
        ros.Publish(cmdVelTopic, twist);
    }

    void SendStop()
    {
        SendTwist(0f, 0f);
    }

    private Vector3 GetStylusPosition()
    {
        double[] position = new double[3];
        HapticPlugin.getPosition("Default Device", position);
        return new Vector3((float)position[0], (float)position[1], (float)position[2]);
    }
}

*/