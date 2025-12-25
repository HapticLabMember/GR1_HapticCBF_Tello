using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Float32MultiArrayMsg

public class DroneController : MonoBehaviour
{
    [SerializeField] private float objDist = -0f;

    ROSConnection ros;
    public string unifiedTopic = "/unity_cmd";

    [Header("ROS References")]
    public HapticPlugin haptic;
    public Transform drone;  // Only for reading rotation
    public DroneCommandSubscriber pixelThresholds;

    // Pixel thresholds for obstacle detection
    float maxThreshold = 130f;
    float minThreshold = 10f;

    [Header("Inputs")]
    public float rawForward = 0f;
    public float forward = 0f;
    public float yawSpeed = 60f;
    public float maxVelocity = 20f;
    private float predictedRealDroneYawDiff = 0.7f;

    [Header("Tuning")]
    public float rotateGain = 1.5f;
    public float alignmentThreshold = 0.95f;

    [Header("Publish rate")]
    public float publishRateHz = 1f;
    float publishInterval;
    float lastPublishTime;

    [Header("Startup Delay")]
    public float startupDelay = 2f;  // Ignore inputs for first 2 seconds
    float startupEndTime;

    // Latched command flags
    bool pendingTakeoff = false;
    bool pendingLand = false;

    public Vector3 rawInputDir;
    public Vector3 droneDir;

    // Drone rotation sync
    float accumulatedYaw = 0f;  // Track total rotation sent to ROS

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(unifiedTopic);

        publishInterval = 1f / publishRateHz;
        lastPublishTime = Time.time;

        // Initialize accumulatedYaw from current drone rotation
        if (drone != null)
            accumulatedYaw = drone.eulerAngles.y * Mathf.Deg2Rad;
    }

    void Update()
    {
        rawForward = 0f;
        forward = 0f;
        // Blocks inputs for the first 2 seconds (to prevent accidental takeoffs)
        if (Time.time < startupEndTime)
        {
            // Still send zero commands during startup to keep ROS happy
            if (Time.time - lastPublishTime >= publishInterval)
            {
                pendingTakeoff = false;
                pendingLand = false;
                lastPublishTime = Time.time;
            }
            return;
        }

        if (drone == null || pixelThresholds == null) return;

        // Keyboard overrides (T/L for takeoff/land)
        if (Input.GetKeyDown(KeyCode.T))
        {
            pendingTakeoff = true;
            pendingLand = false;
            Debug.Log("Takeoff requested (T button) (latched)");
        }
        if (Input.GetKeyDown(KeyCode.L))
        {
            pendingLand = true;
            pendingTakeoff = false;
            Debug.Log("Land requested (L button) (latched)");
        }

        // 1) Stylus ¡ú desired direction
        rawInputDir = GetStylusPositionWithoutY();
        float inputMagnitude = rawInputDir.magnitude;

        // if magnitude is small, dont move drone (other inputs valids)
        if (inputMagnitude <= 50f)
        {
            if (Time.time - lastPublishTime >= publishInterval)
            {
                float takeoffFlag = pendingTakeoff ? 1f : 0f;
                float landFlag = pendingLand ? 1f : 0f;
                PublishCommands(0f, 0f, takeoffFlag, landFlag);
                lastPublishTime = Time.time;

                if (takeoffFlag > 0f) pendingTakeoff = false;
                if (landFlag > 0f) pendingLand = false;
            }
            return;
        }

        Vector3 inputDir = rawInputDir.normalized;
        Vector3 forwardDir = drone.forward.normalized;
        droneDir = forwardDir;
        float dotProd = Vector3.Dot(inputDir, forwardDir);


        // 2) Compute commands
        float yaw = 0f;
        // rotate if input direction ISNT the same as the drone direction
        if (dotProd <= alignmentThreshold)
        {
            float targetYaw = Mathf.Atan2(inputDir.x, inputDir.z);
            float currentYaw = drone.eulerAngles.y * Mathf.Deg2Rad;
            float yawError = Mathf.DeltaAngle(drone.eulerAngles.y, targetYaw * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            yaw = Mathf.Clamp(yawError * rotateGain, -1f, 1f) * yawSpeed;
        }
        else
        {
            rawForward = inputMagnitude / 10f;
            if (rawForward > maxVelocity) rawForward = maxVelocity;

            float frontPixels = objDist;
            if(pixelThresholds.msg.data.Length > 0)
            {
                frontPixels = pixelThresholds.msg.data[0];
            }

            if (frontPixels < minThreshold)
            {
                // REPLACE LINEAR SPEED WIT HTHE INPUT DIRECTION MAGNITIDE OR SOMETHING
                forward = rawForward;
            }
            else
            {
                float lerp = (frontPixels - minThreshold) / (maxThreshold - minThreshold);
                Debug.Log(frontPixels + " : " + lerp);

                forward = Mathf.Max(0f, rawForward * (1f - lerp));
            }
        }

        // 3) Publish at fixed rate + Unity drone rotation
        if (Time.time - lastPublishTime >= publishInterval)
        {
            float takeoffFlag = pendingTakeoff ? 1f : 0f;
            float landFlag = pendingLand ? 1f : 0f;

            PublishCommands(forward, yaw, takeoffFlag, landFlag);

            // Synchronize the unity drone with the yaw value of sent message
            ApplyYawToDrone(yaw);  // Convert yaw rate to angle

            lastPublishTime = Time.time;
            if (takeoffFlag > 0f) pendingTakeoff = false;
            if (landFlag > 0f) pendingLand = false;
        }
    }

    void PublishCommands(float forward, float yaw, float takeoffFlag, float landFlag)
    {
        var msg = new Float32MultiArrayMsg();
        msg.data = new float[] { forward, yaw, takeoffFlag, landFlag, 0f };
        Debug.Log(forward + " : " + yaw + " : " + takeoffFlag + " : " + landFlag);
        ros.Publish(unifiedTopic, msg);
    }

    void ApplyYawToDrone(float yawDeltaDegrees)
    {
        // Apply exact same rotation as sent to ROS (in degrees)
        float newYaw = drone.eulerAngles.y + yawDeltaDegrees * predictedRealDroneYawDiff;
        drone.rotation = Quaternion.Euler(0f, newYaw, 0f);
    }

    private Vector3 GetStylusPosition()
    {
        double[] position = new double[3];
        HapticPlugin.getPosition("Default Device", position);
        return new Vector3((float)position[0], (float)position[1], (float)position[2]);
    }

    public Vector3 GetStylusPositionWithoutY()
    {
        double[] position = new double[3];
        HapticPlugin.getPosition("Default Device", position);
        return new Vector3((float)position[0], 0f, (float)position[2]);
    }

    public void OnClickButton1()
    {
        pendingTakeoff = true;
        pendingLand = false;
        Debug.Log("Takeoff requested (Button1) (latched)");
    }

    public void OnClickButton2()
    {
        pendingLand = true;
        pendingTakeoff = false;
        Debug.Log("Land requested (Button2) (latched)");
    }
}
