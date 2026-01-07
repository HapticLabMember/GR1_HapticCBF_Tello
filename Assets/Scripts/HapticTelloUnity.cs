using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class HapticTelloUnity : MonoBehaviour
{
    public Transform stylus;
    private Vector3 velocity = Vector3.zero;
    private float lastSendTime = 0f;

    private ROSConnection ros;
    public string twistTopic = "haptic_input";
    public float scaleLin = 200f;
    public float scaleAng = 120f;
    public float smoothFactor = 10f;
    public float sendRateHz = 30f;
    public float deadZone = 0.002f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(twistTopic);
        if (stylus == null)
        {
            Debug.LogError("Stylus transform is not assigned!");
            enabled = false;
        }
    }

    void Update()
    {
        if (Time.time - lastSendTime < 1f / sendRateHz) return;
        lastSendTime = Time.time;

        Vector3 delta = stylus.localPosition;
        if (delta.magnitude < deadZone) delta = Vector3.zero;

        velocity = Vector3.Lerp(velocity, delta, 0.3f);

        // —— 坐标系转换（RUF → FLU） ——  
        // Unity: x=右, y=上, z=前
        // ROS/Tello: x=前, y=左, z=上
        float ros_x = -velocity.x;
        float ros_y = -velocity.z;
        float ros_z = velocity.y;

        float dz = Mathf.Clamp(ros_x * scaleLin, -100f, 100f);  // 正前 = 正
        float dx = Mathf.Clamp(ros_y * scaleLin, -100f, 100f);  // 右移 -> ROS 左（-）
        float dy = Mathf.Clamp(ros_z * scaleLin, -100f, 100f);
        float wz = Mathf.Clamp(velocity.x * scaleAng, -100f, 100f); // yaw 根据需要反向

        double unityTime = Time.realtimeSinceStartup;
        var twist = new TwistMsg(
            new Vector3Msg(dz, dx, dy),
            new Vector3Msg(0, (float)unityTime, wz)
        );
        ros.Publish(twistTopic, twist);

        Debug.Log($"Twist → dz:{dz:F1} dx:{dx:F1} dy:{dy:F1} wz:{wz:F1} ts:{unityTime:F3}");
    }
}