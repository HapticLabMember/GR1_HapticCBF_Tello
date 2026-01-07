using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class TakeoffTest : MonoBehaviour
{
    ROSConnection ros;

    string cmdVelTopic  = "/control";
    string takeoffTopic = "/takeoff";
    string landTopic    = "/land";

    public float linearSpeed = 1f;
    public float yawSpeed    = 2f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.RegisterPublisher<EmptyMsg>(takeoffTopic);
        ros.RegisterPublisher<EmptyMsg>(landTopic);

        Debug.Log($"Teleop started. cmdVel={cmdVelTopic}, takeoff={takeoffTopic}, land={landTopic}");
    }

    void Update()
    {
        HandleMotionKeys();
        HandleTakeoffLandKeys();
    }

    void HandleMotionKeys()
    {
        float x = 0f;
        float yaw = 0f;

        if (Input.GetKey(KeyCode.UpArrow))
            x = linearSpeed;
        if (Input.GetKey(KeyCode.LeftArrow))
            yaw = yawSpeed;
        if (Input.GetKey(KeyCode.RightArrow))
            yaw = -yawSpeed;

        TwistMsg twist = new TwistMsg
        {
            linear  = new Vector3Msg(x, 0f, 0f),
            angular = new Vector3Msg(0f, 0f, yaw)
        };

        ros.Publish(cmdVelTopic, twist);
    }

    void HandleTakeoffLandKeys()
    {
        if (Input.GetKeyDown(KeyCode.T))
        {
            Debug.Log($"Publishing takeoff on topic '{takeoffTopic}'");
            ros.Publish(takeoffTopic, new EmptyMsg());
        }

        if (Input.GetKeyDown(KeyCode.L))
        {
            Debug.Log($"Publishing land on topic '{landTopic}'");
            ros.Publish(landTopic, new EmptyMsg());
        }
    }
}
