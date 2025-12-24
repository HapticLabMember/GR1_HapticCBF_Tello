using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; // TwistMsg, Vector3Msg
using RosMessageTypes.Std;      // EmptyMsg

public class PlaceholderDroneController : MonoBehaviour
{
    ROSConnection ros;
    [SerializeField] DroneCommandSubscriber obstacleDistances;

    // Topic names much match the drone topic names
    [Header("ROS topics")]
    public string cmdVelTopic = "/control";   // geometry_msgs/Twist
    public string takeoffTopic = "/takeoff";  // std_msgs/Empty
    public string landTopic = "/land";        // std_msgs/Empty

    [Header("Speeds")]
    public float linearSpeed = 2f;   // forward
    public float angularSpeed = 5f;  // yaw

    [Header("Obstacle Intensities (updated externally)")]
    public float obstaclePixelIntensity = 0f;
    public float leftWallIntensity = 0f;
    public float rightWallIntensity = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.RegisterPublisher<EmptyMsg>(takeoffTopic);
        ros.RegisterPublisher<EmptyMsg>(landTopic);
    }

    void Update()
    {
        HandleMotionKeys();
        HandleTakeoffLandKeys();
    }

    void HandleMotionKeys()
    {
        float linearX = 0f;
        float yaw = 0f;

        if (Input.GetKey(KeyCode.UpArrow)){
            if(obstacleDistances.msg.data[0] < 120.0f)
            {
                linearX = linearSpeed;
                Debug.Log("Forward Pressed");
            }
        }

        if (Input.GetKey(KeyCode.LeftArrow)) {
            if(obstacleDistances.msg.data[1] < 120.0f)
            {
                yaw = angularSpeed;
                Debug.Log("Left Pressed");
            }
        }

        if (Input.GetKey(KeyCode.RightArrow)) {
            if(obstacleDistances.msg.data[2] < 120.0f)
            {
                yaw = -angularSpeed;
                Debug.Log("Right Pressed");
            }
        }

        TwistMsg twist = new TwistMsg
        {
            linear = new Vector3Msg(linearX, 0f, 0f),
            angular = new Vector3Msg(0f, 0f, yaw)
        };

        //ros.Publish(cmdVelTopic, twist);
    }

    void HandleTakeoffLandKeys()
    {
        if (Input.GetKeyDown(KeyCode.T))
        {
            ros.Publish(takeoffTopic, new EmptyMsg());
            Debug.Log("Sent /takeoff");
        }

        if (Input.GetKeyDown(KeyCode.L))
        {
            ros.Publish(landTopic, new EmptyMsg());
            Debug.Log("Sent /land");
        }
    }
}
