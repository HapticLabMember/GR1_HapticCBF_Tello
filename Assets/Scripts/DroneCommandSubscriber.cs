using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class DroneCommandSubscriber : MonoBehaviour
{
    [SerializeField] string topicName = "/haptic/obstacle_averages";  // Replace with your topic
    public Float32MultiArrayMsg msg;

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<Float32MultiArrayMsg>(topicName, OnMessageReceived);
    }

    void OnMessageReceived(Float32MultiArrayMsg _msg)
    {
        msg = _msg;
        //Debug.Log($"[ROS] Received message: {msg.width}x{msg.height}, encoding={msg.encoding}, frame_id={msg.header.frame_id}");
        //Debug.Log($"[ROS] Obstacle Distance: {msg.data[0]}, Left Wall: {msg.data[1]}, Right Wall: {msg.data[2]}");
    }
}
