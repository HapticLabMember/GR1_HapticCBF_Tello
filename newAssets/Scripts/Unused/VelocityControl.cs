using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;


public class VelocityControl : MonoBehaviour
{
    public Rigidbody rb;
    public string cmdVelTopic = "/cmd_vel_safe";
    

    public float moveSpeed = 1.0f;   
    public float yawSpeedDeg = 180f; 

    void Start()
    {
        
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(cmdVelTopic, HandleCmdVelReceived);
    }

    private void HandleCmdVelReceived(TwistMsg msg)
    {
       
        Vector3 rawVelocity = new Vector3(
            (float)msg.linear.x,
            (float)msg.linear.y,
            (float)msg.linear.z
        );
       
        rawVelocity *= moveSpeed;

        
        Vector3 finalVelocity = transform.TransformDirection(rawVelocity);
        rb.velocity = finalVelocity;

        float yawInput = (float)msg.angular.y; 
        
        float yawDelta = yawInput * yawSpeedDeg * Time.deltaTime;

     
        transform.Rotate(0, yawDelta, 0);

        Debug.Log($"[VelocityControl] rawVel={rawVelocity}, finalVel={finalVelocity}, yawDeg={yawDelta}");
    }
}
