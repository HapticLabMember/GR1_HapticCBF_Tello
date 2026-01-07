using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;


public class ObstacleDetector : MonoBehaviour
{
    public Transform drone;          
    public float detectionRange = 5.0f;
    public string obstacleTopic = "/obstacle_data";

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointMsg>(obstacleTopic);
    }

    void Update()
    {

        Vector3 obstaclePos;
        if (DetectObstacle(out obstaclePos))
        {
            
            PointMsg msg = new PointMsg(obstaclePos.x, obstaclePos.y, obstaclePos.z);
            ros.Publish(obstacleTopic, msg);

            Debug.Log("Obstacle detected at " + obstaclePos);
        }
    }

    bool DetectObstacle(out Vector3 hitPos)
    {

        Vector3 forwardDir = transform.right;

        RaycastHit hit;
        if (Physics.Raycast(drone.position, -forwardDir, out hit, detectionRange))
        {
            
            if (hit.collider.CompareTag("Obstacle"))
            {
                hitPos = hit.point;
                return true;
            }
        }
        hitPos = Vector3.zero;
        return false;
    }
}
