using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaceholderDistanceCalculator : MonoBehaviour
{

    [SerializeField] public float maxDist = 30f;
    [SerializeField] public float minDist = 2f;
    [SerializeField] public float safeDist = 4f;
    public LayerMask layerMask;
    public Transform drone;
    public Transform droneFrontCamera; //to see object distance in front

    public float distance = 30f;
    public bool objectInFront = false;

    public float scale = 1f; //scale for velocity

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        //get distance of closest object in front of drone
        distance = GetClosestObstacleDistance();

        //if object is near the drone, set the bool to true
        if (distance < safeDist)
        {
            objectInFront = true;
        }
        else
        {
            objectInFront = false;
        }
        scale = Mathf.InverseLerp(minDist, safeDist, distance);
    }


    float GetClosestObstacleDistance(int rayCount = 7, float raySpread = 60f)
    {
        float closestDistance = maxDist;
        float startAngle = -raySpread / 2f;

        for (int i = 0; i < rayCount; i++)
        {
            // Calculate current ray angle relative to forward
            float angle = startAngle + (raySpread / (rayCount - 1)) * i;

            // Calculate direction by rotating forward vector around Y axis
            Vector3 rayDirection = Quaternion.Euler(0f, angle, 0f) * droneFrontCamera.forward;

            if (Physics.Raycast(droneFrontCamera.position, rayDirection, out RaycastHit hit, maxDist))
            {
                if (hit.distance < closestDistance)
                {
                    closestDistance = hit.distance;
                }
                Debug.DrawRay(droneFrontCamera.position, rayDirection * hit.distance, Color.red);
            }
            else
            {
                Debug.DrawRay(droneFrontCamera.position, rayDirection * maxDist, Color.cyan);
            }
        }

        return closestDistance;
    }

}
