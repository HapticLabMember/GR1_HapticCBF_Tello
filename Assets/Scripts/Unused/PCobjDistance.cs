using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PCobjDistance : MonoBehaviour
{

    public float maxDist = 100f;
    public LayerMask layerMask;
    public Transform drone;

    public float distance = 100f;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {

        if (drone == null)
        {
            Debug.LogWarning("Drone is not assigned!");
            return;
        }

        Debug.DrawRay(drone.position, Vector3.forward * maxDist, Color.red);

        //Ray ray = new Ray(drone.position, drone.forward);
        //RaycastHit hit;

        if(Physics.Raycast(drone.position, drone.forward, out RaycastHit hit, 50))
        {
            distance = hit.distance;
            Debug.Log("Distance in front: " + hit.distance);
            Debug.DrawRay(drone.position, drone.forward * maxDist, Color.red);

        }


    }

    void OnDrawGizmos()
    {
        if (drone != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(drone.position, drone.forward * maxDist);
        }
    }
}
