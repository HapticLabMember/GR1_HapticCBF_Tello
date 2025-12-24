using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PHobjDistance : MonoBehaviour
{

    [SerializeField] public float maxDist = 10f;
    [SerializeField] public float minDist = 0.2f;
    [SerializeField] public float safeDist = 2.0f;
    public LayerMask layerMask;
    public Transform drone;
    public Transform droneCameraTransform;

    public float distance = 3f;
    public bool objectInFront = false;

    public float scale = 1f; //scale for velocity

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {

        Ray ray = new Ray(droneCameraTransform.position, droneCameraTransform.forward);
        RaycastHit hit;

        if(Physics.Raycast(ray, out hit, maxDist))
        {
            distance = hit.distance;
            Debug.DrawRay(droneCameraTransform.position, droneCameraTransform.forward * maxDist, Color.red);

        }
        else
        {
            distance = maxDist;
            Debug.DrawRay(droneCameraTransform.position, droneCameraTransform.forward * maxDist, Color.cyan);
        }
        //if object is near the drone, set the bool to true
        if(distance < minDist)
        {
            objectInFront = true;
        }
        else
        {
            objectInFront = false;
        }


        scale = Mathf.InverseLerp(minDist, safeDist, distance);

        Debug.Log("Distance: " + distance);
        Debug.Log("Scale: " + scale);


    }

}
