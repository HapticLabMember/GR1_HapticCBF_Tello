using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DepthTesting : MonoBehaviour
{
    public Transform tello;
    public Transform obstacle;
    public float distance;
    public Vector3 distVec;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        distVec = obstacle.position - tello.position;
        distance = distVec.magnitude;

    }
}
