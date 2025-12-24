using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CBFHapticNew : MonoBehaviour
{
    [SerializeField] public PlaceholderDroneMovement droneRotate;
    [SerializeField] public PlaceholderDistanceCalculator objDist;

    private Vector3 correctedVectorDifference = Vector3.zero;
    private Vector3 center = Vector3.zero;

    [SerializeField] private float forceMultiplier = 0.3f;

    private float moveSpeed;
    private float scale;
    
    void Awake()
    {

    }

    void Start()
    {
        moveSpeed = droneRotate.moveSpeed;
        scale = objDist.scale;
    }

    void Update()
    {

        //NOT using droneRotate.inputDir because the x and z coordinates fixed to math UNITY drone
        Vector3 stylusPosition = GetStylusPositionWithoutY();
        
        float distance = objDist.distance;
        bool objectInFront = objDist.objectInFront;


        /* Maybe set a max distance (1 meter?) and if the distance value drops below it
         * you can start inccreasing force magnitude like maxDistance - currentDist
         * That means the closer you get, the higher the value would become
         * This way, the closer you get to object, the more force is felt
         */


        //Drone does not move, only rotateable when near an object. Also force is applied in the
        //opposite direction (needs change so its constant and the force changes on distance)
         

        if (objectInFront)
        {
            //when the drone is near object, it applies force in opposite direction
            //replace the distance with the message read from ROS1 MiDaS
            //distance - minimumDistance (the closer it is to object, the less force felt??)
            // 1 - (distance - minimumDistance) <-- this means the closer you are to obstacle,
            // the larger the number becomes (multiplier)

            //ApplyHapticForce(-droneRotate.forwardDir * (1 - (minimumDistance - distance)));

            scale = objDist.scale;
            Vector3 correctedVectorDifference = (stylusPosition * objDist.scale) - stylusPosition;
            correctedVectorDifference = correctedVectorDifference.normalized;

            float correctedMagnitude = 1 - scale;
            //Debug.Log("Magnitude: " + correctedMagnitude);

            if (correctedMagnitude > 1f)
            {
                correctedMagnitude = 1f;
            }

            ApplyHapticForce(correctedVectorDifference, (double)(correctedMagnitude * forceMultiplier) );

        }
        else
        {
            ApplyHapticForce(center, 0d);
        }
        // CHECK HapticToROSController.cs to fix the message send to ROS
        // THAT or just copy and paste the message sending lines to PHDroneWithRotate
        //That has the input xyz values AND rotate degrees (yaw) so send that as message
        //if distance close, send only rotate message else, just xyz message

    }

    public Vector3 GetStylusPosition()
    {
        double[] position = new double[3];
        HapticPlugin.getPosition("Default Device", position);
        return new Vector3((float)position[0], (float)position[1], (float)position[2]);
    }

    public Vector3 GetStylusPositionWithoutY()
    {
        double[] position = new double[3];
        HapticPlugin.getPosition("Default Device", position);
        return new Vector3((float)position[0], 0f, (float)position[2]);
    }

    void ApplyHapticForce(Vector3 force, double magnitude)
    {
        double[] forceArray = { force.x, force.y, force.z };
        //HapticPlugin.setForce("Default Device", forceArray, new double[3]);

        //for anchor, maybe get ( localDronePos.toGlobalPos - stylusPos )
        //HapticPlugin.setSpringValues("Default Device", double[] anchor, double magnitude);

        //direction works like now 
        HapticPlugin.setConstantForceValues("Default Device", forceArray, magnitude);
    }
}
