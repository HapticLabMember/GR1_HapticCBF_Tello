using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CBFHapticNew : MonoBehaviour
{

    [SerializeField] public DroneController droneInputs;

    [SerializeField] private double forceMultiplier = 0.3;
    Vector3 center = new Vector3( 0f, 0f, 0f );


    void Start()
    {

    }

    void Update()
    {

        //NOT using droneRotate.inputDir because the x and z coordinates fixed to math UNITY drone
        Vector3 droneDir = droneInputs.droneDir;

        float rawInput = droneInputs.rawForward;
        float input = droneInputs.forward;


         

        if (rawInput - input > 0f)
        {
            // when input = 0f, full force of 1N
            // when input -> rawInput, force would approach 0f
            ApplyHapticForce(-droneDir, (double)((rawInput-input) / rawInput) * forceMultiplier );
            Debug.Log("Applied force: " + ((rawInput - input) / rawInput) * forceMultiplier);

        }
        else
        {
            ApplyHapticForce(center, 0d);
        }


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
