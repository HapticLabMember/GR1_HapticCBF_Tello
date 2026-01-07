using UnityEngine;

public class PHdroneWithRotate : MonoBehaviour
{
    public Rigidbody dronePlaceholder; // Assign in Inspector
    public Transform droneForward;
    public PlaceholderDistanceCalculator objDistance;

    public float forceMultiplier = 10f;
    public float rotateSpeed = 0.1f;
    public float moveSpeed = 0.5f;
    private Vector3 noSpeed = Vector3.zero;

    public Vector3 forwardDir;
    public Vector3 inputDir;


    void Start()
    {
    }

    void Update()
    {
        if (dronePlaceholder == null)
            return;

        Vector3 rawInputDir = GetStylusPosition();
        float inputMagnitude = rawInputDir.magnitude;

        //change y=0f  because it is not related to the facing direction of drone
        inputDir = new Vector3(-(float)rawInputDir[2], 0f, (float)rawInputDir[0]).normalized;
        forwardDir = droneForward.forward.normalized;

        float dotProd = Vector3.Dot(inputDir, forwardDir);
        //if the direction is incorrect, the drone stops moving forward until the direction is corrected
        if (dotProd <= 0.99f)
        {
            //send message to drone to set speed to 0
            dronePlaceholder.velocity = noSpeed;
            //only rotate around y axis (because its drone)
            float targetYaw = Mathf.Atan2(inputDir.x, inputDir.z) * Mathf.Rad2Deg;
            float currentYaw = dronePlaceholder.transform.eulerAngles.y;

            float newYaw = Mathf.MoveTowardsAngle(currentYaw, targetYaw, 10f*Time.deltaTime*rotateSpeed);
            dronePlaceholder.rotation = Quaternion.Euler(0f, newYaw, 0f);
        }
        else if (!objDistance.objectInFront && inputMagnitude > 50f)
        {
            dronePlaceholder.velocity = inputDir*moveSpeed;
        }
        else if (objDistance.objectInFront)
        {
            //to stop drone from moving when direction is RIGHT but object in front range
            dronePlaceholder.velocity = inputDir * moveSpeed * objDistance.scale;
        }
        else
        {
            //for when input magnitude small
            dronePlaceholder.velocity = noSpeed;
        }


        //======================= Note to Self =========================

        /* THIS CODE SHOULD BE REPLACED WITH ROS MESSAGES SENT TO DRONE
         * 
         * messages for rotation/movement should be sent with ROS
         * AFTER that, changes should be applied to unity viirtual drone
         * 
         * If unity and real drone desynchronize, it should auto fix OR
         * a button should synchronize drone ()
         *  set inputDir to 000 and make it rotate to x direction in REAL and virtual drone
        */

        // ======================= END NOTE ==============================
    }



    private Vector3 GetStylusPosition()
    {
        double[] position = new double[3];
        HapticPlugin.getPosition("Default Device", position);
        return new Vector3((float)position[0], (float)position[1], (float)position[2]);
    }
}
