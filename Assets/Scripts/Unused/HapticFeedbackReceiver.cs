using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using System.Runtime.InteropServices;
using System;


public class HapticFeedbackReceiver : MonoBehaviour
{
    [DllImport("HapticsDirect")]
    private static extern int setSpringValues(string configName, double[] anchor, double magnitude);

    public string deviceName = "Default Device";
    public float maxForce = 0.5f;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<Vector3Msg>("/haptic_feedback", OnFeedbackReceived);
    }

    private void OnFeedbackReceived(Vector3Msg msg)
    {
        double fx = msg.x;
        double fy = msg.y;
        double fz = msg.z;
        double mag = Math.Sqrt(fx * fx + fy * fy + fz * fz);
        if (mag > maxForce)
        {
            double scale = maxForce / mag;
            fx *= scale;
            fy *= scale;
            fz *= scale;
            mag = maxForce;
        }
        double[] force3 = new double[3] { fx, fy, fz };
        double[] torque3 = new double[3] { 0, 0, 0 };
        int result = setSpringValues(deviceName, force3, mag);
        Debug.Log($"[HapticFeedbackReceiver] Force=({fx:F2}, {fy:F2}, {fz:F2}) N, mag={mag:F2} N, result={result}");
    }
}
