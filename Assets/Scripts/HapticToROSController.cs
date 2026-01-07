using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using System.Runtime.InteropServices;


public class HapticToROSController : MonoBehaviour
{
    
    private ROSConnection ros;
    public string rosTopicName = "/cmd_vel"; 


    public float obstaclePixelIntensity = 0;
    public float leftWallIntensity = 0;
    public float rightWallIntensity = 0;


    
    [DllImport("HapticsDirect")]
    private static extern int getPosition(string configName, double[] position3);

    [DllImport("HapticsDirect")]
    private static extern int getButtons(string configName, int[] buttons4, int[] last_buttons4, ref int inkwell);

    private const string deviceName = "Default Device";
    private double[] hapticPosition = new double[3];
    private int[] buttonState = new int[4];
    private int[] lastButtonState = new int[4];
    private int inkwell = 0;

    
    public float publishRate = 0.1f;
    private float nextPublishTime = 0f;

    
    public float linearScale = 0.5f; 
    public float yawRate = 1.0f;     

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        ros.RegisterPublisher<TwistMsg>(rosTopicName);
    }

    void Update()
    {
        
        if (Time.time >= nextPublishTime)
        {
            nextPublishTime = Time.time + publishRate;

            Vector3 hapticPos = GetHapticPosition();

            GetHapticButtons();

         
            float hx = hapticPos.x;
            float hy = hapticPos.y;
            float hz = hapticPos.z;

          
            Vector3 linearVel = new Vector3(hz, -hy, hx) * linearScale;

            
            float yaw = 0f;
            if (buttonState[0] == 1)
                yaw = +yawRate;
            if (buttonState[1] == 1)
                yaw = -yawRate;

            
            Debug.Log($"[Haptic] hPos=({hapticPos}), linearVel=({linearVel}), yaw={yaw}");

            
            TwistMsg twist = new TwistMsg
            {
               
                linear = new Vector3Msg(linearVel.x, linearVel.y, linearVel.z),
                
                angular = new Vector3Msg(0, yaw, 0)
            };

            ros.Publish(rosTopicName, twist);
        }
    }

    
    private Vector3 GetHapticPosition()
    {
        int result = getPosition(deviceName, hapticPosition);
        if (result != 0 || hapticPosition.Length < 3)
        {
           
            return Vector3.zero;
        }
        return new Vector3(
            (float)hapticPosition[0],
            (float)hapticPosition[1],
            (float)hapticPosition[2]
        );
    }

   
    private void GetHapticButtons()
    {
        getButtons(deviceName, buttonState, lastButtonState, ref inkwell);
        
    }
}
