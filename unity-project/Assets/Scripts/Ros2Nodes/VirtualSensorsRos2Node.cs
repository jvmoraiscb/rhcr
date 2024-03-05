using UnityEngine;
using ROS2;

public class VirtualSensorsRos2Node : MonoBehaviour
{
    [Header("ROS2 Constants")]
    [SerializeField] private string nodeName = "VirtualSensorsNode_Unity";
    [SerializeField] private string cmdVelTopicName = "unity_cmd_vel";
    [SerializeField] private string odomTopicName = "unity_odom";
    [SerializeField] private string scanTopicName = "unity_scan";
    [SerializeField] private float publisherFrequency = 100f;
    
    [Header("CmdVel and Odom Constants")]
    [SerializeField] private bool isCmdVelEnabled = true;
    [SerializeField] private bool isOdomEnabled = true;
    [SerializeField] private AckermannMiddleware ackermannMid;

    [Header("Scan Constants")]
    [SerializeField] private bool isScanEnabled = true;
    [SerializeField] private GameObject lidarPosition;
    [SerializeField] private float rangeMinFilter = 0f;
    [SerializeField] private float rangeMaxFilter = 1000f;
    [SerializeField] private float angleMinFilter = 0f;
    [SerializeField] private float angleMaxFilter = 2 * Mathf.PI;
    [SerializeField] private float scale = 1f;
    [SerializeField] private int obstaclesLayer = 6;
    [SerializeField] private int samples = 360;
    [SerializeField] private int angleRVIZ = 0;  // angle to fix in the laser detections orientation in the real env


    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ROS2Clock ros2Clock;
    private IPublisher<nav_msgs.msg.Odometry> odomPub;
    private IPublisher<sensor_msgs.msg.LaserScan> scanPub;

    // scan variables
    private int angleMaxDeg;
    private int angleMinDeg;
    private float angleIncrement;
    private float[] ranges;
    private Vector3[] positions;

    private void Start(){
        angleMaxDeg = (int)(angleMaxFilter * 180 / Mathf.PI);
        angleMinDeg = (int)(angleMinFilter * 180 / Mathf.PI);
        angleIncrement = 2 * Mathf.PI / samples;
        ranges = new float[samples];
        positions = new Vector3[samples];
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Node ??= ros2Unity.CreateNode(nodeName);
        if (ros2Unity.Ok()){
            ros2Clock = new ROS2Clock();
            ros2Node.CreateSubscription<geometry_msgs.msg.Twist>(cmdVelTopicName, msg => CmdVelHandler(msg));
            odomPub = ros2Node.CreatePublisher<nav_msgs.msg.Odometry>(odomTopicName);
            scanPub = ros2Node.CreatePublisher<sensor_msgs.msg.LaserScan>(scanTopicName);
        }
        StartCoroutine(LimitedUpdate(1/publisherFrequency));
    }

    System.Collections.IEnumerator LimitedUpdate(float waitTime){
        while (true){
            yield return new WaitForSeconds(waitTime);
            if (ros2Unity.Ok()){
                OdomUpdate();
                ScanUpdate();
            }
        }
    }

    private void CmdVelHandler(geometry_msgs.msg.Twist msg){
        if (!isCmdVelEnabled) return;
        ackermannMid.Throttle = (float)msg.Linear.X;
        ackermannMid.Steer = (float)msg.Angular.Z;
    }

    private void OdomUpdate(){
        if (!isOdomEnabled) return;
        nav_msgs.msg.Odometry msg = new nav_msgs.msg.Odometry{
            Header = new std_msgs.msg.Header{
                Frame_id = "unity_odom",
            },
            Child_frame_id = "unity_base_link",
            Pose = new geometry_msgs.msg.PoseWithCovariance{
                Pose = new geometry_msgs.msg.Pose{
                    Position = new geometry_msgs.msg.Point{
                        X = ackermannMid.RosPosition.x,
                        Y = ackermannMid.RosPosition.y,
                        Z = 0f,
                    },
                    Orientation = new geometry_msgs.msg.Quaternion{
                        X = 0f,
                        Y = 0f,
                        Z = ackermannMid.RosRotation.z,
                        W = ackermannMid.RosRotation.w
                    }
                }
            }
        };
        ros2Clock.UpdateROSClockTime(msg.Header.Stamp);
        odomPub.Publish(msg);
    }

    // Code created by Fabiana Machado (Fabiana.machado@edu.ufes.br)
    private void ScanUpdate(){
        if (!isScanEnabled) return;
        int layerMask = 1 << obstaclesLayer;
        // rays always statrting from hokuyo fake in vWalker
        Vector3 rayOriginPos = lidarPosition.transform.position;
        Vector3 rayOrigin = lidarPosition.transform.eulerAngles;
        Vector3 rayDirection;
        for (int i = 0; i < samples; i++){
            // Correcting the childs rotation in relation to the parent (90 degrees /1,57)
            float angle = (i - rayOrigin.y) / (180 / Mathf.PI);
            rayDirection.x = Mathf.Cos(angle);
            rayDirection.y = 0;
            rayDirection.z = Mathf.Sin(angle);
            // if the angle is between the limits
            if (i >= angleMinDeg && i <= angleMaxDeg){
                // if the ray collides, this distance information will be used to fill the ranges list
                // also, it can draw the lines from vWalker to the colision point. uncomment debug.drawline
                // it only collides with the Default mask (1)
                if (Physics.Raycast(rayOriginPos, rayDirection, out RaycastHit hit, rangeMaxFilter * scale, layerMask)){
                    // if it is between the allowed ranges
                    if (hit.distance / scale > rangeMinFilter && hit.distance / scale < rangeMaxFilter){
                        // the angles when summed with angle RViz cant be more than 360 degrees
                        if (i + angleRVIZ >= 360){
                            ranges[i - 360 + angleRVIZ] = hit.distance / scale;
                            positions[i - 360 + angleRVIZ] = hit.point;
                        }
                        else {
                            ranges[i + angleRVIZ - angleMinDeg] = hit.distance / scale;
                            positions[i + angleRVIZ - angleMinDeg] = hit.point;
                        }
                            //Debug.DrawLine(rayOriginPos, hit.point, Color.white);
                    }
                }
                // if there is not a hit, the ranges receive zero 
                else {
                    // the angles when summed with angle RViz cant be more than 360 degrees
                    if (i + angleRVIZ >= 360){
                        ranges[i - 360 + angleRVIZ] = 0;
                        positions[i - 360 + angleRVIZ] = new Vector3(0, 0, 0);
                    }
                    else {
                        ranges[i + angleRVIZ - angleMinDeg] = 0;
                        positions[i + angleRVIZ - angleMinDeg] = new Vector3(0, 0, 0);
                    }
                }
            }
        }
        sensor_msgs.msg.LaserScan msg = new sensor_msgs.msg.LaserScan{
            Header = new std_msgs.msg.Header{
                Frame_id = "unity_laser_link",
            },
            Angle_min = angleMinFilter,
            Angle_max = angleMaxFilter,
            Angle_increment = angleIncrement,
            Time_increment = Time.deltaTime / samples,
            Scan_time = Time.deltaTime,
            Range_min = rangeMinFilter,
            Range_max = rangeMaxFilter,
            Ranges = ranges
        };
        ros2Clock.UpdateROSClockTime(msg.Header.Stamp);
        scanPub.Publish(msg);
    }
}
