using UnityEngine;
using ROS2;

public class VirtualSensorsRos2Node : MonoBehaviour
{
    [Header("ROS2 Constants")]
    [SerializeField] private string nodeName = "VirtualSensorsNode";
    [SerializeField] private string odomTopicName = "virtual_odom";
    [SerializeField] private string scanTopicName = "virtual_scan";
    
    [Header("Virtual Scan Constants")]
    [SerializeField] private GameObject virtualLidarPosition;
    [SerializeField] private float rangeMinFilter = 0f;
    [SerializeField] private float rangeMaxFilter = 1000f;
    [SerializeField] private float angleMinFilter = 0f;
    [SerializeField] private float angleMaxFilter = 2 * Mathf.PI;
    [SerializeField] private float scale = 1f;
    [SerializeField] private int obstaclesLayer = 6;
    [SerializeField] private int samples = 360;
    [SerializeField] private int angleRVIZ = 0;  // angle to fix in the laser detections orientation in the real env

    [Header("Virtual Odom Constants")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    // virtual scan variables
    private int angleMaxDeg;
    private int angleMinDeg;
    private float angleIncrement;
    private float[] ranges;
    private Vector3[] positions;

    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ROS2Clock ros2Clock;
    private IPublisher<nav_msgs.msg.Odometry> odomPub;
    private IPublisher<sensor_msgs.msg.LaserScan> scanPub;

    void Start()
    {
        angleMaxDeg = (int)(angleMaxFilter * 180 / Mathf.PI);
        angleMinDeg = (int)(angleMinFilter * 180 / Mathf.PI);
        angleIncrement = 2 * Mathf.PI / samples;
        ranges = new float[samples];
        positions = new Vector3[samples];
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Node == null)
            ros2Node = ros2Unity.CreateNode(nodeName);
        if (ros2Unity.Ok())
        {
            ros2Clock = new ROS2Clock();
            odomPub = ros2Node.CreatePublisher<nav_msgs.msg.Odometry>(odomTopicName);
            scanPub = ros2Node.CreatePublisher<sensor_msgs.msg.LaserScan>(scanTopicName);
        }
    }

    void Update()
    {
        if (ros2Unity.Ok())
        {
            ScanUpdate();
            OdomUpdate();
        }
    }
    void OdomUpdate()
    {
        nav_msgs.msg.Odometry msg = new nav_msgs.msg.Odometry
        {
            Header = new std_msgs.msg.Header
            {
                Frame_id = "odom",
            },
            Child_frame_id = "base_link",
            Pose = new geometry_msgs.msg.PoseWithCovariance
            {
                Pose = new geometry_msgs.msg.Pose
                {
                    Position = new geometry_msgs.msg.Point
                    {
                        X = ackermannMid.RosPosition.x,
                        Y = ackermannMid.RosPosition.y,
                        Z = ackermannMid.RosPosition.z,
                    },
                    Orientation = new geometry_msgs.msg.Quaternion
                    {
                        X = ackermannMid.RosRotation.x,
                        Y = ackermannMid.RosRotation.y,
                        Z = ackermannMid.RosRotation.z,
                        W = ackermannMid.RosRotation.w,
                    }
                }
            }
        };
        ros2Clock.UpdateROSClockTime(msg.Header.Stamp);
        odomPub.Publish(msg);
    }

    private void ScanUpdate()
    {
        int layerMask = 1 << obstaclesLayer;
        // rays always statrting from hokuyo fake in vWalker
        Vector3 rayOriginPos = virtualLidarPosition.transform.position;
        Vector3 rayOrigin = virtualLidarPosition.transform.eulerAngles;
        Vector3 rayDirection;
        for (int i = 0; i < samples; i++)
        {
            // Correcting the childs rotation in relation to the parent (90 degrees /1,57)
            float angle = (i - rayOrigin.y) / (180 / Mathf.PI);
            rayDirection.x = Mathf.Cos(angle);
            rayDirection.y = 0;
            rayDirection.z = Mathf.Sin(angle);
            // if the angle is between the limits
            if (i >= angleMinDeg && i <= angleMaxDeg)
            {
                // if the ray collides, this distance information will be used to fill the ranges list
                // also, it can draw the lines from vWalker to the colision point. uncomment debug.drawline
                // it only collides with the Default mask (1)
                if (Physics.Raycast(rayOriginPos, rayDirection, out RaycastHit hit, rangeMaxFilter * scale, layerMask))
                {
                    // if it is between the allowed ranges
                    if (hit.distance / scale > rangeMinFilter && hit.distance / scale < rangeMaxFilter)
                    {
                        // the angles when summed with angle RViz cant be more than 360 degrees
                        if (i + angleRVIZ >= 360)
                        {
                            ranges[i - 360 + angleRVIZ] = hit.distance / scale;
                            positions[i - 360 + angleRVIZ] = hit.point;
                            Debug.DrawLine(rayOriginPos, hit.point, Color.white);
                        }
                        else
                        {
                            ranges[i + angleRVIZ - (angleMinDeg)] = hit.distance / scale;
                            positions[i + angleRVIZ - (angleMinDeg)] = hit.point;
                            Debug.DrawLine(rayOriginPos, hit.point, Color.white);
                        }
                    }
                }
                // if there is not a hit, the ranges receive zero 
                else
                {
                    // the angles when summed with angle RViz cant be more than 360 degrees
                    if (i + angleRVIZ >= 360)
                    {
                        ranges[i - 360 + angleRVIZ] = 0;
                        positions[i - 360 + angleRVIZ] = new Vector3(0, 0, 0);
                    }
                    else
                    {
                        ranges[i + angleRVIZ - (angleMinDeg)] = 0;
                        positions[i + angleRVIZ - (angleMinDeg)] = new Vector3(0, 0, 0);
                    }
                }
            }
        }
        sensor_msgs.msg.LaserScan msg = new sensor_msgs.msg.LaserScan
        {
            Header = new std_msgs.msg.Header
            {
                Frame_id = "laser_link",
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
