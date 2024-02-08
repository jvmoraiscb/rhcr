using UnityEngine;
using ROS2;

public class RealSensorsRos2Node : MonoBehaviour
{
    [Header("ROS2 Constants")]
    [SerializeField] private string nodeName = "RealSensorsNode_Unity";
    [SerializeField] private string cmdVelTopicName = "cmd_vel";
    [SerializeField] private string odomTopicName = "odom";
    [SerializeField] private string scanTopicName = "scan";

    [Header("CmdVel and Odom Constants")]
    [SerializeField] private bool isCmdVelEnabled = true;
    [SerializeField] private bool isOdomEnabled = true;
    [SerializeField] private AckermannMiddleware ackermannMid;

    [Header("Scan Constants")]
    [SerializeField] private bool isScanEnabled = true;
    [SerializeField] private GameObject lidarGameObject;
    [SerializeField] private GameObject robotGameObject;
    [SerializeField] private GameObject wallPrefab;
    [SerializeField] private float rangeMinFilter = 0f;
    [SerializeField] private float rangeMaxFilter = 1000f;
    // [SerializeField] private float angleMinFilter = 0f;
    // [SerializeField] private float angleMaxFilter = 2*Mathf.PI;
    [SerializeField] private float scale = 1f;

    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Twist> cmdVelPub;

    // scan variables
    float[] ranges;
    float angleMin;
    float angleIncrement;
    private System.Collections.Generic.List<GameObject> walls;

    private void Start(){
        walls = new System.Collections.Generic.List<GameObject>();
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Node ??= ros2Unity.CreateNode(nodeName);
        if (ros2Unity.Ok()){
            ros2Node.CreateSubscription<sensor_msgs.msg.LaserScan>(scanTopicName, msg => ScanHandler(msg));
            ros2Node.CreateSubscription<nav_msgs.msg.Odometry>(odomTopicName, msg => OdomHandler(msg));
            cmdVelPub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>(cmdVelTopicName);
        }
    }

    private void Update(){
        if(ros2Unity.Ok()){
            CmdVelUpdate();
            ScanUpdate();
        }
    }

    private void CmdVelUpdate(){
        if (!isCmdVelEnabled) return;
        geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist{
            Linear = new geometry_msgs.msg.Vector3{
                X = ackermannMid.Throttle,
                Y = 0f,
                Z = 0f
            },
            Angular = new geometry_msgs.msg.Vector3{
                X = 0f,
                Y = 0f,
                Z = ackermannMid.Steer
            }
        };
        cmdVelPub.Publish(msg);
    }

    private void OdomHandler(nav_msgs.msg.Odometry msg){
        if (!isOdomEnabled) return;
        ackermannMid.RosPosition = new Vector3{
            x = (float)msg.Pose.Pose.Position.X,
            y = (float)msg.Pose.Pose.Position.Y,
            z = 0f
        };
        ackermannMid.RosRotation = new Quaternion{
            x = 0f,
            y = 0f,
            z = (float)msg.Pose.Pose.Orientation.Z,
            w = (float)msg.Pose.Pose.Orientation.W
        };
    }

    // Code created by Fabiana Machado (Fabiana.machado@edu.ufes.br)
    void ScanHandler(sensor_msgs.msg.LaserScan msg){
        if (!isScanEnabled) return;
        ranges = msg.Ranges;
        angleMin = msg.Angle_min;
        angleIncrement = msg.Angle_increment;
    }

    // Code created by Fabiana Machado (Fabiana.machado@edu.ufes.br)
    void ScanUpdate(){
        if (!isScanEnabled) return;
        Vector3 robotPosition = robotGameObject.transform.position;
        Vector3 lidarAngle = lidarGameObject.transform.eulerAngles;
        foreach (GameObject wall in walls){
            Destroy(wall);
        }
        walls.Clear();
        if (ranges != null){
            for (int i = 0; i < ranges.Length; i++){
                // if the ranges are between the lower and upper limits
                if (ranges[i] > rangeMinFilter && ranges[i] < rangeMaxFilter){
                    // transform the scan topic so it can have the walker reference and corrected angles
                    Vector3 position = new Vector3(-Mathf.Cos(angleMin + angleIncrement * i - (lidarAngle.y * Mathf.PI / 180)), -Mathf.Sin(angleMin + angleIncrement * i - (lidarAngle.y * Mathf.PI / 180)), 0).Ros2Unity();
                    Vector3 directionsNew = new Vector3(position.x * scale * ranges[i] + robotPosition.x, robotPosition.y, position.z * scale * ranges[i] + robotPosition.z);
                    // Instatiate a prefab to warn the user that that is a possible colision in the real env
                    walls.Add(Instantiate(wallPrefab, new Vector3(directionsNew.x, directionsNew.y, directionsNew.z), new Quaternion(0, 0, 0, 1)));
                    // destructor = Instantiate(wallPrefab, new Vector3(realDirectionsNew.x, realDirectionsNew.y, realDirectionsNew.z), new Quaternion(0, 0, 0, 1));
                    // destroy this game object after 0.3 seconds so it doesnt flood the scene
                    // Destroy(destructor, timeToDestroyWall);
                    //Debug.DrawLine(transform.position, real_directionsNew);
                    //Debug.Log(real_directionsNew);
                }
            }
        }
    }
}
