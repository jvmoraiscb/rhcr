using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class RealSensorsRos2Node : MonoBehaviour
{
    [Header("ROS2")]
    [SerializeField] private string cmdVelTopicName = "cmd_vel";
    [SerializeField] private string odomTopicName = "odom";
    [SerializeField] private string scanTopicName = "scan";
    [SerializeField] private float publisherFrequency = 10f;
    private ROSConnection ros;
    private float timeElapsed;

    [Header("CmdVel and Odom")]
    [SerializeField] private bool isCmdVelEnabled = true;
    [SerializeField] private bool isOdomEnabled = true;
    [SerializeField] private AckermannMiddleware ackermannMid;

    [Header("Scan")]
    [SerializeField] private bool isScanEnabled = true;
    [SerializeField] private GameObject lidarGameObject;
    [SerializeField] private GameObject wallPrefab;
    [SerializeField] private float scale = 1f;
    private System.Collections.Generic.List<GameObject> walls;

    private void Start(){
        walls = new System.Collections.Generic.List<GameObject>();
        timeElapsed = 0;
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(cmdVelTopicName);
        ros.Subscribe<OdometryMsg>(odomTopicName, OdomSubscriber);
        ros.Subscribe<LaserScanMsg>(scanTopicName, ScanSubscriber);
    }

    private void Update(){
        timeElapsed += Time.deltaTime;

        if(timeElapsed > 1/publisherFrequency){
            CmdVelPublisher();
            timeElapsed = 0;
        }
    }

    private void CmdVelPublisher(){
        if (!isCmdVelEnabled) return;
        var msg = new TwistMsg{
            linear = new Vector3Msg{
                x = ackermannMid.Throttle,
                y = 0f,
                z = 0f
            },
            angular = new Vector3Msg{
                x = 0f,
                y = 0f,
                z = ackermannMid.Steer
            }
        };
        ros.Publish(cmdVelTopicName, msg);
    }

    private void OdomSubscriber(OdometryMsg msg){
        if (!isOdomEnabled) return;
        ackermannMid.Position = new Vector3Msg{
            x = (float)msg.pose.pose.position.x,
            y = (float)msg.pose.pose.position.y,
            z = 0f
        }.From<FLU>();
        ackermannMid.Rotation = new QuaternionMsg{
            x = 0f,
            y = 0f,
            z = (float)msg.pose.pose.orientation.z,
            w = (float)msg.pose.pose.orientation.w
        }.From<FLU>();
    }

    // Code created by Fabiana Machado (Fabiana.machado@edu.ufes.br)
    void ScanSubscriber(LaserScanMsg msg){
        if (!isScanEnabled) return;
        Vector3 lidarPosition = lidarGameObject.transform.position;
        Vector3 lidarAngle = lidarGameObject.transform.eulerAngles;
        foreach (GameObject wall in walls){
            Destroy(wall);
        }
        walls.Clear();
        if (msg.ranges != null){
            for (int i = 0; i < msg.ranges.Length; i++){
                // transform the scan topic so it can have the walker reference and corrected angles
                Vector3 position = new Vector3Msg(-Mathf.Cos(msg.angle_min + msg.angle_increment * i - (lidarAngle.y * Mathf.PI / 180)), -Mathf.Sin(msg.angle_min + msg.angle_increment * i - (lidarAngle.y * Mathf.PI / 180)), 0).From<FLU>();
                Vector3 directionsNew = new Vector3(position.x * scale * msg.ranges[i] + lidarPosition.x, lidarPosition.y, position.z * scale * msg.ranges[i] + lidarPosition.z);
                // Instatiate a prefab to warn the user that that is a possible colision in the real env
                walls.Add(Instantiate(wallPrefab, new Vector3(directionsNew.x, directionsNew.y, directionsNew.z), new Quaternion(0, 0, 0, 1)));
            }
        }
    }
}