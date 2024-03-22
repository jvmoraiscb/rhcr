using UnityEngine;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;

public class VirtualSensorsRos2Node : MonoBehaviour
{
    [Header("ROS2")]
    [SerializeField] private string cmdVelTopicName = "unity/cmd_vel";
    [SerializeField] private string odomTopicName = "unity/odom";
    [SerializeField] private string scanTopicName = "unity/scan";
    [SerializeField] private float publisherFrequency = 10f;
    private ROSConnection ros;
    private float timeElapsed;
    
    [Header("CmdVel and Odom")]
    [SerializeField] private bool isCmdVelEnabled = true;
    [SerializeField] private bool isOdomEnabled = true;
    [SerializeField] private AckermannMiddleware ackermannMid;

    [Header("Scan")]
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
        timeElapsed = 0;
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(odomTopicName);
        ros.RegisterPublisher<LaserScanMsg>(scanTopicName);
        ros.Subscribe<TwistMsg>(cmdVelTopicName, CmdVelSubscriber);
    }

    private void Update(){
        timeElapsed += Time.deltaTime;

        if(timeElapsed > 1/publisherFrequency){
            OdomPublisher();
            ScanPublisher();
            timeElapsed = 0;
        }
    }

    private void CmdVelSubscriber(TwistMsg msg){
        if (!isCmdVelEnabled) return;
        ackermannMid.Throttle = (float)msg.linear.x;
        ackermannMid.Steer = (float)msg.angular.z;
    }

    private void OdomPublisher(){
        if (!isOdomEnabled) return;
        var timestamp = new TimeStamp(Clock.Now);
        var msg = new OdometryMsg{
            header = new HeaderMsg{
                frame_id = "unity_odom",
                stamp = new TimeMsg{
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds
                }
            },
            child_frame_id = "unity_base_link",
            pose = new PoseWithCovarianceMsg{
                pose = new PoseMsg{
                    position = new Vector3{
                        x = ackermannMid.Position.x,
                        y = ackermannMid.Position.y,
                        z = 0f,
                    }.To<FLU>(),
                    orientation = new Quaternion{
                        x = 0f,
                        y = 0f,
                        z = ackermannMid.Rotation.z,
                        w = ackermannMid.Rotation.w
                    }.To<FLU>()
                }
            }
        };
        ros.Publish(odomTopicName, msg);
    }

    // Code created by Fabiana Machado (Fabiana.machado@edu.ufes.br)
    private void ScanPublisher(){
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
        var timestamp = new TimeStamp(Clock.Now);
        var msg = new LaserScanMsg{
            header = new HeaderMsg{
                frame_id = "unity_laser_link",
                stamp = new TimeMsg{
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds
                }
            },
            angle_min = angleMinFilter,
            angle_max = angleMaxFilter,
            angle_increment = angleIncrement,
            time_increment = 1 / (publisherFrequency * samples),
            scan_time = 1/publisherFrequency,
            range_min = rangeMinFilter,
            range_max = rangeMaxFilter,
            ranges = ranges
        };
        ros.Publish(scanTopicName, msg);
    }
}
