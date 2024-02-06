using UnityEngine;
using ROS2;
using System.Linq;
public class LaserScanRos2Node : MonoBehaviour
{
    [SerializeField] private string nodeName = "LaserScanNode_Unity";
    [SerializeField] private string ScanTopicName = "scan";
    [SerializeField] private float rangeMinFilter = 0f;
    [SerializeField] private float rangeMaxFilter = 1000f;
    [SerializeField] private float angleMinFilter = 0f;
    [SerializeField] private float angleMaxFilter = 2*Mathf.PI;
    [SerializeField] private float realScale = 1f;
    private float rangeMax;
    private float rangeMin;
    private float angleMax;
    private float angleMin;
    private float angleIncrement;
    private float[] ranges = null;
    private Vector3[] positions = null;
    private System.Collections.Generic.List<GameObject> walls;
    
    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    [SerializeField] private GameObject virtualLidarPosition;
    [SerializeField] private GameObject robotPosition;
    [SerializeField] private GameObject wallPrefab;
    // [SerializeField] private float timeToDestroyWall = 0.1f;
    // private GameObject destructor;

    void Start()
    {
        walls = new System.Collections.Generic.List<GameObject>();
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Node == null)
            ros2Node = ros2Unity.CreateNode(nodeName);
        if (ros2Unity.Ok())
        {
            ros2Node.CreateSubscription<sensor_msgs.msg.LaserScan>(ScanTopicName, msg => RealLaserScanHandler(msg));
        }
    }

    void Update(){
        if(ros2Unity.Ok()){
            if(walls.Count != 0){
                foreach (GameObject wall in walls){
                    Destroy(wall);
                }
                walls.Clear();
            }
            RealLaserScanUpdate();
        }
    }

    // Code created by Fabiana Machado
    void RealLaserScanUpdate()
    {
        Vector3 angle = virtualLidarPosition.transform.eulerAngles;
        if (ranges != null)
        {
            for (int i = 0; i < ranges.Length; i++)
            {
                // if the real_ranges are between the lower and upper limits
                if (ranges[i] > rangeMinFilter && ranges[i] < rangeMaxFilter)
                {
                    // transform the scan topic so it can have the walker reference and corrected angles
                    positions[i] = new Vector3(-Mathf.Cos(angleMin + angleIncrement * i - (angle.y * Mathf.PI / 180)), -Mathf.Sin(angleMin + angleIncrement * i - (angle.y * Mathf.PI / 180)), 0).Ros2Unity();
                    Vector3 realDirectionsNew = new Vector3(positions[i].x * realScale * ranges[i] + robotPosition.transform.position.x, robotPosition.transform.position.y, positions[i].z * realScale * ranges[i] + robotPosition.transform.position.z);
                    // Instatiate a real_prefab to warn the user that that is a possible colision in the real env
                    walls.Add(Instantiate(wallPrefab, new Vector3(realDirectionsNew.x, realDirectionsNew.y, realDirectionsNew.z), new Quaternion(0, 0, 0, 1)));
                    // destructor = Instantiate(wallPrefab, new Vector3(realDirectionsNew.x, realDirectionsNew.y, realDirectionsNew.z), new Quaternion(0, 0, 0, 1));
                    // destroy this game object after 0.3 seconds so it doesnt flood the scene
                    // Destroy(destructor, timeToDestroyWall);
                    //Debug.DrawLine(transform.position, real_directionsNew);
                    //Debug.Log(real_directionsNew);
                }
            }
        }
    }

    // Code created by Fabiana Machado
    void RealLaserScanHandler(sensor_msgs.msg.LaserScan msg)
    {
        ranges = new float[msg.Ranges.Length];
        positions = new Vector3[msg.Ranges.Length];
        rangeMax = msg.Range_max;
        rangeMin = msg.Range_min;
        ranges = msg.Ranges;
        angleMin = msg.Angle_min;
        angleMax = msg.Angle_max;
        angleIncrement = msg.Angle_increment;
    }
}
