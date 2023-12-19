using UnityEngine;

namespace ROS2
{
    public class LaserScanRos2Node : MonoBehaviour
    {
        [SerializeField] private string nodeName = "LaserScanNode_Unity";
        [SerializeField] private string realLaserScanTopicName = "real_scan";
        [SerializeField] private string virtualLaserScanTopicName = "virtual_scan";
        [SerializeField] private float rangeMinFilter = 0f;
        [SerializeField] private float rangeMaxFilter = 1000f;
        [SerializeField] private float angleMinFilter = 0f;
        [SerializeField] private float angleMaxFilter = 2*Mathf.PI;
        [SerializeField] private float realScale = 1f;
        [SerializeField] private float virtualScale = 1f;
        [SerializeField] private int obstaclesLayer = 6;
        [SerializeField] private int virtualSamples = 360;
        [SerializeField] private int virtualAngleRVIZ = 0;  // angle to fix in the laser detections orientation in the real env
        private float realRangeMax;
        private float realRangeMin;
        private float realAngleMax;
        private float realAngleMin;
        private float realAngleIncrement;
        private float[] realRanges = null;
        private Vector3[] realPositions;

        private float virtualRangeMax;
        private float virtualRangeMin;
        private float virtualAngleMax;
        private float virtualAngleMin;
        private int virtualAngleMaxDeg;
        private int virtualAngleMinDeg;
        private float virtualAngleIncrement;
        private float[] virtualRanges;
        private Vector3[] virtualPositions;
        
        // ros2 variables
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private ROS2Clock ros2Clock;
        private IPublisher<sensor_msgs.msg.LaserScan> virtualScanPub;

        [SerializeField] private GameObject virtualLidarPosition;
        [SerializeField] private GameObject robotPosition;
        [SerializeField] private GameObject wallPrefab;
        [SerializeField] private float timeToDestroyWall = 0.1f;
        private GameObject destructor;

        void Start()
        {
            virtualAngleMax = angleMaxFilter;
            virtualAngleMin = angleMinFilter;
            virtualAngleMaxDeg = (int)(angleMaxFilter * 180/Mathf.PI);
            virtualAngleMinDeg = (int)(angleMinFilter * 180/Mathf.PI);
            virtualRangeMax = rangeMaxFilter;
            virtualRangeMin = rangeMinFilter;
            virtualAngleIncrement = 2*Mathf.PI / virtualSamples;
            virtualRanges = new float[virtualSamples];
            virtualPositions = new Vector3[virtualSamples];
            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Node == null)
                ros2Node = ros2Unity.CreateNode(nodeName);
            if (ros2Unity.Ok())
            {
                ros2Clock = new ROS2Clock();
                virtualScanPub = ros2Node.CreatePublisher<sensor_msgs.msg.LaserScan>(virtualLaserScanTopicName);
                ros2Node.CreateSubscription<sensor_msgs.msg.LaserScan>(realLaserScanTopicName, msg => RealLaserScanHandler(msg));
            }
        }

        void FixedUpdate()
        {
            if (ros2Unity.Ok())
            {
                RealLaserScanUpdate();
            }
        }

        // Code created by Fabiana Machado
        void RealLaserScanUpdate()
        {
            Vector3 angle = virtualLidarPosition.transform.eulerAngles;
            if (realRanges != null)
            {
                for (int i = 0; i < realRanges.Length; i++)
                {
                    // if the real_ranges are between the lower and upper limits
                    if (realRanges[i] > rangeMinFilter && realRanges[i] < rangeMaxFilter)
                    {
                        // transform the scan topic so it can have the walker reference and corrected angles
                        realPositions[i] = new Vector3(-Mathf.Cos(realAngleMin + realAngleIncrement * i - (angle.y * Mathf.PI / 180)), -Mathf.Sin(realAngleMin + realAngleIncrement * i - (angle.y * Mathf.PI / 180)), 0).Ros2Unity();
                        Vector3 realDirectionsNew = new Vector3(realPositions[i].x * realScale * realRanges[i] + robotPosition.transform.position.x, robotPosition.transform.position.y, realPositions[i].z * realScale * realRanges[i] + robotPosition.transform.position.z);
                        // Instatiate a real_prefab to warn the user that that is a possible colision in the real env
                        destructor = (GameObject)Instantiate(wallPrefab, new Vector3(realDirectionsNew.x, realDirectionsNew.y, realDirectionsNew.z), new Quaternion(0, 0, 0, 1));
                        // destroy this game object after 0.3 seconds so it doesnt flood the scene
                        Destroy(destructor, timeToDestroyWall);
                        //Debug.DrawLine(transform.position, real_directionsNew);
                        //Debug.Log(real_directionsNew);
                    }
                }
            }
        }

        // Code created by Fabiana Machado
        void RealLaserScanHandler(sensor_msgs.msg.LaserScan msg)
        {
            realRanges = new float[msg.Ranges.Length];
            realPositions = new Vector3[msg.Ranges.Length];
            realRangeMax = msg.Range_max;
            realRangeMin = msg.Range_min;
            realRanges = msg.Ranges;
            realAngleMin = msg.Angle_min;
            realAngleMax = msg.Angle_max;
            realAngleIncrement = msg.Angle_increment;
        }
    }
}
