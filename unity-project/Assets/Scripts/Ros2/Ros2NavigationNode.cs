using UnityEngine;

namespace ROS2
{
    public class Ros2NavigationNode : MonoBehaviour
    {
        [SerializeField] private string nodeName;
        [SerializeField] private string realLaserScanTopicName;
        [SerializeField] private string virtualLaserScanTopicName;

        [SerializeField] private float rangeMinFilter = 0f;
        [SerializeField] private float rangeMaxFilter = 1000f;
        [SerializeField] private float realScale = 1f;
        [SerializeField] private float virtualScale = 1f;
        private float realRangeMax;
        private float realRangeMin;
        private float realAngleMax;
        private float realAngleMin;
        private float realAngleIncrement;
        private float[] realRanges = null;
        private Vector3[] realDirections;
        private Vector3 realDirectionsNew;

        // O limite por angulo ainda não funciona. 
        // Comportamento estranho do laser no Rviz, melhor passar um filtro pelo ROS.
        [SerializeField] public float range_max = 1000f;
        [SerializeField] private float range_min = 0f;
        [SerializeField] private float angle_max = 2*Mathf.PI;
        [SerializeField] private float angle_min = 0f;
        [SerializeField] private int samples = 360;
        private float angle_increment;
        private float angle_max_deg;
        private float angle_min_deg;
        public int update_rate = 70000;
        public float time_increment = 0.000199110814719f;
        public float[] intensities;
        public Vector3[] hitPoints;
        // angle to fix in the laser detections orientation in the real env
        public int angleRVIZ = 0;
        //public Vector3[] hitPoints;//Used to store the measured points
        public float[] ranges; //Measured distances
        private readonly float rad_2_deg = 57.3f;
        [HideInInspector]
        public RaycastHit hit;

        [SerializeField] private GameObject virtualLidarPosition;
        [SerializeField] private GameObject robotPosition;
        [SerializeField] private GameObject realPrefab;
        [SerializeField] private float timeForDestructor = 0.3f;
        private GameObject destructor;

        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<sensor_msgs.msg.LaserScan> virtualScanPub;

        void Start()
        {
            angle_increment = 2 * Mathf.PI / samples;
            // rad to deg
            angle_min_deg = angle_min * rad_2_deg;
            angle_max_deg = angle_max * rad_2_deg;
            //360 degrees related to distances
            ranges = new float[samples];
            hitPoints = new Vector3[samples];
            //hitPoints = new Vector3[samples];

            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Node == null)
                ros2Node = ros2Unity.CreateNode(nodeName);
            if (ros2Unity.Ok())
            {
                virtualScanPub = ros2Node.CreatePublisher<sensor_msgs.msg.LaserScan>(virtualLaserScanTopicName);
                ros2Node.CreateSubscription<sensor_msgs.msg.LaserScan>(realLaserScanTopicName, msg => RealLaserScanHandler(msg));
            }
        }

        void FixedUpdate()
        {
            if (ros2Unity.Ok())
            {
                RealLaserScanUpdate();
                VirtualLaserUpdate();
                VirtualLaserHandler();
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
                    if (realRanges[i] > rangeMinFilter)
                    {
                        if (realRanges[i] < rangeMaxFilter)
                        {
                            // transform the scan topic so it can have the walker reference and corrected angles
                            realDirections[i] = new Vector3(-Mathf.Cos(realAngleMin + realAngleIncrement * i - (angle.y * Mathf.PI / 180)), -Mathf.Sin(realAngleMin + realAngleIncrement * i - (angle.y * Mathf.PI / 180)), 0).Ros2Unity();
                            realDirectionsNew = new Vector3(realDirections[i].x * realScale * realRanges[i] + robotPosition.transform.position.x, robotPosition.transform.position.y, realDirections[i].z * realScale * realRanges[i] + robotPosition.transform.position.z);
                            // Instatiate a real_prefab to warn the user that that is a possible colision in the real env
                            destructor = (GameObject)Instantiate(realPrefab, new Vector3(realDirectionsNew.x, realDirectionsNew.y, realDirectionsNew.z), new Quaternion(0, 0, 0, 1));
                            // destroy this game object after 0.3 seconds so it doesnt flood the scene
                            Destroy(destructor, timeForDestructor);
                            //Debug.DrawLine(transform.position, real_directionsNew);
                            //Debug.Log(real_directionsNew);
                        }

                    }

                }
            }
        }

        // Code created by Fabiana Machado
        void RealLaserScanHandler(sensor_msgs.msg.LaserScan msg)
        {
            realRanges = new float[msg.Ranges.Length];
            realDirections = new Vector3[msg.Ranges.Length];
            realRangeMax = msg.Range_max;
            realRangeMin = msg.Range_min;
            realRanges = msg.Ranges;
            realAngleMin = msg.Angle_min;
            realAngleMax = msg.Angle_max;
            realAngleIncrement = msg.Angle_increment;
        }

        private void VirtualLaserUpdate()
        {
            // rays always statrting from hokuyo fake in vWalker
            Vector3 rayOriginPos = robotPosition.transform.position;
            Vector3 rayOrigin = robotPosition.transform.eulerAngles;
            Vector3 rayDirection;


            for (int i = 0; i < samples; i++)
            {

                // Correcting the childs rotation in relation to the parent (90 degrees /1,57)
                float angle = (float)(i - rayOrigin.y) / rad_2_deg;
                rayDirection.x = Mathf.Cos(angle);
                rayDirection.y = 0;
                rayDirection.z = Mathf.Sin(angle);

                //Debug.Log("rayLaser:" + angle.ToString());
                // if the angle is between the limits
                if (i >= (int)angle_min_deg && i <= (int)angle_max_deg)
                {

                    // if the ray collides, this distance information will be used to fill the ranges list
                    // also, it can draw the lines from vWalker to the colision point. uncomment debug.drawline
                    // it only collides with the Default mask (1)
                    if (Physics.Raycast(rayOriginPos, rayDirection, out hit, range_max * virtualScale))
                    {
                        // if it is between the allowed ranges
                        if (hit.distance / virtualScale > range_min && hit.distance / virtualScale < range_max)
                        {


                            // the angles when summed with angle RViz cant be more than 360 degrees
                            if (i + angleRVIZ >= 360)
                            {
                                // divided by ten due to scale factor
                                ranges[i - 360 + angleRVIZ] = hit.distance / virtualScale;
                                hitPoints[i - 360 + angleRVIZ] = hit.point;
                                Debug.DrawLine(rayOriginPos, hit.point, Color.white);

                                // vector from the hit point
                                //hitPoints[i - (int)(angle_min_deg)] = hit.point;


                            }
                            else
                            {
                                // divided by ten due to scale factor
                                ranges[i + angleRVIZ - (int)(angle_min_deg)] = hit.distance / virtualScale;
                                hitPoints[i + angleRVIZ - (int)(angle_min_deg)] = hit.point;
                                //Debug.Log("hit distance: " + hit.distance + " hitPoint: " + hit.point + " angle: " + i);
                                Debug.DrawLine(rayOriginPos, hit.point, Color.white);

                                //hitPoints[i - (int)(angle_min_deg)] = hit.point;


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
                            hitPoints[i - 360 + angleRVIZ] = new Vector3(0, 0, 0);
                            //hitPoints[i - (int)(angle_min_deg)] = rayDirection*0;
                        }
                        else
                        {
                            ranges[i + angleRVIZ - (int)(angle_min_deg)] = 0;
                            hitPoints[i + angleRVIZ - (int)(angle_min_deg)] = new Vector3(0, 0, 0);
                            //hitPoints[i - (int)(angle_min_deg)] = rayDirection*0;
                        }

                    }

                }
            }
        }

        private void VirtualLaserHandler()
        {
            sensor_msgs.msg.LaserScan msg = new sensor_msgs.msg.LaserScan
            {
                Angle_min = angle_min,
                Angle_max = angle_max,
                Angle_increment = angle_increment,
                Time_increment = time_increment,
                Range_min = range_min,
                Range_max = range_max,
                Ranges = ranges,
                Intensities = intensities
            };

            virtualScanPub.Publish(msg);
        }
    }
}
