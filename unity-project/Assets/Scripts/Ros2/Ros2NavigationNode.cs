using UnityEngine;

namespace ROS2
{
    public class Ros2NavigationNode : MonoBehaviour
    {
        [SerializeField] private string nodeName;
        [SerializeField] private string realLaserScanTopicName;
        [SerializeField] private string virtualLaserScanTopicName;

        [SerializeField] private float range_min_filter;
        [SerializeField] private float range_max_filter;
        private float real_range_max;
        private float real_range_min;
        private float real_angle_min;
        private float real_angle_max;
        private float real_angle_increment;
        private float[] real_ranges = null;
        private Vector3[] real_directions;
        private Vector3 real_directionsNew;

        // O limite por angulo ainda não funciona. 
        // Comportamento estranho do laser no Rviz, melhor passar um filtro pelo ROS.
        public float range_max = 1000;
        public float range_min = 0.0f;
        public float angle_max = 6.28f;
        public float angle_min = 0.0f;
        private float angle_max_deg;
        private float angle_min_deg;
        public float angle_increment = 0.0174533f;
        private Vector3 positionDiff;
        public int samples = 360;
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
        [SerializeField] private GameObject real_prefab;
        [SerializeField] private float timeForDestructor;
        private GameObject destructor;

        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<sensor_msgs.msg.LaserScan> virtualScanPub;

        void Start()
        {
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

        void Update()
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

            if (real_ranges != null)
            {
                for (int i = 0; i < real_ranges.Length; i++)
                {
                    // if the real_ranges are between the lower and upper limits
                    if (real_ranges[i] > range_min_filter)
                    {
                        if (real_ranges[i] < range_max_filter)
                        {
                            // transform the scan topic so it can have the walker reference and corrected angles
                            real_directions[i] = new Vector3(-Mathf.Cos(real_angle_min + real_angle_increment * i - (angle.y * Mathf.PI / 180)), -Mathf.Sin(real_angle_min + real_angle_increment * i - (angle.y * Mathf.PI / 180)), 0).Ros2Unity();
                            real_directionsNew = new Vector3(real_directions[i].x * 10 * real_ranges[i] + transform.position.x, transform.position.y, real_directions[i].z * 10 * real_ranges[i] + transform.position.z);
                            // Instatiate a real_prefab to warn the user that that is a possible colision in the real env
                            destructor = (GameObject)Instantiate(real_prefab, new Vector3(real_directionsNew.x, real_directionsNew.y, real_directionsNew.z), new Quaternion(0, 0, 0, 1));
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
            real_ranges = new float[msg.Ranges.Length];
            real_directions = new Vector3[msg.Ranges.Length];
            real_range_max = msg.Range_max;
            real_range_min = msg.Range_min;
            real_ranges = msg.Ranges;
            real_angle_min = msg.Angle_min;
            real_angle_max = msg.Angle_max;
            real_angle_increment = msg.Angle_increment;
        }

        private void VirtualLaserUpdate()
        {
            // rays always statrting from hokuyo fake in vWalker
            Vector3 rayOrigin = virtualLidarPosition.transform.eulerAngles;
            Vector3 rayDirection;


            for (int i = 0; i <= 361; i++)
            {

                // Correcting the childs rotation in relation to the parent (90 degrees /1,57)
                float angle = (float)(i - transform.parent.rotation.eulerAngles.y) / rad_2_deg;


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
                    if (Physics.Raycast(rayOrigin, rayDirection, out hit, range_max * 10))
                    {

                        // if it is between the allowed ranges

                        if (hit.distance / 10 > range_min && hit.distance / 10 < range_max)
                        {


                            // the angles when summed with angle RViz cant be more than 360 degrees
                            if (i + angleRVIZ >= 360)
                            {
                                // divided by ten due to scale factor
                                ranges[i - 360 + angleRVIZ] = hit.distance / 10;
                                hitPoints[i - 360 + angleRVIZ] = hit.point;
                                //Debug.DrawLine(rayOrigin, hit.point, Color.white);

                                // vector from the hit point
                                //hitPoints[i - (int)(angle_min_deg)] = hit.point;


                            }
                            else
                            {
                                // divided by ten due to scale factor
                                ranges[i + angleRVIZ - (int)(angle_min_deg)] = hit.distance / 10;
                                hitPoints[i + angleRVIZ - (int)(angle_min_deg)] = hit.point;
                                //Debug.Log("hit distance: " + hit.distance + " hitPoint: " + hit.point + " angle: " + i);
                                //Debug.DrawLine(rayOrigin, hit.point, Color.white);

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
