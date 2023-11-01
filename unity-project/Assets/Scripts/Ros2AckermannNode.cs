using UnityEngine;

namespace ROS2
{
    public class Ros2AckermannNode : MonoBehaviour
    {
        [SerializeField]
        private AckermannEnv ackermannEnv;
        [SerializeField]
        private string nodeName;
        [SerializeField]
        private string odomTopicName;
        [SerializeField]
        private string cmdVelTopicName;
        [SerializeField]
        private string laserScanTopicName;

        [SerializeField]
        private GameObject LidarPosition;
        [SerializeField]
        private float range_min_filter;
        [SerializeField]
        private float range_max_filter;
        private float range_max;
        private float range_min;
        private float angle_min;
        private float angle_max;
        private float angle_increment;
        private float[] ranges = null;
        private Vector3[] directions;
        private Vector3 directionsNew;

        [SerializeField]
        private GameObject prefab;
        [SerializeField]
        private float tempoParaDestruir;
        private GameObject destruidor;

        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<geometry_msgs.msg.Twist> cmdVel_pub;

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Node == null)
                ros2Node = ros2Unity.CreateNode(nodeName);
            if (ros2Unity.Ok())
            {
                cmdVel_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>(cmdVelTopicName);
                ros2Node.CreateSubscription<nav_msgs.msg.Odometry>(odomTopicName, msg => OdomHandler(msg));
                ros2Node.CreateSubscription<sensor_msgs.msg.LaserScan>(laserScanTopicName, msg => LaserScanHandler(msg));
            }
        }

        void Update()
        {
            if (ros2Unity.Ok())
            {
                CmdVelUpdate();
                LaserScanUpdate();
            }
        }

        void CmdVelUpdate()
        {
            geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist
            {
                Linear = new geometry_msgs.msg.Vector3
                {
                    X = ackermannEnv.throttle,
                    Y = 0f,
                    Z = 0f
                },
                Angular = new geometry_msgs.msg.Vector3
                {
                    X = 0f,
                    Y = 0f,
                    Z = ackermannEnv.steer
                }
            };

            cmdVel_pub.Publish(msg);
        }

        // Code created by Fabiana Machado
        void LaserScanUpdate()
        {
            Vector3 angle = LidarPosition.transform.eulerAngles;

            if (ranges != null)
            {
                for (int i = 0; i < ranges.Length; i++)
                {
                    // if the ranges are between the lower and upper limits
                    if (ranges[i] > range_min_filter)
                    {
                        if (ranges[i] < range_max_filter)
                        {
                            // transform the scan topic so it can have the walker reference and corrected angles
                            directions[i] = new Vector3(-Mathf.Cos(angle_min + angle_increment * i - (angle.y * Mathf.PI / 180)), -Mathf.Sin(angle_min + angle_increment * i - (angle.y * Mathf.PI / 180)), 0).Ros2Unity();
                            directionsNew = new Vector3(directions[i].x * 10 * ranges[i] + transform.position.x, transform.position.y, directions[i].z * 10 * ranges[i] + transform.position.z);
                            // Instatiate a prefab to warn the user that that is a possible colision in the real env
                            destruidor = (GameObject)Instantiate(prefab, new Vector3(directionsNew.x, directionsNew.y, directionsNew.z), new Quaternion(0, 0, 0, 1));
                            // destroy this game object after 0.3 seconds so it doesnt flood the scene
                            Destroy(destruidor, tempoParaDestruir);
                            //Debug.DrawLine(transform.position, directionsNew);
                            //Debug.Log(directionsNew);
                        }

                    }

                }
            }
        }

        void OdomHandler(nav_msgs.msg.Odometry msg)
        {
            ackermannEnv.position.x = (float)msg.Pose.Pose.Position.Y * -1;
            ackermannEnv.position.y = 0f;
            ackermannEnv.position.z = (float)msg.Pose.Pose.Position.X;

            Quaternion quart_aux;

            quart_aux.x = (float)msg.Pose.Pose.Orientation.X;
            quart_aux.y = (float)msg.Pose.Pose.Orientation.Y;
            quart_aux.z = (float)msg.Pose.Pose.Orientation.Z;
            quart_aux.w = (float)msg.Pose.Pose.Orientation.W;

            Vector3 euler_aux = quart_aux.eulerAngles;

            euler_aux.y = euler_aux.z * -1;
            euler_aux.x = 0;
            euler_aux.z = 0;

            ackermannEnv.rotation = Quaternion.Euler(euler_aux);
        }
        // Code created by Fabiana Machado
        void LaserScanHandler(sensor_msgs.msg.LaserScan msg)
        {
            ranges = new float[msg.Ranges.Length];
            directions = new Vector3[msg.Ranges.Length];
            range_max = msg.Range_max;
            range_min = msg.Range_min;
            ranges = msg.Ranges;
            angle_min = msg.Angle_min;
            angle_max = msg.Angle_max;
            angle_increment = msg.Angle_increment;
        }
    }

}