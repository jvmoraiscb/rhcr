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
                ros2Node.CreateSubscription<nav_msgs.msg.Odometry>(odomTopicName, msg => OdomHandler(msg));
                cmdVel_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>(cmdVelTopicName);
            }
        }

        void Update()
        {
            if (ros2Unity.Ok())
            {
                CmdVelHandler();
            }
        }

        void CmdVelHandler()
        {
            geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist
            {
                Linear = new geometry_msgs.msg.Vector3
                {
                    X = ackermannEnv.position.x,
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
    }

}
