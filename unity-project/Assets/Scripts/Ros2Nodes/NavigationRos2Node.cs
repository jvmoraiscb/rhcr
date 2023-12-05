using UnityEngine;

namespace ROS2
{
    public class NavigationRos2Node : MonoBehaviour
    {
        [SerializeField]
        private AckermannMiddleware ackermannMid;
        [SerializeField]
        private string nodeName;
        [SerializeField]
        private string odomTopicName;

        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<nav_msgs.msg.Odometry> odom_pub;

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Node == null)
                ros2Node = ros2Unity.CreateNode(nodeName);
            if (ros2Unity.Ok())
            {
                odom_pub = ros2Node.CreatePublisher<nav_msgs.msg.Odometry>(odomTopicName);
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (ros2Unity.Ok())
            {
                OdomUpdate();
            }
        }
        void OdomUpdate()
        {
            nav_msgs.msg.Odometry msg = new nav_msgs.msg.Odometry
            {
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
            odom_pub.Publish(msg);
        }
    }
}
