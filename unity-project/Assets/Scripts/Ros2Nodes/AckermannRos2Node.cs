using UnityEngine;
using ROS2;

public class AckermannRos2Node : MonoBehaviour
{
    [Header("ROS2 Constants")]
    [SerializeField] private string nodeName = "AckermannNode_Unity";
    [SerializeField] private string odomTopicName = "odom";
    [SerializeField] private string cmdVelTopicName = "cmd_vel";

    [Header("Ackermann Constants")]
    [SerializeField] private AckermannMiddleware ackermannMid;

    // ros2 variables
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
        }
    }

    void Update()
    {
        if (ros2Unity.Ok())
        {
            CmdVelUpdate();
        }
    }

    void CmdVelUpdate()
    {
        geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist
        {
            Linear = new geometry_msgs.msg.Vector3
            {
                X = ackermannMid.Throttle,
                Y = 0f,
                Z = 0f
            },
            Angular = new geometry_msgs.msg.Vector3
            {
                X = 0f,
                Y = 0f,
                Z = ackermannMid.Steer
            }
        };

        cmdVel_pub.Publish(msg);
    }
    void OdomHandler(nav_msgs.msg.Odometry msg)
    {
        ackermannMid.RosPosition = new Vector3
        {
            x = (float)msg.Pose.Pose.Position.X,
            y = (float)msg.Pose.Pose.Position.Y,
            z = 0f
        };
        ackermannMid.RosRotation = new Quaternion
        {
            x = (float)msg.Pose.Pose.Orientation.X,
            y = (float)msg.Pose.Pose.Orientation.Y,
            z = (float)msg.Pose.Pose.Orientation.Z,
            w = (float)msg.Pose.Pose.Orientation.W,
        };
    }
}