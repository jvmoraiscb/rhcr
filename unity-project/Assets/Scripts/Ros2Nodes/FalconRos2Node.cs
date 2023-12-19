using UnityEngine;
using ROS2;

public class FalconRos2Node : MonoBehaviour
{
    [Header("ROS2 Constants")]
    [SerializeField] private string nodeName = "FalconNode_Unity";
    [SerializeField] private string positionTopicName = "position_vector";
    [SerializeField] private string forceTopicName = "force_vector";
    [SerializeField] private string rgbTopicName = "rgb_vector";
    [SerializeField] private string rightButtonTopicName = "right_button";
    [SerializeField] private string upButtonTopicName = "up_button";
    [SerializeField] private string centerButtonTopicName = "center_button";
    [SerializeField] private string leftButtonTopicName = "left_button";

    [Header("Falcon Constants")]
    [SerializeField] private FalconMiddleware falconMid;

    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Vector3> force_pub;
    private IPublisher<geometry_msgs.msg.Vector3> rgb_pub;

    // falcon variables
    private const int RIGHT = 1;
    private const int UP = 2;
    private const int CENTER = 3;
    private const int LEFT = 4;
    private int lastStatus_right = -1;
    private int lastStatus_up = -1;
    private int lastStatus_center = -1;
    private int lastStatus_left = -1;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Node == null)
            ros2Node = ros2Unity.CreateNode(nodeName);
        if (ros2Unity.Ok())
        {
            ros2Node.CreateSubscription<geometry_msgs.msg.Vector3>(positionTopicName, msg => PositionHandler(msg.X, msg.Y, msg.Z));
            ros2Node.CreateSubscription<std_msgs.msg.Int16>(rightButtonTopicName, msg => ButtonHandler(RIGHT, msg.Data));
            ros2Node.CreateSubscription<std_msgs.msg.Int16>(upButtonTopicName, msg => ButtonHandler(UP, msg.Data));
            ros2Node.CreateSubscription<std_msgs.msg.Int16>(centerButtonTopicName, msg => ButtonHandler(CENTER, msg.Data));
            ros2Node.CreateSubscription<std_msgs.msg.Int16>(leftButtonTopicName, msg => ButtonHandler(LEFT, msg.Data));

            force_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Vector3>(forceTopicName);
            rgb_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Vector3>(rgbTopicName);
        }
    }

    void Update()
    {
        if (ros2Unity.Ok())
        {
            ForceUpdate();
            RgbUpdate();
        }
    }

    void ForceUpdate()
    {
        geometry_msgs.msg.Vector3 msg = new geometry_msgs.msg.Vector3
        {
            X = falconMid.Force.x,
            Y = falconMid.Force.y,
            Z = falconMid.Force.z
        };
        force_pub.Publish(msg);
    }

    void RgbUpdate()
    {
        geometry_msgs.msg.Vector3 msg = new geometry_msgs.msg.Vector3
        {
            X = falconMid.Rgb.x,
            Y = falconMid.Rgb.y,
            Z = falconMid.Rgb.z
        };
        rgb_pub.Publish(msg);
    }

    void PositionHandler(double x, double y, double z)
    {
        falconMid.Position = new Vector3((float)x, (float)y, (float)z);
    }

    void ButtonHandler(int button, int value)
    {
        if (button == RIGHT && value != lastStatus_right) {
            if (lastStatus_right != -1)
                falconMid.RightButtonHandler();
            lastStatus_right = value;
        }
        if (button == UP && value != lastStatus_up) {
            if (lastStatus_up != -1)
                falconMid.UpButtonHandler();
            lastStatus_up = value;
        }
        if (button == CENTER && value != lastStatus_center) {
            if (lastStatus_center != -1)
                falconMid.CenterButtonHandler();
            lastStatus_center = value;
        }
        if (button == LEFT && value != lastStatus_left) {
            if (lastStatus_left != -1)
                falconMid.LeftButtonHandler();
            lastStatus_left = value;
        }
    }
}
