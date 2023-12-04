using UnityEngine;

namespace ROS2
{
    public class Ros2FalconNode : MonoBehaviour
    {
        [SerializeField]
        private MiddlewareFalcon falconMid;
        [SerializeField]
        private string nodeName;
        [SerializeField]
        private string positionTopicName;
        [SerializeField]
        private string rightButtonTopicName;
        [SerializeField]
        private string upButtonTopicName;
        [SerializeField]
        private string centerButtonTopicName;
        [SerializeField]
        private string leftButtonTopicName;
        [SerializeField]
        private string forceTopicName;
        [SerializeField]
        private string rgbTopicName;

        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<geometry_msgs.msg.Vector3> force_pub;
        private IPublisher<geometry_msgs.msg.Vector3> rgb_pub;

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
                ForceHandler();
                RgbHandler();
            }
            
        }

        void ForceHandler()
        {
            geometry_msgs.msg.Vector3 msg = new geometry_msgs.msg.Vector3
            {
                X = falconMid.force.x,
                Y = falconMid.force.y,
                Z = falconMid.force.z
            };

            force_pub.Publish(msg);
        }

        void RgbHandler()
        {
            geometry_msgs.msg.Vector3 msg = new geometry_msgs.msg.Vector3
            {
                X = falconMid.rgb.x,
                Y = falconMid.rgb.y,
                Z = falconMid.rgb.z
            };

            rgb_pub.Publish(msg);
        }

        void PositionHandler(double x, double y, double z)
        {
            falconMid.position.x = (float)x;
            falconMid.position.y = (float)y;
            falconMid.position.z = (float)z;
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

}
