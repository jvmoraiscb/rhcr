using UnityEngine;

namespace ROS2
{
    public class FalconNode : MonoBehaviour
    {
        [SerializeField]
        private FalconEnv falconEnv;

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
                ros2Node = ros2Unity.CreateNode("UnityNode");
            if (ros2Unity.Ok())
            {
                ros2Node.CreateSubscription<geometry_msgs.msg.Vector3>("position_vector", msg => PositionHandler(msg.X, msg.Y, msg.Z));
                ros2Node.CreateSubscription<std_msgs.msg.Int16>("right_button", msg => ButtonHandler(RIGHT, msg.Data));
                ros2Node.CreateSubscription<std_msgs.msg.Int16>("up_button", msg => ButtonHandler(UP, msg.Data));
                ros2Node.CreateSubscription<std_msgs.msg.Int16>("center_button", msg => ButtonHandler(CENTER, msg.Data));
                ros2Node.CreateSubscription<std_msgs.msg.Int16>("left_button", msg => ButtonHandler(LEFT, msg.Data));

                force_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Vector3>("force_vector");
                rgb_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Vector3>("rgb_vector");
            }
        }

        void Update()
        {
            ForceHandler();
            RgbHandler();
        }

        void ForceHandler()
        {
            geometry_msgs.msg.Vector3 msg = new geometry_msgs.msg.Vector3();
            msg.X = falconEnv.force.x;
            msg.Y = falconEnv.force.y;
            msg.Z = falconEnv.force.z;
            force_pub.Publish(msg);
        }

        void RgbHandler()
        {
            geometry_msgs.msg.Vector3 msg = new geometry_msgs.msg.Vector3();
            msg.X = falconEnv.rgb.x;
            msg.Y = falconEnv.rgb.y;
            msg.Z = falconEnv.rgb.z;
            rgb_pub.Publish(msg);
        }

        void PositionHandler(double x, double y, double z)
        {
            falconEnv.position.x = (float)x;
            falconEnv.position.y = (float)y;
            falconEnv.position.z = (float)z;
        }

        void ButtonHandler(int button, int value)
        {
            if (button == RIGHT && value != lastStatus_right) {
                if (lastStatus_right != -1)
                    falconEnv.Right_button_handler();
                lastStatus_right = value;
            }
            if (button == UP && value != lastStatus_up) {
                if (lastStatus_up != -1)
                    falconEnv.Up_button_handler();
                lastStatus_up = value;
            }
            if (button == CENTER && value != lastStatus_center) {
                if (lastStatus_center != -1)
                    falconEnv.Center_button_handler();
                lastStatus_center = value;
            }
            if (button == LEFT && value != lastStatus_left) {
                if (lastStatus_left != -1)
                    falconEnv.Left_button_handler();
                lastStatus_left = value;
            }
        }
    }

}
