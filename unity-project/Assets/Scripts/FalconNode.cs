using UnityEngine;

namespace ROS2
{
    public class FalconNode : MonoBehaviour
    {
        [SerializeField]
        private CarController carController;
        [SerializeField]
        private CollisionController collisionController;

        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private ISubscription<geometry_msgs.msg.Vector3> position_sub;
        private ISubscription<std_msgs.msg.Int16> right_sub;
        private ISubscription<std_msgs.msg.Int16> up_sub;
        private ISubscription<std_msgs.msg.Int16> center_sub;
        private ISubscription<std_msgs.msg.Int16> left_sub;
        private IPublisher<geometry_msgs.msg.Vector3> force_pub;

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
                position_sub = ros2Node.CreateSubscription<geometry_msgs.msg.Vector3>("position_vector", msg => PositionHandler(msg.X, msg.Z));
                right_sub = ros2Node.CreateSubscription<std_msgs.msg.Int16>("right_button", msg => ButtonHandler(RIGHT, msg.Data));
                up_sub = ros2Node.CreateSubscription<std_msgs.msg.Int16>("up_button", msg => ButtonHandler(UP, msg.Data));
                center_sub = ros2Node.CreateSubscription<std_msgs.msg.Int16>("center_button", msg => ButtonHandler(CENTER, msg.Data));
                left_sub = ros2Node.CreateSubscription<std_msgs.msg.Int16>("left_button", msg => ButtonHandler(LEFT, msg.Data));

                force_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Vector3>("force_vector");
            }
        }

        void Update()
        {
            ForceHandler();
        }

        void PositionHandler(double x, double z)
        {
            carController.throttle = (float)z;
            carController.steer = (float)x;
        }
        void ForceHandler()
        {
            geometry_msgs.msg.Vector3 msg = new geometry_msgs.msg.Vector3();
            msg.X = collisionController.force.x;
            msg.Y = collisionController.force.y;
            msg.Z = collisionController.force.z;
            force_pub.Publish(msg);
        }
        void ButtonHandler(int button, int value)
        {
            if (button == RIGHT && value != lastStatus_right) {
                lastStatus_right = value;
            }
            if (button == UP && value != lastStatus_up) {
                lastStatus_up = value;
            }
            if (button == CENTER && value != lastStatus_center) {
                if(lastStatus_center != -1)
                    carController.isBreaking = !carController.isBreaking;
                lastStatus_center = value;
            }
            if (button == LEFT && value != lastStatus_left) {
                lastStatus_left = value;
            }

        }
    }

}  // namespace ROS2
