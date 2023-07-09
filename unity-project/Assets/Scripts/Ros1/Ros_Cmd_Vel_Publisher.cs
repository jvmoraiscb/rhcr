using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class Ros_Cmd_Vel_Publisher : UnityPublisher<MessageTypes.Geometry.Twist>
    {
        [SerializeField]
        private Falcon falcon;
        [SerializeField]
        private Ackermann ackermann;

        private MessageTypes.Geometry.Twist message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.Twist();
        }
        private void UpdateMessage()
        {
            message.linear.x = falcon.position.z * ackermann.speed;
            message.linear.y = 0;
            message.linear.z = 0;

            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = falcon.position.x * ackermann.speed;

            Publish(message);
        }
    }
}
