using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RosCmdVelPublisher : UnityPublisher<MessageTypes.Geometry.Twist>
    {
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
            message.linear.x = ackermann.throttle;
            message.linear.y = 0;
            message.linear.z = 0;

            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = ackermann.steer;

            Publish(message);
        }
    }
}
