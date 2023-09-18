using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RosCmdVelPublisher : UnityPublisher<MessageTypes.Geometry.Twist>
    {
        [SerializeField]
        private AckermannEnv ackermannEnv;

        private MessageTypes.Geometry.Twist message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void Update()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.Twist();
        }
        private void UpdateMessage()
        {
            message.linear.x = ackermannEnv.throttle;
            message.linear.y = 0;
            message.linear.z = 0;

            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = ackermannEnv.steer;

            Publish(message);
        }
    }
}
