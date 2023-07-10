using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RosOdomSubscriber : UnitySubscriber<MessageTypes.Nav.Odometry>
    {
        [SerializeField]
        private Ackermann ackermann;
        
        private MessageTypes.Nav.Odometry msg;
        private float previousRealTime;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Nav.Odometry message)
        {
            msg = message;
            isMessageReceived = true;
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
            previousRealTime = Time.realtimeSinceStartup;
        }
        private void ProcessMessage()
        {
            ackermann.position.x = (float)msg.pose.pose.position.y;
            ackermann.position.y = 0f;
            ackermann.position.z = (float)msg.pose.pose.position.x;

            ackermann.rotation.x = 0f;
            ackermann.rotation.y = (float)msg.pose.pose.orientation.z * -1;
            ackermann.rotation.z = 0f;
            ackermann.rotation.w = (float)msg.pose.pose.orientation.w;

            float deltaTime = Time.realtimeSinceStartup - previousRealTime;
            isMessageReceived = false;
        }
    }
}