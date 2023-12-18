using UnityEngine;

namespace ROS2
{
    public class ClockRos2 : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Clock clock;
        private builtin_interfaces.msg.Time time;

        public builtin_interfaces.msg.Time Time
        {
            get {
                return time;
            }
        }
        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Unity.Ok())
            {
                clock = new ROS2Clock();
            }
        }

        void Update()
        {
            if (ros2Unity.Ok())
            {
                clock.UpdateROSClockTime(time);
            }
        }
    }
}
