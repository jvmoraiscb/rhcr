using UnityEngine;
using ROS2;

public class AckermannMiddleware : MonoBehaviour
{
    [SerializeField] private Vector3 position;
    [SerializeField] private Quaternion rotation;
    [SerializeField] private float throttle;
    [SerializeField] private float steer;

    public float Throttle
    {
        get
        {
            return throttle;
        }
        set
        {
            throttle = value;
        }
    }
    public float Steer
    {
        get
        {
            return steer;
        }
        set
        {
            steer = value;
        }
    }
    public Vector3 RosPosition
    {
        get
        {
            return Transformations.Unity2Ros(position);
        }
        set
        {
            position = Transformations.Ros2Unity(value);
        }
    }
    public Vector3 UnityPosition
    {
        get
        {
            return position;
        }
        set
        {
            position = value;
        }
    }
    public Quaternion RosRotation
    {
        get
        {
            return Transformations.Unity2Ros(rotation);
        }
        set
        {
            rotation = Transformations.Ros2Unity(value);
        }
    }
    public Quaternion UnityRotation
    {
        get
        {
            return rotation;
        }
        set
        {
            rotation = value;
        }
    }
}
