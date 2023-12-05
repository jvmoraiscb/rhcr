using UnityEngine;

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
            return position;
        }
        set
        {
            position = value;
        }
    }
    public Vector3 UnityPosition
    {
        get
        {
            return new Vector3
            {
                x = position.y * -1f,
                y = position.z,
                z = position.x
            };
        }
        set
        {
            position.x = value.z;
            position.y = value.x * -1f;
            position.z = value.y;
        }
    }
    public Quaternion RosRotation
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
    public Quaternion UnityRotation
    {
        get
        {
            Vector3 euler_aux = new Vector3 {
                x = rotation.eulerAngles.y,
                y = rotation.eulerAngles.z * -1f,
                z = rotation.eulerAngles.x
            };
            return Quaternion.Euler(euler_aux);
        }
        set
        {
            Vector3 euler_aux = new Vector3
            {
                x = value.eulerAngles.z,
                y = value.eulerAngles.x,
                z = value.eulerAngles.y * -1f
            };
            rotation = Quaternion.Euler(euler_aux);
        }
    }
}
