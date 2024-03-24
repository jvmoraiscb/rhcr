using UnityEngine;

public class AckermannMiddleware : MonoBehaviour
{
    [SerializeField] private Vector3 position;
    [SerializeField] private Quaternion rotation;
    [SerializeField] private float throttle;
    [SerializeField] private float steer;

    public float Throttle{
        get{
            return throttle;
        }
        set{
            throttle = value;
        }
    }
    public float Steer{
        get{
            return steer;
        }
        set{
            steer = value;
        }
    }
    public Vector3 Position{
        get{
            return position;
        }
        set{
            position = value;
        }
    }
    public Quaternion Rotation{
        get{
            return rotation;
        }
        set{
            rotation = value;
        }
    }
}
