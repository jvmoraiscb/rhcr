using UnityEngine;

// TODO: is not working
public class WheelsAnimation : MonoBehaviour
{
    [SerializeField]
    private Transform frontLeft;
    [SerializeField]
    private Transform frontRight;
    [SerializeField]
    private Transform rearLeft;
    [SerializeField]
    private Transform rearRight;

    [SerializeField]
    private float maxAngleDegrees;
    [SerializeField]
    private float angleDifferenceDegrees;

    public float wheelsSteer;
    public float wheelsThrottle;

    private void Start()
    {
        Vector3 nullAngle = new Vector3 { x = 0f, y = 0f, z = 0f };

        frontLeft.transform.rotation = Quaternion.Euler(nullAngle);
        frontRight.transform.rotation = Quaternion.Euler(nullAngle);
    }

    void Update()
    {
        FrontWheelsAngleAnimation();
        WheelsThrottleAnimation();
    }


    void FrontWheelsAngleAnimation()
    {
        Vector3 majorAngle = new Vector3 { x = 0f, y = wheelsSteer * maxAngleDegrees, z = 0f };
        Vector3 minorAngle = new Vector3 { x = 0f, y = wheelsSteer * (maxAngleDegrees - angleDifferenceDegrees), z = 0f };
        Vector3 nullAngle = new Vector3 { x = 0f, y = 0f, z = 0f };

        // turning left
        if (wheelsSteer < 0)
        {
            frontLeft.transform.localRotation = Quaternion.Euler(majorAngle);
            frontRight.transform.localRotation = Quaternion.Euler(minorAngle);

        }
        // turning right
        else if (wheelsSteer > 0)
        {
            frontLeft.transform.localRotation = Quaternion.Euler(minorAngle);
            frontRight.transform.localRotation = Quaternion.Euler(majorAngle);
        }
        else
        {
            frontLeft.transform.localRotation = Quaternion.Euler(nullAngle);
            frontRight.transform.localRotation = Quaternion.Euler(nullAngle);
        }
    }
    void WheelsThrottleAnimation()
    {
        // TODO
    }
}