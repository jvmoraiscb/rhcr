using UnityEngine;

// Remote Haptic Control Robot (RHCR) is the main class, all others classes just send or receive data from it
public class RemoteHapticControlRobot : MonoBehaviour
{
    [SerializeField]
    private FalconEnv falconEnv;
    [SerializeField]
    private AckermannEnv ackermannEnv;
    [SerializeField]
    private WheelsAnimation wheelsAnim;
    [SerializeField]
    private Camera closeUpCam;
    [SerializeField]
    private Camera topDownCam;

    [SerializeField]
    private float defaultSpeed;
    [SerializeField]
    private float maxSpeed;
    [SerializeField]
    private float minSpeed;

    private float speed;
    private bool isBreaking;

    private void Start()
    {
        closeUpCam.enabled = true;
        topDownCam.enabled = false;
        speed = defaultSpeed;
        isBreaking = true;
        falconEnv.OnCenterButtonPress += CenterButtonHandler;
        falconEnv.OnLeftButtonPress += LeftButtonHandler;
        falconEnv.OnRightButtonPress += RightButtonHandler;
        falconEnv.OnUpButtonPress += UpButtonHandler;
    }

    private void Update()
    {
        float auxSteer = 0f;

        if (isBreaking)
        {
            ackermannEnv.throttle = 0f;
            ackermannEnv.steer = 0f;

            falconEnv.force.x = 0f;
            falconEnv.force.y = 0f;
            falconEnv.force.z = 0f;

            falconEnv.rgb.x = 0;
            falconEnv.rgb.y = 0;
            falconEnv.rgb.z = 1;
        }
        else
        {
            // TODO: new collisions
            if(false)
            {
                // TODO: we need collisions?
            }
            else
            {
                ackermannEnv.throttle = falconEnv.position.z * speed;

                auxSteer = Mathf.Abs(falconEnv.position.x) > 0.3f ? falconEnv.position.x : 0f;
                ackermannEnv.steer = ackermannEnv.throttle > 0f ? auxSteer * -1 : auxSteer;

                falconEnv.force.x = 0f;
                falconEnv.force.y = 0f;
                falconEnv.force.z = 0f;

                falconEnv.rgb.x = 0f;
                falconEnv.rgb.y = 1f;
                falconEnv.rgb.z = 0f;
            }
            
        }

        wheelsAnim.wheelsSteer = ackermannEnv.throttle != 0f ? auxSteer : 0f;
    }

    private void FixedUpdate()
    {
        transform.SetPositionAndRotation(ackermannEnv.position, ackermannEnv.rotation);
    }

    public void CenterButtonHandler()
    {
        isBreaking = !isBreaking;
    }
    public void LeftButtonHandler()
    {
        if (speed > minSpeed)
            speed -= 0.1f;
    }
    public void RightButtonHandler()
    {
        if (speed < maxSpeed)
            speed += 0.1f;
    }
    public void UpButtonHandler()
    {
        // TODO: change camera view
    }
}