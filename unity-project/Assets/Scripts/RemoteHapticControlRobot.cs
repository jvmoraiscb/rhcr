using UnityEngine;

// Remote Haptic Control Robot (RHCR) is the main class, all others classes just send or receive data from it
public class RemoteHapticControlRobot : MonoBehaviour
{
    [SerializeField]
    private FalconEnv falconEnv;
    [SerializeField]
    private AckermannEnv ackermannEnv;
    [SerializeField]
    private CameraFollow cam;

    [SerializeField]
    private float defaultSpeed = .6f;
    [SerializeField]
    private float maxSpeed = 1f;
    [SerializeField]
    private float minSpeed = .2f;

    [SerializeField]
    private float speed;
    [SerializeField]
    private bool isBreaking;

    private void Start()
    {
        speed = defaultSpeed;
        isBreaking = true;
        falconEnv.OnCenterButtonPress += CenterButtonHandler;
        falconEnv.OnLeftButtonPress += LeftButtonHandler;
        falconEnv.OnRightButtonPress += RightButtonHandler;
        falconEnv.OnUpButtonPress += UpButtonHandler;
    }

    private void Update()
    {
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
            float auxSteer;
            // TODO: new collisions
            if (false)
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
        // transform.SetPositionAndRotation(ackermannEnv.position, ackermannEnv.rotation);
    }

    private void CenterButtonHandler()
    {
        isBreaking = !isBreaking;
    }
    private void LeftButtonHandler()
    {
        if (speed > minSpeed)
            speed -= 0.1f;
    }
    private void RightButtonHandler()
    {
        if (speed < maxSpeed)
            speed += 0.1f;
    }
    private void UpButtonHandler()
    {
        cam.isCloseUpCam = !cam.isCloseUpCam;
    }
}