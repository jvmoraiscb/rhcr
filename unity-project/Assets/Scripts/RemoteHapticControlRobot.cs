using UnityEngine;

// Remote Haptic Control Robot (RHCR) is the main class, all others classes just send or receive data from it
public class RemoteHapticControlRobot : MonoBehaviour
{
    [SerializeField] private MiddlewareFalcon falconMid;
    [SerializeField] private MiddlewareAckermann ackermannMid;
    [SerializeField] private VirtualCamera cam;

    [SerializeField] private float defaultSpeed = .6f;
    [SerializeField] private float maxSpeed = 1f;
    [SerializeField] private float minSpeed = .2f;

    private float speed;
    private bool isBreaking;

    private void Start()
    {
        speed = defaultSpeed;
        isBreaking = true;
        falconMid.OnCenterButtonPress += CenterButtonHandler;
        falconMid.OnLeftButtonPress += LeftButtonHandler;
        falconMid.OnRightButtonPress += RightButtonHandler;
        falconMid.OnUpButtonPress += UpButtonHandler;
    }

    private void Update()
    {
    if (isBreaking)
        {
            ackermannMid.throttle = 0f;
            ackermannMid.steer = 0f;

            falconMid.force.x = 0f;
            falconMid.force.y = 0f;
            falconMid.force.z = 0f;

            falconMid.rgb.x = 0;
            falconMid.rgb.y = 0;
            falconMid.rgb.z = 1;
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
                ackermannMid.throttle = falconMid.position.z * speed;

                auxSteer = Mathf.Abs(falconMid.position.x) > 0.3f ? falconMid.position.x : 0f;
                ackermannMid.steer = ackermannMid.throttle > 0f ? auxSteer * -1 : auxSteer;

                falconMid.force.x = 0f;
                falconMid.force.y = 0f;
                falconMid.force.z = 0f;

                falconMid.rgb.x = 0f;
                falconMid.rgb.y = 1f;
                falconMid.rgb.z = 0f;
            }
            
        }
        transform.SetPositionAndRotation(ackermannMid.position, ackermannMid.rotation);
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