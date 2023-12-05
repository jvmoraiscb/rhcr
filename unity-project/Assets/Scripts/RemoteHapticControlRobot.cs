using UnityEngine;

// Remote Haptic Control Robot (RHCR) is the main class, all others classes just send or receive data from it
public class RemoteHapticControlRobot : MonoBehaviour
{
    [SerializeField] private FalconMiddleware falconMid;
    [SerializeField] private AckermannMiddleware ackermannMid;
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
            ackermannMid.Throttle = 0f;
            ackermannMid.Steer = 0f;

            falconMid.Force = new Vector3(0f, 0f, 0f);

            falconMid.Rgb = new Vector3(0f, 0f, 1f);
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
                ackermannMid.Throttle = falconMid.Position.z * speed;

                auxSteer = Mathf.Abs(falconMid.Position.x) > 0.3f ? falconMid.Position.x : 0f;
                ackermannMid.Steer = ackermannMid.Throttle > 0f ? auxSteer * -1 : auxSteer;

                falconMid.Force = new Vector3(0f, 0f, 0f);

                falconMid.Rgb = new Vector3(0f, 1f, 0f);
            }
            
        }
        transform.SetPositionAndRotation(ackermannMid.UnityPosition, ackermannMid.UnityRotation);
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