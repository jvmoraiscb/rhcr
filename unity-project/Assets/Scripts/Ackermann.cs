using UnityEngine;

// Ackermann is the main class, all others classes just send or receive data from it
public class Ackermann : MonoBehaviour
{
    [SerializeField]
    private Rigidbody rb;
    [SerializeField]
    private FalconEnv falconEnv;
    [SerializeField]
    private AckermannEnv ackermannEnv;
    
    [SerializeField]
    private float rbVelocityFactor;
    [SerializeField]
    private float falconForceFactor;

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
        speed = defaultSpeed;
        
        isBreaking = true;
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
            // TODO: fazer com seis colisores
            if(false)
            {

            }
            else
            {
                ackermannEnv.throttle = falconEnv.position.z * speed;

                float auxSteer = Mathf.Abs(falconEnv.position.x) > 0.3f ? falconEnv.position.x : 0f;
                ackermannEnv.steer = ackermannEnv.throttle > 0f ? auxSteer * -1 : auxSteer;

                falconEnv.force.x = 0f;
                falconEnv.force.y = 0f;
                falconEnv.force.z = 0f;

                falconEnv.rgb.x = 0f;
                falconEnv.rgb.y = 1f;
                falconEnv.rgb.z = 0f;
            }
            
        }
    }

    private void FixedUpdate()
    {
        transform.rotation = ackermannEnv.rotation;
        rb.velocity = (ackermannEnv.position - transform.position) * rbVelocityFactor;
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

    }
}