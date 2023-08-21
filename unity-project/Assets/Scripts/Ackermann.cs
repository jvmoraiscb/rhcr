using UnityEngine;

public class Ackermann : MonoBehaviour
{
    private Rigidbody rb;
    [SerializeField]
    private Falcon falcon;
    
    [SerializeField]
    private float rbVelocityFactor;
    [SerializeField]
    private float falconForceFactor;

    public float speed;
    public float maxSpeed;
    public float minSpeed;
    public float throttle;
    public float steer;
    public bool isBreaking;
    public bool isColliding;
    public bool isCollidingFront;

    public Vector3 position;
    public Quaternion rotation;

    private void Start()
    {
        speed = 0.5f;
        maxSpeed = 1.0f;
        minSpeed = 0.2f;

        isBreaking = true;
        isColliding = false;
        isCollidingFront = false;

        rb = GetComponent<Rigidbody>();    
    }

    private void FixedUpdate()
    {
        transform.rotation = rotation;
        rb.velocity = (position - transform.position) * rbVelocityFactor;
        
        if (isBreaking)
        {
            throttle = 0f;
            steer = 0f;

            falcon.force.x = 0f;
            falcon.force.y = 0f;
            falcon.force.z = 0f;

            falcon.rgb.x = 0;
            falcon.rgb.y = 0;
            falcon.rgb.z = 1;
        }
        else
        {
            if((isColliding && isCollidingFront && falcon.position.z > 0f) || (isColliding && !isCollidingFront && falcon.position.z < 0f))
            {
                throttle = 0f;
                steer = 0f;

                falcon.force.x = 0f;
                falcon.force.y = 0f;
                falcon.force.z = speed * falconForceFactor;

                falcon.rgb.x = 1f;
                falcon.rgb.y = 0f;
                falcon.rgb.z = 0f;
            }
            else
            {
                throttle = falcon.position.z * speed;
                steer = falcon.position.x;

                falcon.force.x = 0f;
                falcon.force.y = 0f;
                falcon.force.z = 0f;

                falcon.rgb.x = 0f;
                falcon.rgb.y = 1f;
                falcon.rgb.z = 0f;
            }
            
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        isColliding = true;
        if (throttle > 0f)
        {
            isCollidingFront = true;
        }
        else
        {
            isCollidingFront = false;
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        isColliding = false;
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