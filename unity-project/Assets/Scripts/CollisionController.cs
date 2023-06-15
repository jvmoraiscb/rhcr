using UnityEngine;

public class CollisionController : MonoBehaviour
{
    public Vector3 force;

    [SerializeField]
    private float maxForce;

    [SerializeField]
    private CarController car;

    private float finalForce;

    void Start()
    {
        force.x = 0f;
        force.y = 0f;
        force.z = 0f;
    }

    void Update()
    {
        finalForce = maxForce - car.speed;    

        if(car.isColliding && !car.isBreaking && ((car.isCollidingFront && car.throttle > 0f) || (!car.isCollidingFront && car.throttle < 0f))){
            force.z = car.throttle * finalForce;
        }
        else{
            force.z = 0f;
        }
    }
}
