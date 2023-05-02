using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionController : MonoBehaviour
{
    public Vector3 force;

    [SerializeField]
    private float maxForce;

    [SerializeField]
    private CarController car;

    private float finalForce;

    // Start is called before the first frame update
    void Start()
    {
        force.x = 0f;
        force.y = 0f;
        force.z = 0f;
    }

    // Update is called once per frame
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
