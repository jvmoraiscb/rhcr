using UnityEngine;

public class Ackermann : MonoBehaviour
{
    private Rigidbody rb;
    [SerializeField]
    private float velocityFactor;
    
    public float speed = 0.5f;
    public float maxSpeed = 1.0f;
    public float minSpeed = 0.2f;
    public bool isBreaking = true;

    public Vector3 position;
    public Quaternion rotation;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();    
    }

    private void FixedUpdate()
    {
        transform.rotation = rotation;

        rb.velocity = (position - transform.position) * velocityFactor;
    }
}