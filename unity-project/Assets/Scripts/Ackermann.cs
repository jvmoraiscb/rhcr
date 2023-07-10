using UnityEngine;

public class Ackermann : MonoBehaviour
{
    [SerializeField]
    private Transform transform;
    
    public float speed = 0.5f;
    public float maxSpeed = 1.0f;
    public float minSpeed = 0.2f;
    public bool isBreaking = true;

    public Vector3 position;
    public Quaternion rotation;

    private void FixedUpdate()
    {
        transform.position = position;
        transform.localRotation = rotation;
    }
}
