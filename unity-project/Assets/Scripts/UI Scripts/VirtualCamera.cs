using UnityEngine;

public class VirtualCamera : MonoBehaviour
{
    [SerializeField]
    private Vector3 offsetCloseUp;
    [SerializeField]
    private Vector3 offsetTopDown;
    [SerializeField]
    private Transform target;
    [SerializeField]
    private float translateSpeed;
    [SerializeField]
    private float rotationSpeed;

    private Vector3 offset;

    public bool isCloseUpCam;

    private void Start()
    {
        isCloseUpCam = true;
    }

    private void FixedUpdate()
    {
        if (isCloseUpCam)
        {
            offset = offsetCloseUp;
        }
        else
        {
            offset = offsetTopDown;
        }
        HandleTranslation();
        HandleRotation();
    }

    private void HandleTranslation()
    {
        var targetPosition = target.TransformPoint(offset);
        transform.position = Vector3.Lerp(transform.position, targetPosition, translateSpeed * Time.deltaTime);
    }
    private void HandleRotation()
    {
        var direction = target.position - transform.position;
        var rotation = Quaternion.LookRotation(direction, Vector3.up);
        transform.rotation = Quaternion.Lerp(transform.rotation, rotation, rotationSpeed * Time.deltaTime);
    }
}