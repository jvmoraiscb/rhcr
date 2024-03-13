using UnityEngine;

public class VirtualCamera : MonoBehaviour
{   
    [SerializeField] private Vector3 offsetCloseUp;
    [SerializeField] private Vector3 offsetTopDown;
    [SerializeField] private Transform target;
    [SerializeField] private float translateSpeed;
    [SerializeField] private float rotationSpeed;

    private Vector3 offset;

    public bool isCloseUpCam = true;

    private void Update()
    {
        offset = isCloseUpCam ? offsetCloseUp : offsetTopDown;
        HandleTranslation();
        HandleRotation();
    }

    private void HandleTranslation(){
        var targetPosition = target.TransformPoint(offset);
        transform.position = Vector3.Lerp(transform.position, targetPosition, translateSpeed * Time.deltaTime);
    }
    private void HandleRotation(){
        var direction = target.position - transform.position;
        direction.y = isCloseUpCam ? 0f : direction.y;
        var rotation = Quaternion.LookRotation(direction, Vector3.up);
        transform.rotation = Quaternion.Lerp(transform.rotation, rotation, rotationSpeed * Time.deltaTime);
    }
}