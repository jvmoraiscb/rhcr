using UnityEngine;

public class MockupAckermann : MonoBehaviour
{
    [SerializeField] private MiddlewareAckermann ackermannMid;

    [SerializeField] private Rigidbody car;

    [SerializeField] private float motorForce;
    [SerializeField] private float breakForce;
    [SerializeField] private float maxSteerAngle;

    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    private float currentSteerAngle;
    public float currentbreakForce;

    private void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        ackermannMid.position = transform.position;
        ackermannMid.rotation = transform.rotation;
    }

    private void HandleMotor()
    {
        rearLeftWheelCollider.motorTorque = ackermannMid.throttle * motorForce;
        rearRightWheelCollider.motorTorque = ackermannMid.throttle * motorForce;
        currentbreakForce = ackermannMid.throttle == 0 ? breakForce : 0f;
        ApplyBreaking();
    }

    private void ApplyBreaking()
    {
        frontRightWheelCollider.brakeTorque = currentbreakForce;
        frontLeftWheelCollider.brakeTorque = currentbreakForce;
        rearLeftWheelCollider.brakeTorque = currentbreakForce;
        rearRightWheelCollider.brakeTorque = currentbreakForce;
    }

    private void HandleSteering()
    {
        currentSteerAngle = ackermannMid.throttle > 0 ? maxSteerAngle * ackermannMid.steer * -1 : maxSteerAngle * ackermannMid.steer;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }
}