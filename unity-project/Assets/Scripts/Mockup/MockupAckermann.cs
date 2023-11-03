using UnityEngine;

public class MockupAckermann : MonoBehaviour
{
    [SerializeField] private AckermannEnv ackermannEnv;

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
        ackermannEnv.position = transform.position;
        ackermannEnv.rotation = transform.rotation;
    }

    private void HandleMotor()
    {
        rearLeftWheelCollider.motorTorque = ackermannEnv.throttle * motorForce;
        rearRightWheelCollider.motorTorque = ackermannEnv.throttle * motorForce;
        currentbreakForce = ackermannEnv.throttle == 0 ? breakForce : 0f;
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
        currentSteerAngle = ackermannEnv.throttle > 0 ? maxSteerAngle * ackermannEnv.steer * -1 : maxSteerAngle * ackermannEnv.steer;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }
}