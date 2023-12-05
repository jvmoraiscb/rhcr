using UnityEngine;

public class AckermannMockup : MonoBehaviour
{
    [SerializeField] private AckermannMiddleware ackermannMid;

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
        ackermannMid.UnityPosition = transform.position;
        ackermannMid.UnityRotation = transform.rotation;
    }

    private void HandleMotor()
    {
        rearLeftWheelCollider.motorTorque = ackermannMid.Throttle * motorForce;
        rearRightWheelCollider.motorTorque = ackermannMid.Throttle * motorForce;
        currentbreakForce = ackermannMid.Throttle == 0 ? breakForce : 0f;
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
        currentSteerAngle = ackermannMid.Throttle > 0 ? maxSteerAngle * ackermannMid.Steer * -1 : maxSteerAngle * ackermannMid.Steer;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }
}