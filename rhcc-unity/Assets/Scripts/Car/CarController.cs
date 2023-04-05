using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{
    [SerializeField] private Rigidbody car;

    public float throttle;
    public float steer;
    public bool isBreaking;

    public float acceleration;

    private float lastAbsSpeed;
    private float lastTime;
    private int i;
    private float speed;
    private float currentSteerAngle;
    private float currentbreakForce;

    [SerializeField] private float motorForce;
    [SerializeField] private float breakForce;
    [SerializeField] private float maxSteerAngle;

    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheeTransform;
    [SerializeField] private Transform rearLeftWheelTransform;
    [SerializeField] private Transform rearRightWheelTransform;
    
    private void Start()
    {
        lastTime = Time.time;
        i = 0;
    }
    
    private void FixedUpdate()
    {
        speed = Mathf.Sqrt(Mathf.Pow(car.velocity.x, 2) + Mathf.Pow(car.velocity.y, 2) + Mathf.Pow(car.velocity.z,2));
        i += 1;

        HandleMotor();
        HandleSteering();
        UpdateWheels();

        if(i == 100){
            lastAbsSpeed = Mathf.Sqrt(Mathf.Pow(car.velocity.x, 2) + Mathf.Pow(car.velocity.y, 2) + Mathf.Pow(car.velocity.z,2));
            lastTime += Time.time;
            i = 0;

            acceleration = (speed - lastAbsSpeed) / (Time.time - lastTime);
        }

        Debug.Log(acceleration); 
    }

    private void HandleMotor()
    {
        rearLeftWheelCollider.motorTorque = throttle * motorForce;
        rearRightWheelCollider.motorTorque = throttle * motorForce;
        currentbreakForce = isBreaking ? breakForce : 0f;
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
        currentSteerAngle = maxSteerAngle * steer;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }

    private void UpdateWheels()
    {
        UpdateSingleWheel(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateSingleWheel(frontRightWheelCollider, frontRightWheeTransform);
        UpdateSingleWheel(rearRightWheelCollider, rearRightWheelTransform);
        UpdateSingleWheel(rearLeftWheelCollider, rearLeftWheelTransform);
    }

    private void UpdateSingleWheel(WheelCollider wheelCollider, Transform wheelTransform)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        wheelTransform.rotation = rot;
    }
}