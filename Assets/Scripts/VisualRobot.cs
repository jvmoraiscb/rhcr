using UnityEngine;
public class VisualRobot : MonoBehaviour {
    [SerializeField] private Transform robot;
    [SerializeField] private float lerpSpeed = 10f;
    [SerializeField] private int throttleAngleSpeed = 360;
    [SerializeField] private float steerMaxAngle = 30;
    [SerializeField] private AckermannMiddleware ackermannMid;
    [SerializeField] private GameObject frontRight;
    [SerializeField] private GameObject frontLeft;
    [SerializeField] private GameObject rearRight;
    [SerializeField] private GameObject rearLeft;
    private void FixedUpdate(){
        var lerpPosition = Vector3.Lerp(transform.position, robot.position, lerpSpeed * Time.deltaTime);
        var lerpRotation = Quaternion.Lerp(transform.rotation, robot.rotation, lerpSpeed * Time.deltaTime);
        transform.SetPositionAndRotation(lerpPosition, lerpRotation);
        ThrottleAnimation();
        SteerAnimation();
    }

    private void ThrottleAnimation(){
        var throttleRotation = new Vector3 (throttleAngleSpeed * ackermannMid.Throttle * Time.deltaTime, 0f, 0f);
        frontRight.transform.Rotate(throttleRotation);
        rearRight.transform.Rotate(throttleRotation);
        frontLeft.transform.Rotate(throttleRotation);
        rearLeft.transform.Rotate(throttleRotation);
    }

    private void SteerAnimation(){
        var frontRightRotation = new Vector3(frontRight.transform.rotation.eulerAngles.x, ackermannMid.Throttle > 0f ? -1 * ackermannMid.Steer * steerMaxAngle : ackermannMid.Steer * steerMaxAngle, frontRight.transform.rotation.eulerAngles.z);
        var frontLeftRotation = new Vector3(frontLeft.transform.rotation.eulerAngles.x, ackermannMid.Throttle > 0f ? -1 * ackermannMid.Steer * steerMaxAngle : ackermannMid.Steer * steerMaxAngle, frontLeft.transform.rotation.eulerAngles.z);
        frontRight.transform.localRotation = Quaternion.Euler(frontRightRotation);
        frontLeft.transform.localRotation = Quaternion.Euler(frontLeftRotation);
    }
}