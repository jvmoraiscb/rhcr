using UnityEngine;

// Remote Haptic Control Robot (RHCR) is the main class, all others classes just send or receive data from it
public class RemoteHapticControlRobot : MonoBehaviour
{
    [SerializeField] private FalconMiddleware falconMid;
    [SerializeField] private AckermannMiddleware ackermannMid;
    [SerializeField] private VirtualCamera cam;
    [SerializeField] private Transform frontSensor;
    [SerializeField] private Transform rearSensor;

    [SerializeField] private float defaultSpeed = .6f;
    [SerializeField] private float maxSpeed = 1f;
    [SerializeField] private float minSpeed = .2f;

    private float speed;
    private bool isBreaking;
    private int frontDanger;
    private int rearDanger;
    private int leftDanger;
    private int rightDanger;

    private void Start(){
        speed = defaultSpeed;
        isBreaking = true;
        frontDanger = 0;
        falconMid.OnCenterButtonPress += CenterButtonHandler;
        falconMid.OnLeftButtonPress += LeftButtonHandler;
        falconMid.OnRightButtonPress += RightButtonHandler;
        falconMid.OnUpButtonPress += UpButtonHandler;
    }

    private void Update()
    {
        frontDanger = CollisionHandler(frontSensor, 30, 1f, 2f);
        rearDanger = CollisionHandler(rearSensor, 30, 1f, 2f);
        if (isBreaking){
            ackermannMid.Throttle = 0f;
            ackermannMid.Steer = 0f;

            falconMid.Force = new Vector3(0f, 0f, 0f);

            falconMid.Rgb = new Vector3(0f, 0f, 1f);
        }
        else{
            float forceFactor = 4f;
            float tempThrottle = falconMid.Position.z * speed;
            float tempSteer = tempThrottle > 0f ? (Mathf.Abs(falconMid.Position.x) > 0.3f ? falconMid.Position.x : 0f) * -1 : (Mathf.Abs(falconMid.Position.x) > 0.3f ? falconMid.Position.x : 0f);
            float tempZForce = 0f;
            tempThrottle = falconMid.Position.z * speed;
            if(tempThrottle > 0){
                if(frontDanger == 2){
                    tempThrottle = 0f;
                    tempZForce = forceFactor;
                }
                else if (frontDanger == 1){
                    tempThrottle /= 2;
                    tempZForce = forceFactor/2;
                }
            }
            else{
                if(rearDanger == 2){
                    tempThrottle = 0f;
                    tempZForce = forceFactor;
                }
                else if (rearDanger == 1){
                    tempThrottle /= 2;
                    tempZForce = forceFactor/2;
                }
            }
            falconMid.Force = new Vector3(0f, 0f, tempZForce * falconMid.Position.z);
            ackermannMid.Throttle = tempThrottle;
            ackermannMid.Steer = tempSteer;
        }
        transform.SetPositionAndRotation(ackermannMid.UnityPosition, ackermannMid.UnityRotation);
    }

    private void CenterButtonHandler()
    {
        isBreaking = !isBreaking;
    }
    private void LeftButtonHandler()
    {
        if (speed > minSpeed)
            speed -= 0.1f;
    }
    private void RightButtonHandler()
    {
        if (speed < maxSpeed)
            speed += 0.1f;
    }
    private void UpButtonHandler()
    {
        cam.isCloseUpCam = !cam.isCloseUpCam;
    }

    private int CollisionHandler(Transform tf, int openingAngle, float redDistance, float yellowDistance){
                int layerMask = 1 << 6;
        // rays always statrting from hokuyo fake in vWalker
        Vector3 rayOriginPos = tf.position;
        Vector3 rayOrigin = tf.eulerAngles;
        Vector3 rayDirection;
        int countRed = 0;
        int countYellow = 0;
        for (int i = 0; i < 360; i++){
            // Correcting the childs rotation in relation to the parent (90 degrees /1,57)
            float angle = (i - rayOrigin.y) / (180 / Mathf.PI);
            rayDirection.x = Mathf.Cos(angle);
            rayDirection.y = 0;
            rayDirection.z = Mathf.Sin(angle);
            // if the ray collides, this distance information will be used to fill the ranges list
            // also, it can draw the lines from vWalker to the colision point. uncomment debug.drawline
            // it only collides with the Default mask (1)
            if (i >= 90 - openingAngle/2 && i <= 90 + openingAngle/2){
                if (Physics.Raycast(rayOriginPos, rayDirection, out RaycastHit hit, 1000f, layerMask)){
                    if(hit.distance < redDistance){
                        Debug.DrawLine(rayOriginPos, hit.point, Color.red);
                        countRed++;
                    }
                    else if(hit.distance < yellowDistance){
                        Debug.DrawLine(rayOriginPos, hit.point, Color.yellow);
                        countYellow++;
                    }
                    else{
                        Debug.DrawLine(rayOriginPos, hit.point, Color.green);
                    }
                }
            }
        }
        if(countRed > 0)
            return 2;
        else if (countYellow > 0)
            return 1;
        else
            return 0;
    }
}