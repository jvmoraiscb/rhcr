using UnityEngine;

// Remote Haptic Control Robot (RHCR) is the main class, all others classes just send or receive data from it
public class RemoteHapticControlRobot : MonoBehaviour {
    [SerializeField] private FalconMiddleware falconMid;
    [SerializeField] private AckermannMiddleware ackermannMid;
    [SerializeField] private VirtualCamera cam;
    [SerializeField] private Transform[] frontSensors;
    [SerializeField] private Transform[] rearSensors;
    [SerializeField] private Transform[] rightSideSensors;
    [SerializeField] private Transform[] leftSideSensors;


    [SerializeField] private float defaultSpeed = .6f;
    [SerializeField] private float maxSpeed = 1f;
    [SerializeField] private float minSpeed = .2f;
    [SerializeField] private float maxFrontDistance = 0.5f;
    [SerializeField] private float maxSideDistance = 0.25f;
    [SerializeField] private float throttleForceFactor = 5f;
    // [SerializeField] private float steerForceFactor = 2f;

    private float speed;
    private bool isBreaking;
    private int frontDanger;
    private int rearDanger;
    private int leftSideDanger;
    private int rightSideDanger;

    private void Start(){
        speed = defaultSpeed;
        isBreaking = true;
        frontDanger = 0;
        rearDanger = 0;
        falconMid.OnCenterButtonPress += CenterButtonHandler;
        falconMid.OnLeftButtonPress += LeftButtonHandler;
        falconMid.OnRightButtonPress += RightButtonHandler;
        falconMid.OnUpButtonPress += UpButtonHandler;
    }

    private void Update()
    {
        frontDanger = CollisionHandler(frontSensors, maxFrontDistance, maxFrontDistance*2);
        rearDanger = CollisionHandler(rearSensors, maxFrontDistance, maxFrontDistance*2);
        leftSideDanger = CollisionHandler(leftSideSensors, maxSideDistance, maxSideDistance*2);
        rightSideDanger = CollisionHandler(rightSideSensors, maxSideDistance, maxSideDistance*2);
        if (isBreaking){
            ackermannMid.Throttle = 0f;
            ackermannMid.Steer = 0f;

            falconMid.Force = new Vector3(0f, 0f, 0f);

            falconMid.Rgb = new Vector3(0f, 0f, 1f);
        }
        else{
            float tempThrottle = falconMid.Position.z * speed;
            float tempSteer = tempThrottle > 0f ? (Mathf.Abs(falconMid.Position.x) > 0.3f ? falconMid.Position.x : 0f) * -1 : (Mathf.Abs(falconMid.Position.x) > 0.3f ? falconMid.Position.x : 0f);
            float tempZForce = 0f;
            float tempXForce = 0f;
            
            if(tempThrottle > 0){
                if(frontDanger == 2){
                    tempThrottle = 0f;
                    tempZForce = throttleForceFactor;
                }
                else if (frontDanger == 1){
                    tempThrottle /= 2;
                    tempZForce = throttleForceFactor/2;
                }
            }
            else{
                if(rearDanger == 2){
                    tempThrottle = 0f;
                    tempZForce = throttleForceFactor;
                }
                else if (rearDanger == 1){
                    tempThrottle /= 2;
                    tempZForce = throttleForceFactor/2;
                }
            }

            /*
            if(tempSteer > 0){
                if(rightSideDanger == 2){
                    tempSteer = 0f;
                    tempXForce = steerForceFactor;
                }
                else if(rightSideDanger == 1){
                    tempSteer /= 2;
                    tempXForce = steerForceFactor/2;
                }
            }
            else{
                if(rightSideDanger == 2){
                    tempSteer = 0f;
                    tempXForce = steerForceFactor;
                }
                else if(rightSideDanger == 1){
                    tempSteer /= 2;
                    tempXForce = steerForceFactor/2;
                }
            }
            */

            if(frontDanger == 2 || rearDanger == 2 || leftSideDanger == 2 || rightSideDanger == 2){
                falconMid.Rgb = new Vector3(1f, 0f, 0f);
            }
            else if(frontDanger == 1 || rearDanger == 1 || leftSideDanger == 1 || rightSideDanger == 1){
                falconMid.Rgb = new Vector3(1f, 1f, 0f);
            }
            else{
                falconMid.Rgb = new Vector3(0f, 1f, 0f);
            }

            falconMid.Force = new Vector3(tempXForce * falconMid.Position.x, 0f, tempZForce * falconMid.Position.z);
            ackermannMid.Throttle = tempThrottle;
            ackermannMid.Steer = tempSteer;
        }
        //transform.SetPositionAndRotation(ackermannMid.UnityPosition, ackermannMid.UnityRotation);
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

    private int CollisionHandler(Transform[] tfs, float redDistance, float yellowDistance){
        int countRed = 0;
        int countYellow = 0;
        foreach (Transform tf in tfs){
            int layerMask = 1 << 6;
            // rays always statrting from hokuyo fake in vWalker
            Vector3 rayOriginPos = tf.position;
            Vector3 rayOrigin = tf.eulerAngles;
            // Correcting the childs rotation in relation to the parent (90 degrees /1,57)
            float angle = (90 - rayOrigin.y) / (180 / Mathf.PI);
            Vector3 rayDirection = new Vector3 {
                x =  Mathf.Cos(angle),
                y = 0,
                z = Mathf.Sin(angle)
            };
            // if the ray collides, this distance information will be used to fill the ranges list
            // also, it can draw the lines from vWalker to the colision point. uncomment debug.drawline
            // it only collides with the Default mask (1)
            if (Physics.Raycast(rayOriginPos, rayDirection, out RaycastHit hit, yellowDistance, layerMask)){
                if(hit.distance < redDistance){
                    Debug.DrawLine(rayOriginPos, hit.point, Color.red);
                    countRed++;
                }
                else if(hit.distance < yellowDistance){
                    Debug.DrawLine(rayOriginPos, hit.point, Color.yellow);
                    countYellow++;
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