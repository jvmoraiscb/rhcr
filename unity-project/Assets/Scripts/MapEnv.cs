using UnityEngine;

public class MapEnv : MonoBehaviour
{
    private class LaserScanClass : MonoBehaviour
    {
        public Vector3 robotAngle;
        public float range_max;
        public float range_min;
        public float angle_min;
        public float angle_max;
        public float angle_increment;
        public float[] ranges;
        public Vector3[] directions;
        public Vector3 directionsNew;
    }

    private LaserScanClass robotLaser;
    private LaserScanClass virtualLaser;
    
    [SerializeField]
    private GameObject prefab;
    private GameObject destruidor;
    [SerializeField]
    private float range_min_filter = (float)0.15;
    [SerializeField]
    private float range_max_filter = (float)1.5;
    [SerializeField]
    private float tempoParaDestruir = (float)0.2;


    // Update is called once per frame
    void Update()
    {
        RobotLaserUpdate();
    }

    void RobotLaserUpdate()
    {
        for (int i = 0; i < robotLaser.ranges.Length; i++)
        {
            // if the ranges are between the lower and upper limits
            if (robotLaser.ranges[i] > range_min_filter)
            {
                if (robotLaser.ranges[i] < range_max_filter)
                {
                    // transform the scan topic so it can have the walker reference and corrected angles
                    Vector3 vectorAux = new Vector3(-Mathf.Cos(robotLaser.angle_min + robotLaser.angle_increment * i - (robotLaser.robotAngle.y * Mathf.PI / 180)), -Mathf.Sin(robotLaser.angle_min + robotLaser.angle_increment * i - (robotLaser.robotAngle.y * Mathf.PI / 180)), 0);
                    robotLaser.directions[i] = new Vector3(vectorAux.y, vectorAux.z, vectorAux.x); // convert Ros to Unity 
                    robotLaser.directionsNew = new Vector3(robotLaser.directions[i].x * 10 * robotLaser.ranges[i] + transform.position.x, transform.position.y, robotLaser.directions[i].z * 10 * robotLaser.ranges[i] + transform.position.z);
                    // Instatiate a prefab to warn the user that that is a possible colision in the real env
                    destruidor = Instantiate(prefab, new Vector3(robotLaser.directionsNew.x, robotLaser.directionsNew.y, robotLaser.directionsNew.z), new Quaternion(0, 0, 0, 1));
                    // destroy this game object after 0.3 seconds so it doesnt flood the scene
                    Destroy(destruidor, tempoParaDestruir);
                    //Debug.DrawLine(transform.position, directionsNew);
                    //Debug.Log(directionsNew);
                }

            }

        }
    }

    void VirtualLaserUpdate()
    {

    }

    public void SetRobotLaser(Vector3 robotAngle, float[] ranges, float range_max, float range_min, float angle_max, float angle_min, float angle_increment)
    {
        robotLaser.robotAngle = robotAngle;
        robotLaser.ranges = ranges;
        robotLaser.range_max = range_max;
        robotLaser.range_min = range_min;
        robotLaser.angle_min = angle_min;
        robotLaser.angle_max = angle_max;
        robotLaser.angle_increment = angle_increment;
    }
}
