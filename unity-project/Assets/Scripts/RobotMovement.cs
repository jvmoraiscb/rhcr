using UnityEngine;
using System.Collections.Generic;
public class RobotMovement : MonoBehaviour {

    private struct Pose{
        public Vector3 position;
        public Quaternion rotation;
        public Pose(Vector3 pos, Quaternion rot){
            position = pos;
            rotation = rot;
        }
    }
    private struct PoseStamped{
        public Pose pose;
        public float time;
        public PoseStamped(Vector3 pos, Quaternion rot, float t){
            pose = new Pose(pos, rot);
            time = t;
        }
    }
    [SerializeField] private Transform robot;
    [SerializeField] float accumulatedTimeFactor = 1.5f;
    private Queue<PoseStamped> queue;
    bool canLerp;
    private Pose lastPose;
    private Pose nextPose;
    private float timeBetweenPoses;
    private float accumulatedTime;


    private void Start(){
        queue = new Queue<PoseStamped>();
        canLerp = false;
    }
    private void FixedUpdate(){
        if(!canLerp){
            if(queue.Count > 0){
                lastPose = new Pose(robot.position, robot.rotation);
                var localPose = queue.Dequeue();
                nextPose = new Pose(localPose.pose.position, localPose.pose.rotation);
                timeBetweenPoses = localPose.time;
                accumulatedTime = Time.fixedDeltaTime * accumulatedTimeFactor;
            }
            else{
                Debug.Log("queue empty");
                return;
            }
        }
        float lerpFactor = accumulatedTime < timeBetweenPoses ? accumulatedTime/timeBetweenPoses : 1f;
        accumulatedTime += Time.fixedDeltaTime;
        Debug.Log("lastPose: " + lastPose.position.ToString() + " nextPose: " + nextPose.position.ToString() + " " + accumulatedTime + " " + timeBetweenPoses + " " + queue.Count);
        Vector3 lerpPosition = Vector3.Lerp(lastPose.position, nextPose.position, lerpFactor);
        Quaternion lerpRotation = Quaternion.Lerp(lastPose.rotation, nextPose.rotation, lerpFactor);
        canLerp = lerpFactor < 1f;
        robot.SetPositionAndRotation(lerpPosition, lerpRotation);
    }

    public void Enqueue(Vector3 pos, Quaternion rot, float t){
        queue.Enqueue(new PoseStamped(pos, rot, t));
    }
}