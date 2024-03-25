using System;
using System.Linq;
using UnityEngine;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

public class ConsoleRos2Node : MonoBehaviour{
    [Header("Console Settings")]
    [SerializeField] private string consoleTopicName = "console";
    private ROSConnection ros;

    [Header("Virtual Map")]
    [SerializeField] private GameObject wallPrefab = null;
    private System.Collections.Generic.List<GameObject> walls;

    void Start(){
        walls = new System.Collections.Generic.List<GameObject>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(consoleTopicName, ConsoleSubscriber);
    }

    void ConsoleSubscriber(StringMsg msg){
        string[] args = msg.data.Split(' ');
        if (args.Length > 1){
            if(args[0] == "virtual-map")
                VirtualMapOption(args);
        }
    }

    // virtual-map remove pos-x pos-y pos-z
    void VirtualMapOption(string[] args){
        if(args.Length >= 2){
            // virtual-map create pos-x pos-y pos-z quat-x quat-y quat-z quat-w x-scale y-scale z-scale
            if(args[1] == "create" && args.Length == 12){
                float posX, posY, posZ, quatX, quatY, quatZ, quatW, scaleX, scaleY, scaleZ;
                try{
                    IFormatProvider format = System.Globalization.CultureInfo.InvariantCulture.NumberFormat;
                    posX = float.Parse(args[2], format);
                    posY = float.Parse(args[3], format);
                    posZ = float.Parse(args[4], format);
                    quatX = float.Parse(args[5], format);
                    quatY = float.Parse(args[6], format);
                    quatZ = float.Parse(args[7], format);
                    quatW = float.Parse(args[8], format);
                    scaleX = float.Parse(args[9], format);
                    scaleY = float.Parse(args[10], format);
                    scaleZ = float.Parse(args[11], format);
                }
                catch{
                    return;
                }
                var position = Transformations.Ros2Unity(new PointMsg(posX, posY, posZ));
                var rotation = Transformations.Ros2Unity(new QuaternionMsg(quatX, quatY, quatZ, quatW));
                var scale = Transformations.Ros2Unity(new PointMsg(scaleX, scaleY, scaleZ));
                scale.x = Math.Abs(scale.x);
                scale.y = Math.Abs(scale.y);
                scale.z = Math.Abs(scale.z);
                VirtualMapAddWall(position, rotation, scale);
            }
            // virtual-map delete pos-x pos-y pos-z
            if(args[1] == "delete" && args.Length == 5){
                float posX, posY, posZ;
                try{
                    IFormatProvider format = System.Globalization.CultureInfo.InvariantCulture.NumberFormat;
                    posX = float.Parse(args[2], format);
                    posY = float.Parse(args[3], format);
                    posZ = float.Parse(args[4], format);
                }
                catch{
                    return;
                }
                var position = Transformations.Ros2Unity(new PointMsg(posX, posY, posZ));
                VirtualMapDeleteWall(position);
            }
            // virtual-map delete-all
            if(args[1] == "delete-all" && args.Length == 2){
                VirtualMapDeleteAllWalls();
            }
        }
    }

    private void VirtualMapAddWall(Vector3 position, Quaternion rotation, Vector3 scale){
        GameObject newWall = Instantiate(wallPrefab, position, rotation);
        newWall.transform.localScale = scale;
        walls.Add(newWall);
    }
    private void VirtualMapDeleteWall(Vector3 position){
        foreach (GameObject wall in walls.ToList()){
            bool isInXAxis = wall.transform.localPosition.x - wall.transform.localScale.x < position.x && wall.transform.localPosition.x + wall.transform.localScale.x > position.x;
            bool isInZAxis = wall.transform.localPosition.z - wall.transform.localScale.z < position.z && wall.transform.localPosition.z + wall.transform.localScale.z > position.z;
            if(isInXAxis && isInZAxis){
                walls.Remove(wall);
                Destroy(wall);
            }
        }
    }
    private void VirtualMapDeleteAllWalls(){
        foreach (GameObject wall in walls.ToList()){
            Destroy(wall);
        }
        walls.Clear();
    }
}
