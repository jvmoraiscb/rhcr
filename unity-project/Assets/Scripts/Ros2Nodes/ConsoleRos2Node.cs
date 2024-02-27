using UnityEngine;
using ROS2;
using System;

public class ConsoleRos2Node : MonoBehaviour{
    [Header("ROS2 Constants")]
    [SerializeField] private string nodeName = "ConsoleNode_Unity";
    [SerializeField] private string consoleTopicName = "console";

    [Header("Virtual Map Constants")]
    [SerializeField] private GameObject wallPrefab = null;

    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    // virtual map variables
    private System.Collections.Generic.List<GameObject> walls;
    private Vector3 position;
    private Quaternion rotation;
    private Vector3 scale;
    private bool shouldAddWall = false;
    private bool shouldClear = false;

    void Start(){
        walls = new System.Collections.Generic.List<GameObject>();
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Node ??= ros2Unity.CreateNode(nodeName);
        if (ros2Unity.Ok()){
            ros2Node.CreateSubscription<std_msgs.msg.String>(consoleTopicName, msg => ConsoleHandler(msg));
        }
    }

    void Update(){
        if(shouldAddWall){
            GameObject newWall = Instantiate(wallPrefab, position, rotation);
            newWall.transform.localScale = scale;
            walls.Add(newWall);
            shouldAddWall = false;
        }
        if(shouldClear){
            foreach (GameObject wall in walls){
                Destroy(wall);
            }
            walls.Clear();
            shouldClear = false;
        }
    }

    void ConsoleHandler(std_msgs.msg.String msg){
        string[] args = msg.Data.Split(' ');
        if (args.Length > 1){
            if(args[0] == "virtual-map")
                VirtualMapOption(args);
        }
    }

    // virtual-map remove pos-x pos-y pos-z
    void VirtualMapOption(string[] args){
        if(args.Length >= 2){
            // virtual-map add pos-x pos-y pos-z quat-x quat-y quat-z quat-w x-scale y-scale z-scale
            if(args[1] == "add" && args.Length == 12){
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
                position = new Vector3(posX, posY, posZ);
                rotation = new Quaternion(quatX, quatY, quatZ, quatW);
                scale = new Vector3(scaleX, scaleY, scaleZ);
                shouldAddWall = true;
            }
            // virtual-map clear
            if(args[1] == "clear" && args.Length == 2){
                shouldClear = true;
            }
        }
    }
}
