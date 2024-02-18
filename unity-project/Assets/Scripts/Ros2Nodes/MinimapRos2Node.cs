using UnityEngine;
using ROS2;

public class MinimapRos2Node : MonoBehaviour
{
    [Header("ROS2 Constants")]
    [SerializeField] private string nodeName = "MinimapNode_Unity";
    [SerializeField] private string minimapTopicName = "minimap";

    [Header("Minimap Constants")]
    [SerializeField] private GameObject wallPrefab = null;
    [SerializeField] private float yAxis = 0f;

    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    // minimap variables
    private System.Collections.Generic.List<GameObject> walls;
    private nav_msgs.msg.OccupancyGrid lastMinimap = null;

    void Start(){
        walls = new System.Collections.Generic.List<GameObject>();
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Node ??= ros2Unity.CreateNode(nodeName);
        if (ros2Unity.Ok()){
            ros2Node.CreateSubscription<nav_msgs.msg.OccupancyGrid>(minimapTopicName, msg => MinimapHandler(msg));
        }
    }

    void Update(){
        if (ros2Unity.Ok()){
            MinimapUpdate();
        }
    }

    void MinimapHandler(nav_msgs.msg.OccupancyGrid msg){
        lastMinimap = msg;
    }

    void MinimapUpdate(){
        if(lastMinimap == null) return;
        foreach (GameObject wall in walls){
            Destroy(wall);
        }
        walls.Clear();
        for (uint i = 0; i < lastMinimap.Info.Height; i++){
            for (uint j = 0; j < lastMinimap.Info.Width; j++){
                if(lastMinimap.Data[i * lastMinimap.Info.Width + j] == 100){
                    Vector3 ros_position = new Vector3(j*lastMinimap.Info.Resolution + (float)lastMinimap.Info.Origin.Position.X, i*lastMinimap.Info.Resolution + (float)lastMinimap.Info.Origin.Position.Y, yAxis);
                    walls.Add(Instantiate(wallPrefab, ros_position.Ros2Unity(), new Quaternion(0, 0, 0, 1)));
                }
            }
        }
    }
}
