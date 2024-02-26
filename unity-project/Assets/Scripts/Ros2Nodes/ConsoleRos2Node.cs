using UnityEngine;
using ROS2;

public class ConsoleRos2Node : MonoBehaviour{
    [Header("ROS2 Constants")]
    [SerializeField] private string nodeName = "ConsoleNode_Unity";
    [SerializeField] private string minimapTopicName = "console";

    [Header("Virtual Map Constants")]
    [SerializeField] private GameObject wallPrefab = null;

    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    // virtual map variables
    private System.Collections.Generic.List<GameObject> walls;
}
