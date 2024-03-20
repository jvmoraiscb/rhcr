using ROS2;
using UnityEngine;
using UnityEngine.UI;
public class CamSensorRos2Node : MonoBehaviour{
    [Header("ROS2 Constants")]
    [SerializeField] private string nodeName = "ImageNode_Unity";
    [SerializeField] private string imageTopicName = "image_raw";

    [Header("Image Constants")]
    // [SerializeField] private RawImage rawImage;
    // ros2 variables
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;

    // image variables
    public MeshRenderer meshRenderer;
    private Texture2D texture2D;
    private bool msgReceived;
    private sensor_msgs.msg.CompressedImage msg;

    void Start(){
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Node ??= ros2Unity.CreateNode(nodeName);
        if (ros2Unity.Ok()){
            ros2Node.CreateSubscription<sensor_msgs.msg.CompressedImage>(imageTopicName, msg => ImageHandler(msg));
        }
        msgReceived = false;
        texture2D = new Texture2D(1,1);
        meshRenderer.material = new Material(Shader.Find("Standard"));
        
    }
    void Update(){
        if(msgReceived){
            //BgrToRgb(msg.Data);
            texture2D.LoadImage(msg.Data);
            texture2D.Apply();
            meshRenderer.material.SetTexture("_MainTex", texture2D);
            msgReceived = false;
        }
    }
    void ImageHandler(sensor_msgs.msg.CompressedImage msg){
        this.msg = msg;
        msgReceived = true;
    }

    void BgrToRgb(byte[] data){
        for (int i = 0; i < data.Length; i += 3){
            (data[i + 2], data[i]) = (data[i], data[i + 2]);
        }
    }
}
