using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

public class FalconRos2Node : MonoBehaviour
{
    [Header("ROS2")]
    [SerializeField] private float publisherFrequency = 10f;
    [SerializeField] private string positionTopicName = "falcon/position";
    [SerializeField] private string forceTopicName = "falcon/force";
    [SerializeField] private string rgbTopicName = "falcon/rgb";
    [SerializeField] private string rightButtonTopicName = "falcon/buttons/right";
    [SerializeField] private string upButtonTopicName = "falcon/buttons/up";
    [SerializeField] private string centerButtonTopicName = "falcon/buttons/center";
    [SerializeField] private string leftButtonTopicName = "falcon/buttons/left";
    private ROSConnection ros;
    private float timeElapsed;

    [Header("Falcon")]
    [SerializeField] private FalconMiddleware falconMid;
    private const int RIGHT = 1;
    private const int UP = 2;
    private const int CENTER = 3;
    private const int LEFT = 4;
    private int lastStatus_right = -1;
    private int lastStatus_up = -1;
    private int lastStatus_center = -1;
    private int lastStatus_left = -1;

    void Start(){
        timeElapsed = 0;
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Vector3Msg>(forceTopicName);
        ros.RegisterPublisher<Vector3Msg>(rgbTopicName);
        ros.Subscribe<Vector3Msg>(positionTopicName, PositionSubscriber);
        ros.Subscribe<BoolMsg>(centerButtonTopicName, CenterButtonSubscriber);
        ros.Subscribe<BoolMsg>(leftButtonTopicName, LeftButtonSubscriber);
        ros.Subscribe<BoolMsg>(rightButtonTopicName, RightButtonSubscriber);
        ros.Subscribe<BoolMsg>(upButtonTopicName, UpButtonSubscriber);
    }

    void Update(){
        timeElapsed += Time.deltaTime;

        if(timeElapsed > 1/publisherFrequency){
            ForcePublisher();
            RgbPublisher();
            timeElapsed = 0;
        }
    }

    void ForcePublisher(){
        var msg = new Vector3Msg{
            x = falconMid.Force.x,
            y = falconMid.Force.y,
            z = falconMid.Force.z
        };
        ros.Publish(forceTopicName, msg);
    }

    void RgbPublisher(){
        var msg = new Vector3Msg{
            x = falconMid.Rgb.x,
            y = falconMid.Rgb.y,
            z = falconMid.Rgb.z
        };
        ros.Publish(rgbTopicName, msg);
    }

    void PositionSubscriber(Vector3Msg msg){
        falconMid.Position = new Vector3((float)msg.x, (float)msg.y, (float)msg.z);
    }

    void CenterButtonSubscriber(BoolMsg msg){
        ButtonHandler(CENTER, msg.data);
    }
    void LeftButtonSubscriber(BoolMsg msg){
        ButtonHandler(LEFT, msg.data);
    }
    void RightButtonSubscriber(BoolMsg msg){
        ButtonHandler(RIGHT, msg.data);
    }
    void UpButtonSubscriber(BoolMsg msg){
        ButtonHandler(UP, msg.data);
    }

    void ButtonHandler(int button, bool b)
    {
        var value = b ? 1 : 0;
        if (button == RIGHT && value != lastStatus_right) {
            if (lastStatus_right != -1)
                falconMid.RightButtonHandler();
            lastStatus_right = value;
        }
        if (button == UP && value != lastStatus_up) {
            if (lastStatus_up != -1)
                falconMid.UpButtonHandler();
            lastStatus_up = value;
        }
        if (button == CENTER && value != lastStatus_center) {
            if (lastStatus_center != -1)
                falconMid.CenterButtonHandler();
            lastStatus_center = value;
        }
        if (button == LEFT && value != lastStatus_left) {
            if (lastStatus_left != -1)
                falconMid.LeftButtonHandler();
            lastStatus_left = value;
        }
    }
}
