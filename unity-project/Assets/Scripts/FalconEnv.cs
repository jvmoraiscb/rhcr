using UnityEngine;

public class FalconEnv : MonoBehaviour
{
    [SerializeField]
    private Ackermann ackermann;
    
    public Vector3 position;
    public Vector3 force;
    public Vector3 rgb;

    public void Center_button_handler() {
        ackermann.CenterButtonHandler();
    }
    public void Left_button_handler() {
        ackermann.LeftButtonHandler();
    }
    public void Right_button_handler() {
        ackermann.RightButtonHandler();
    }
    public void Up_button_handler() {
        ackermann.UpButtonHandler();
    }
}
