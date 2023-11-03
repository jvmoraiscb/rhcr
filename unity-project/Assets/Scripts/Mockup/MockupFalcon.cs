using UnityEngine;

public class MockupFalcon : MonoBehaviour
{
    [SerializeField] private FalconEnv falconEnv;
    private bool lastStatus_right = true;
    private bool lastStatus_up = true;
    private bool lastStatus_center = true;
    private bool lastStatus_left = true;
    

    void Update()
    {
        falconEnv.position.x = Input.GetAxis("Horizontal");
        falconEnv.position.z = Input.GetAxis("Vertical");

        bool status_right = Input.GetKey(KeyCode.J);
        bool status_left = Input.GetKey(KeyCode.L);
        bool status_up = Input.GetKey(KeyCode.I);
        bool status_center = Input.GetKey(KeyCode.K);

        if (status_right && !lastStatus_right)
            falconEnv.RightButtonHandler();
        if (status_left && !lastStatus_left)
            falconEnv.LeftButtonHandler();
        if (status_up && !lastStatus_up)
            falconEnv.UpButtonHandler();
        if (status_center && !lastStatus_center)
            falconEnv.CenterButtonHandler();

        lastStatus_right = status_right;
        lastStatus_left = status_left;
        lastStatus_up = status_up;
        lastStatus_center = status_center;

    }
}
