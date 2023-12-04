using UnityEngine;

public class MockupFalcon : MonoBehaviour
{
    [SerializeField] private MiddlewareFalcon falconMid;
    private bool lastStatus_right = true;
    private bool lastStatus_up = true;
    private bool lastStatus_center = true;
    private bool lastStatus_left = true;
    

    void Update()
    {
        falconMid.position.x = Input.GetAxis("Horizontal");
        falconMid.position.z = Input.GetAxis("Vertical");

        bool status_right = Input.GetKey(KeyCode.L);
        bool status_left = Input.GetKey(KeyCode.J);
        bool status_up = Input.GetKey(KeyCode.I);
        bool status_center = Input.GetKey(KeyCode.K);

        if (status_right && !lastStatus_right)
            falconMid.RightButtonHandler();
        if (status_left && !lastStatus_left)
            falconMid.LeftButtonHandler();
        if (status_up && !lastStatus_up)
            falconMid.UpButtonHandler();
        if (status_center && !lastStatus_center)
            falconMid.CenterButtonHandler();

        lastStatus_right = status_right;
        lastStatus_left = status_left;
        lastStatus_up = status_up;
        lastStatus_center = status_center;

    }
}
