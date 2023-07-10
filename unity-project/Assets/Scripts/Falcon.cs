using UnityEngine;

public class Falcon : MonoBehaviour
{
    [SerializeField]
    private Ackermann ackermann;
    
    public Vector3 position;
    public Vector3 force;
    public Vector3 rgb;
    public void center_button_handler() {
        ackermann.isBreaking = !ackermann.isBreaking;
    }
    public void left_button_handler() {
        if(ackermann.speed > ackermann.minSpeed)
            ackermann.speed -= 0.1f;
    }
    public void right_button_handler() {
        if(ackermann.speed < ackermann.maxSpeed)
            ackermann.speed += 0.1f;
    }
    public void up_button_handler() { }

    private void Update()
    {
        if (ackermann.isBreaking)
        {
            rgb.x = 1;
            rgb.y = 0;
            rgb.z = 0;
        }
        else
        {
            rgb.x = 0;
            rgb.y = 1;
            rgb.z = 0;
        }
    }

}
