using UnityEngine;

public class InputManager : MonoBehaviour
{
    [SerializeField]
    private CarController carController;
    
    void Update()
    {
        carController.steer = Input.GetAxis("Horizontal");
        carController.throttle = Input.GetAxis("Vertical");
        carController.isBreaking = Input.GetKey(KeyCode.Space);       
    }
}
