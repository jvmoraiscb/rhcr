using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InputManager : MonoBehaviour
{
    [SerializeField]
    private CarController carController;
    
    // Update is called once per frame
    void Update()
    {
        carController.steer = Input.GetAxis("Horizontal");
        carController.throttle = Input.GetAxis("Vertical");
        carController.isBreaking = Input.GetKey(KeyCode.Space);       
    }
}
