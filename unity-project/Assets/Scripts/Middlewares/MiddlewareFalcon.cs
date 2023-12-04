using System;
using UnityEngine;

public class MiddlewareFalcon : MonoBehaviour
{
    public event Action OnCenterButtonPress;
    public event Action OnLeftButtonPress;
    public event Action OnRightButtonPress;
    public event Action OnUpButtonPress;

    public Vector3 position;
    public Vector3 force;
    public Vector3 rgb;

    public void CenterButtonHandler() {
        OnCenterButtonPress?.Invoke();
    }
    public void LeftButtonHandler() {
        OnLeftButtonPress?.Invoke();
    }
    public void RightButtonHandler() {
        OnRightButtonPress?.Invoke();
    }
    public void UpButtonHandler() {
        OnUpButtonPress?.Invoke();
    }
}
