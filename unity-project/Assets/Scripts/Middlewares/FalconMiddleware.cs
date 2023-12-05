using System;
using UnityEngine;

public class FalconMiddleware : MonoBehaviour
{
    [SerializeField] private Vector3 position;
    [SerializeField] private Vector3 force;
    [SerializeField] private Vector3 rgb;

    public Vector3 Position
    {
        get
        {
            return position;
        }
        set
        {
            position = value;
        }
    }
    public Vector3 Force
    {
        get
        {
            return force;
        }
        set
        {
            force = value;
        }
    }
    public Vector3 Rgb
    {
        get
        {
            return rgb;
        }
        set
        {
            rgb = value;
        }
    }
    public event Action OnCenterButtonPress;
    public event Action OnLeftButtonPress;
    public event Action OnRightButtonPress;
    public event Action OnUpButtonPress;
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
