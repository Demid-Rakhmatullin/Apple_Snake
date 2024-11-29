using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SnakeController : MonoBehaviour
{
    [SerializeField] private DynamicJoystick joystick;
    //[SerializeField] private Transform snake;

    public Vector3 Input { get; private set; }

    // Update is called once per frame
    void Update()
    {
        var joystickInput = joystick.Direction;
        //Debug.Log("joystick dir: " + joystick.Direction + ", hor: " + joystick.Horizontal + ", vert: " + joystick.Vertical);
        Input = Camera.main.transform.TransformDirection(joystick.Horizontal, joystick.Vertical, 0f);
        //Debug.DrawLine(transform.position, transform.position + Input * 5f);
    }
}
