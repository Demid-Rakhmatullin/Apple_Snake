using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class CameraFollow2 : MonoBehaviour
{
    [SerializeField] Transform snakeHead;
    [SerializeField] Transform apple;
    [SerializeField] float maxMovementSpeed = 50f;

    private Vector3 cameraUp;

    void Start()
    {
        var direction = (snakeHead.position - apple.position).normalized;
        var position = apple.position + direction * 100f;

        transform.position = position;

        var dot1_abs = Mathf.Abs(Vector3.Dot(Vector3.up, direction));
        var dot2_abs = Mathf.Abs(Vector3.Dot(Vector3.forward, direction));
        var worldUp = dot1_abs < dot2_abs ? Vector3.up : Vector3.forward;

        transform.LookAt(snakeHead.position, worldUp);
        cameraUp = transform.up;
    }

    void LateUpdate()
    {
        //var rot90 = Quaternion.AngleAxis(90, Vector3.up) * Vector3.right;
        //Debug.DrawLine(apple.position, Vector3.right * 100f, Color.red);
        //Debug.DrawLine(apple.position, Vector3.forward * 100f, Color.blue);

        var currRight = transform.right;
        var currDirection = (transform.position - apple.position).normalized;
        var snakeDirection = (snakeHead.position - apple.position).normalized;
        var nextPosition = apple.position + snakeDirection * 100f;
        var adjustCamera = false;

        //var currDirection = (snakeHead.position - apple.position).normalized
        var proj = Vector3.ProjectOnPlane(snakeDirection, transform.right);
        var ang = Vector3.Angle(currDirection, proj);
        if (ang > 0.1f)
        {
            //cameraUp = Quaternion.AngleAxis(ang, transform.right) * cameraUp;
            adjustCamera = true;
            Debug.Log("ang: " + ang);
            //Debug.DrawLine(transform.position, transform.position + cameraUp * 30f, Color.red, 10f);
        }

        //transform.position = position; //todo smooth movement?
        transform.position = nextPosition;// Vector3.MoveTowards(transform.position, nextPosition, 
            //maxMovementSpeed * Time.deltaTime);

        //var snakeMovementNormal = Vector3.Cross(appleDir, transform.forward);
        var appleDir = (apple.position - transform.position).normalized;

        //var dot1_abs = Mathf.Abs(Vector3.Dot(Vector3.right, appleDir));
        //var dot2_abs = Mathf.Abs(Vector3.Dot(Vector3.forward, appleDir));
        //var current_right = dot1_abs < dot2_abs ? Vector3.right : Vector3.forward;
        //var proj = Vector3.ProjectOnPlane(appleDir, current_right);

        //var selected = dot1_abs < dot2_abs ? "right" : "forward";
        //Debug.Log("Selected: " + selected);
        ////var v = Mathf.Min(Mathf.Abs(dot1), Mathf.Abs(dot2));

        //var up = Vector3.Cross(current_right, appleDir);
        //Debug.DrawLine(apple.position, up * 100f, Color.green);
        //var dot = Vector3.Dot(Vector3.right, appleDir);
        //if (dot < -0.9f || dot > 0.9f)
        //{
        //    Debug.Log("Dot: " + up);
        //}

        //var dot1_abs = Mathf.Abs(Vector3.Dot(transform.forward, Vector3.up));
        //var dot2_abs = Mathf.Abs(Vector3.Dot(transform.forward, Vector3.right));
        //var camera_up = dot1_abs < dot2_abs ? Vector3.up : Vector3.right;

        //var camera_up = Vector3.Cross(appleDir, transform.right);
        //var camera_up = Vector3.Cross(transform.right, apple.up);

        //var dotUp = Vector3.Dot(transform.up, Vector3.up);
        //Debug.Log("dot up: " + dotUp);
        //var dotDown = Vector3.Dot(transform.up, -Vector3.up);
        //Debug.Log("dot down: " + dotDown);

        //if (Mathf.Abs(Vector3.Dot(transform.up, cameraUp)) < 0.4f)
        //{
        //    cameraUp = -cameraUp;
        //    Debug.Log("Change up");
        //}

        //Debug.Log("min angle: " + MinAngleWorldAxis(transform.up).Item2);
        //Debug.Log("dot up: " + Vector3.Dot(transform.up, Vector3.up) +
        //    ", dot down: " + Vector3.Dot(-transform.up, Vector3.down));

        //if (adjustCamera)
        //var lookRot = Quaternion.LookRotation(snakeHead.position - transform.position, Vector3.up);
        //transform.rotation = lookRot;
        transform.LookAt(snakeHead.position, transform.up);
    }

    private (Vector3, string) MinAngleWorldAxis(Vector3 currentAngle)
    {
        return axises.Select(a => new { axis = a, angle = Vector3.Dot(currentAngle, a.Item1) })
             .OrderByDescending(x => x.angle)
             .First().axis;
    }

    private (Vector3, string)[] axises = { 
        (Vector3.up, "up pos"), 
        (-Vector3.up, "up neg"),
        (Vector3.right, "right pos"),
        (-Vector3.right, "right neg"),
        (Vector3.forward, "forward pos"),
        (-Vector3.forward, "forward neg")
    };
}
