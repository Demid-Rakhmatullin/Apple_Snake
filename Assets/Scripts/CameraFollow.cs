using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [SerializeField] private Transform cameraContainer;
    [SerializeField] private Transform apple;
    //использовать коллайдер или меш яблока?
    [SerializeField] private float appleRadius;
    [SerializeField] private Transform snake;
    private SnakeMovement snakeMovement;
    [SerializeField] private float verticalOffset;
    [SerializeField] private float initialOrientationAngle;

    [SerializeField] private DynamicJoystick joystick;

    private bool _init;
    private float _currentOrientationAngle;
    private Vector3 _snakeR;

    // Start is called before the first frame update
    void Start()
    {
        snakeMovement = snake.GetComponent<SnakeMovement>();

        _snakeR = snake.right;
        _currentOrientationAngle = -Vector3.Angle(snake.right, Vector3.right) + initialOrientationAngle;

        //followCamera.enabled = false;
        UpdateCamera(true, false);
    }

    void LateUpdate()
    {
        snakeMovement.AllowRotation = true;

        var joystickInput = joystick.Direction;

        if (!snakeMovement.Rotating)
        { 
            //вынести позицию и ориентацию в разные функции?
            UpdateCamera(true);

            //var rot = Quaternion.AngleAxis(5, cameraContainer.forward);
            //var ro = rot * cameraContainer.right;
            //var rot = Quaternion.AngleAxis(5, Vector3.up) * Vector3.forward;
            //rigidbody.rotation = rot * rigidbody.transform.rotation;
            //rigidbody.transform.Rotate(Vector3.up * 5f, Space.World);
            //targetRbRotation = targetRbRotation * rot;

            //var curr = Quaternion.AngleAxis(_currentOrientationAngle, cameraContainer.forward) * cameraContainer.right;
            //var rot = Quaternion.AngleAxis(5, cameraContainer.forward) * curr;
            //var a = Vector3.Angle(curr, rot);
            //_currentOrientationAngle += a;
            //Debug.Log(a);

            //Debug.Log(Vector3.Angle(_snakeR, snake.right));

            var angle = Vector3.Angle(_snakeR, snake.right);
            if (angle > 2f)
            {
                //_currentOrientationAngle = -Vector3.Angle(snake.right, Vector3.right) + initialOrientationAngle;

                _currentOrientationAngle -= angle;
                _snakeR = snake.right;
                snakeMovement.AllowRotation = false; //очень сомнительно, что это надо
                Debug.Log("fix camera");
            }
        }
        //else
            //UpdateCamera(true, false);

        //var ray = new Ray(apple.position, snake.position - apple.position);
        //var position = ray.direction * (23f + 40f);

        ////followCamera.transform.position = position;

        //followCamera.transform.position = Vector3.MoveTowards(followCamera.transform.position, position, 100f * Time.deltaTime);


        ////followCamera.transform.LookAt(apple, ray.direction);

        //var rot = Quaternion.LookRotation(apple.position - followCamera.transform.position, ray.direction);
        //followCamera.transform.rotation = Quaternion.RotateTowards(followCamera.transform.rotation,
        //    rot, 50f * Time.deltaTime);

        ////Debug.Log("position " + position);
    }

    Vector3 prevRot;

    private void UpdateCamera(bool instant = false, bool setOrientation = true)
    {
        //var ray = new Ray(apple.position, (snake.position - apple.position).normalized);
        //var position = ray.direction * (appleRadius + verticalOffset);
        var direction = (snake.position - apple.position).normalized;
        var position = apple.position + direction * (appleRadius + verticalOffset);
        
        //Debug.DrawRay(apple.position, position, Color.red, 1f);

        //followCamera.transform.position = position;
        if (instant)
            cameraContainer.position = position;
        else
            cameraContainer.position = Vector3.MoveTowards(cameraContainer.position, position, 50f * Time.deltaTime);


        //followCamera.transform.LookAt(apple, ray.direction);

        //var rot = Quaternion.LookRotation(apple.position - cameraContainer.position, ray.direction);

        //var rot = Quaternion.LookRotation(apple.position - cameraContainer.position, direction);

        //var up = (cameraContainer.position - apple.position).normalized;
        //var proj = Vector3.ProjectOnPlane(direction, cameraContainer.right);
        //var projLook = Vector3.ProjectOnPlane(-direction, cameraContainer.right);

        var snakeRot = snake.rotation.eulerAngles.y < 180f ? snake.rotation.eulerAngles.y : snake.rotation.eulerAngles.y - 180f;
        //var v = Quaternion.AngleAxis(snakeRot, Vector3.up) * Vector3.right;
        //Debug.Log($"snake rot: {snake.rotation.eulerAngles.y}, up vector: {v}");

        //var v = Mathf.Abs(Vector3.Dot(cameraContainer.forward, Vector3.right)) > 0.5

        //Debug.Log($"dot forward: {Vector3.Dot(cameraContainer.forward, Vector3.right)}, right: {Vector3.Dot(cameraContainer.right, Vector3.right)}, " +
        //    $"up: {Vector3.Dot(cameraContainer.up, Vector3.right)}");
        //Debug.Log($"dot forward: {Vector3.Dot(cameraContainer.forward, Vector3.right)}, -forward: {Vector3.Dot(cameraContainer.forward, -Vector3.right)}");


        //var rot = Quaternion.LookRotation(-direction, 
        //    Quaternion.AngleAxis(Vector3.Angle(snake.right, Vector3.right), -direction) * snake.right);
        //cameraContainer.rotation = rot * cameraContainer.rotation;

        //Debug.Log($"up: {cameraContainer.up}");

        //cameraContainer.rotation = Quaternion.Euler(cameraContainer.rotation.eulerAngles.x, cameraContainer.rotation.eulerAngles.y, 0f);

        //var rot = cameraContainer.rotation;
        ////var rot = Quaternion.Euler(cameraContainer.rotation.eulerAngles.x, cameraContainer.rotation.eulerAngles.y, 0f);

        //var projectRight = Vector3.ProjectOnPlane(-direction, cameraContainer.right);
        //rot = Quaternion.FromToRotation(cameraContainer.forward, projectRight) * rot;
        //var projectUp = Vector3.ProjectOnPlane(-direction, cameraContainer.up);
        //rot = Quaternion.FromToRotation(cameraContainer.forward, projectUp) * rot;

        //Debug.Log("rotation: " + rot.eulerAngles);

        //var rot = Quaternion.FromToRotation(cameraContainer.forward, -direction) * cameraContainer.rotation;
        //rot = Quaternion.FromToRotation(cameraContainer.right, Vector3.right) * rot;

        //Debug.Log(Quaternion.FromToRotation(cameraContainer.up, Vector3.forward).eulerAngles);
        //var ang1 = Mathf.Min(Vector3.Angle(cameraContainer.right, Vector3.right), Vector3.Angle(cameraContainer.right, -Vector3.right));
        //var ang2 = Mathf.Min(Vector3.Angle(cameraContainer.up, Vector3.forward), Vector3.Angle(cameraContainer.up, -Vector3.forward));
        //Debug.Log($"ang1: {ang1}, ang2: {ang2}");
        //if (ang1 < ang2)
        //cameraContainer.rotation = Quaternion.FromToRotation(cameraContainer.right, Vector3.right) * cameraContainer.rotation;

        //commented
        //cameraContainer.rotation = Quaternion.FromToRotation(Quaternion.AngleAxis(-snake.rotation.eulerAngles.y, cameraContainer.forward) * cameraContainer.right,
        //Quaternion.AngleAxis(snake.rotation.eulerAngles.y, Vector3.up) * Vector3.right) * cameraContainer.rotation;
        //---commented

        //var rot1 = Quaternion.FromToRotation(Quaternion.AngleAxis(-snake.rotation.eulerAngles.y, cameraContainer.forward) * cameraContainer.right,
        //    Quaternion.AngleAxis(snake.rotation.eulerAngles.y, Vector3.up) * Vector3.right);
        //var rot2 = Quaternion.FromToRotation(Quaternion.AngleAxis(-snake.rotation.eulerAngles.z, cameraContainer.forward) * cameraContainer.up,
        //    Quaternion.AngleAxis(snake.rotation.eulerAngles.z, Vector3.forward) * Vector3.up);
        //cameraContainer.rotation = rot1 * cameraContainer.rotation;

        //var ang1 = Vector3.Angle(snake.right, Vector3.right);
        //var ang2 = Vector3.Angle(snake.forward, Vector3.forward);
        //var ang = ang1;
        //Debug.Log($"ang {ang}, ang1 {ang1}, ang2 {ang2}");
        //Debug.Log($"ang {ang}, sum {snake.rotation.eulerAngles.y + snake.rotation.eulerAngles.z}, extr {snake.rotation.eulerAngles.y - snake.rotation.eulerAngles.z}");
        //cameraContainer.rotation = Quaternion.FromToRotation(Quaternion.AngleAxis(-ang, cameraContainer.forward) * cameraContainer.right,
        //    Quaternion.AngleAxis(ang, Vector3.up) * Vector3.right) * cameraContainer.rotation;
        //cameraContainer.rotation = Quaternion.FromToRotation(cameraContainer.right,
        //    Quaternion.AngleAxis(ang, -direction) * snake.right) * cameraContainer.rotation;
        //Vector3[] axises = { Vector3.up, Vector3.right, Vector3.forward };
        //var projA = axises.Select(a => Vector3.ProjectOnPlane(a, snake.right)).OrderByDescending(a => a.magnitude).First();

        //var proj = Vector3.ProjectOnPlane(snake.right, cameraContainer.forward).normalized;
        //var radiansX = Mathf.Atan2(snake.forward.y, -snake.forward.z);
        //var radiansZ = Mathf.Atan2(snake.right.y, -snake.right.x);

        //var t1 = Quaternion.AngleAxis(180 + radiansX * Mathf.Rad2Deg, Vector3.up) * Vector3.right;
        //var t2 = Quaternion.AngleAxis(180 + radiansZ * Mathf.Rad2Deg, Vector3.right) * t1;
        //Debug.Log($"x: {radiansX * Mathf.Rad2Deg}, z: {radiansZ * Mathf.Rad2Deg}, ang1: {ang1}");
        //Debug.Log($"res: {t2} y: {snake.localEulerAngles.y} z: {snake.localEulerAngles.z}");
        //cameraContainer.rotation = Quaternion.FromToRotation(Quaternion.AngleAxis(ang1, cameraContainer.forward) * cameraContainer.right,
        //    t2) * cameraContainer.rotation;


        //cameraContainer.rotation = Quaternion.FromToRotation(Quaternion.AngleAxis(-snake.rotation.eulerAngles.y, cameraContainer.forward) * cameraContainer.right,
        //Quaternion.AngleAxis(-snake.rotation.eulerAngles.y, snake.up) * snake.right) * cameraContainer.rotation;
        //else
        //cameraContainer.rotation = Quaternion.FromToRotation(cameraContainer.up, Vector3.forward) * cameraContainer.rotation;

        //var rot = cameraContainer.rotation;

        //commented
        if (setOrientation)
        {
            cameraContainer.rotation = Quaternion.FromToRotation(Quaternion.AngleAxis(_currentOrientationAngle, cameraContainer.forward) * cameraContainer.right,
                snake.right) * cameraContainer.rotation;
            //cameraContainer.rotation = Quaternion.RotateTowards(cameraContainer.rotation,
            //    Quaternion.FromToRotation(Quaternion.AngleAxis(_currentOrientationAngle, cameraContainer.forward) * cameraContainer.right,
            //    snake.right) * cameraContainer.rotation, 50f * Time.deltaTime);
            //cameraContainer.rotation = Quaternion.FromToRotation(Quaternion.AngleAxis(-Vector3.Angle(snake.right, Vector3.right), cameraContainer.forward) * cameraContainer.right,
            //    snake.right) * cameraContainer.rotation;

        }

        //commented
        cameraContainer.rotation = Quaternion.FromToRotation(cameraContainer.forward, -direction) * cameraContainer.rotation;
        //cameraContainer.rotation = Quaternion.RotateTowards(cameraContainer.rotation,
        //    Quaternion.FromToRotation(cameraContainer.forward, -direction) * cameraContainer.rotation, 50f * Time.deltaTime);

        //var vector = Quaternion.AngleAxis(90, Vector3.up) * cameraContainer.right;

        prevRot = cameraContainer.rotation.eulerAngles;
        //Debug.Log("prev rot: " + prevRot);
        // var rot = cameraContainer.rotation;
        //rot = Quaternion.Euler(cameraContainer.rotation.eulerAngles.x, cameraContainer.rotation.eulerAngles.y, 0f) * rot;
        //if (prevRot.z != 0f)
        //rot = Quaternion.AngleAxis(-prevRot.z, cameraContainer.forward) * rot;

        //var proj = Vector3.ProjectOnPlane(cameraContainer.right, direction).normalized;
        //var projRight = Vector3.ProjectOnPlane(Vector3.right, direction).normalized;
        //Debug.Log("right: " + cameraContainer.right + "; proj: " + proj);
        //if (proj != Vector3.right)
        //    Debug.Log("not eq " + proj);
        //else
        //    Debug.Log("equal ");
        //rot = Quaternion.FromToRotation(proj, projRight) * rot;
        //rot = Quaternion.FromToRotation(proj, new Vector3(1f, proj.y, 0f)) * rot;
        //rot = Quaternion.FromToRotation(cameraContainer.right, Vector3.right) * rot;

        //rot = Quaternion.FromToRotation(cameraContainer.forward, -direction) * rot;
        //var yRot = Quaternion.FromToRotation(cameraContainer.up

        //if (instant)
        //    cameraContainer.rotation = rot;
        //else
        //    cameraContainer.rotation = Quaternion.RotateTowards(cameraContainer.rotation,
        //        rot, 50f * Time.deltaTime);

        //Debug.Log("position " + position);
    }

    private Vector3 MinAngleWorldAxis(Vector3 currentAngle)
    {
        return axises.Select(a => new { axis = a, angle = Vector3.Dot(currentAngle, a) })
             .OrderByDescending(x => x.angle).First().axis;
    }

    private Vector3[] axises = { Vector3.up, -Vector3.up, Vector3.right, -Vector3.right, Vector3.forward, -Vector3.forward };
}
