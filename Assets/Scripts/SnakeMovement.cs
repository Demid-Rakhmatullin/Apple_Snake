using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SnakeMovement : MonoBehaviour
{
    [SerializeField] private Collider gravityCollider;
    /*[SerializeField]*/ private new Rigidbody rigidbody;
    [SerializeField] private CharacterController characterController;
    [SerializeField] private Transform mesh;
    [SerializeField] private float speed;
    [SerializeField] private float rotationSpeed;
    [SerializeField] private DynamicJoystick joystick;
    [SerializeField] private new Transform camera;

    // Start is called before the first frame update
    void Start()
    {
        rigidbody = GetComponent<Rigidbody>();
    }

    private Vector3 velocity;
    private float velocityZ;
    private Vector2 joystickInput;
    Quaternion inputRotation = Quaternion.identity;

    // Update is called once per frame
    void Update()
    {
        joystickInput = joystick.Direction;

        //if (joystickInput != Vector2.zero)
        //{
        //    var inputRot = Quaternion.FromToRotation(rigidbody.transform.forward, new Vector3(joystickInput.x, 0f, joystickInput.y));
        //    rigidbody.rotation = Quaternion.RotateTowards(rigidbody.rotation, inputRot * rigidbody.rotation, angularSpeed * Time.deltaTime);
        //    //inputRotation = Quaternion.RotateTowards(rigidbody.rotation, inputRot * rigidbody.rotation, angularSpeed * Time.deltaTime);
        //    //Debug.Log($"input: {joystickInput}");
        //}
        //else
        //    inputRotation = Quaternion.identity;


        //var center = gravityCollider.bounds.center;
        //var direction = (center - transform.position).normalized;
        var grDir = (gravityCollider.transform.position - transform.position).normalized;


        //if (characterController.isGrounded && playerVelocity != Vector3.zero)
        //{
        //    playerVelocity = Vector3.zero;
        //    Debug.Log("grounded: " + characterController.isGrounded);
        //    //playerVelocity.y = 0f;
        //    //playerVelocity.y = -9.81f * Time.deltaTime;
        //}

        if (Physics.Raycast(transform.position, -transform.up, out RaycastHit hit))
        {
            //correct
            //var slopeRotation = Quaternion.FromToRotation(transform.up, hit.normal);
            //
            //var excludeY = Quaternion.Euler(slopeRotation.eulerAngles.x, 0f, slopeRotation.eulerAngles.z);
            //var slerped = Quaternion.Slerp(transform.rotation, slopeRotation, 10f * Time.deltaTime);

            //correct
            //transform.rotation = slopeRotation * transform.rotation;
            //

            //var zazor = hit.distance - (characterController.height / 2f);
            //if (zazor > 0.1f)
            //{
            //    transform.Translate(hit.point);
            //    //characterController.Move(10f * Time.deltaTime * -transform.up);
            //    //transform.position = transform.position - Vector3.up * zazor;
            //    Debug.Log("y moved");
            //}
            //transform.rotation = Quaternion.Slerp(transform.rotation, slopeRotation, 5f * Time.deltaTime);
            //transform.up = info.normal;
            //Debug.Log(/*"normal: " + hit.normal + */", object: " + hit.collider.name);

        }

        //var dir = speed * transform.forward + (-hit.normal * 9.8f);
        //characterController.Move(dir * Time.deltaTime);

        //characterController.Move(speed * Time.deltaTime * transform.forward);

        //var dest = speed * Time.deltaTime * transform.forward;
        //var check = Physics.OverlapSphere(dest, characterController.height / 2f + 0.1f);
        //while (!check.Any(c => c.name == "Apple"))
        //{
        //    dest = Vector3.Slerp(dest, gravityCollider.transform.position, 0.01f);
        //    check = Physics.OverlapSphere(dest, characterController.height / 2f + 0.1f);
        //}

        //characterController.Move(dest);      

        //var groundHits = Physics.OverlapSphere(transform.position, characterController.height / 2f + 0.1f);
        //if (!groundHits.Any(c => c.name == "Apple"))
        //    characterController.Move(grDir * (9.81f) * Time.deltaTime);
        //else
        //    characterController.Move(speed * Time.deltaTime * transform.forward);
        // Debug.Log("Hit! " + string.Join(",", groundHits.Select(c => c.name).ToArray()));

        //playerVelocity += direction * (9.81f) * Time.deltaTime;
        //characterController.Move(direction * (9.81f) * Time.deltaTime);

        //velocity = transform.forward * speed * Time.deltaTime;
        //velocityZ = /*transform.forward */ speed * Time.deltaTime;
    }

    private Vector3 prevMove;
    private bool alloInp = true;

    private bool _rotating;
    public bool Rotating => _rotating;

    public bool AllowRotation { get; set; } = true;

    IEnumerator AllowInput()
    {
        yield return new WaitForSeconds(2f);

        alloInp = true;
    }

    private void FixedUpdate()
    {
        _rotating = false;

        //вынести rb.transform в переменную?
        //подсказка VS о порядке аргументов при умножении вектора на числа
        //хит по лейеру яблока (или ground)
        if (Physics.Raycast(rigidbody.position, -rigidbody.transform.up, out RaycastHit hit))
        {
            var targetRbRotation = rigidbody.transform.rotation;

            

            //змейка ползет криво при 45 градусах и кривовато при 90
            //commented
            var projectRight = Vector3.ProjectOnPlane(hit.normal, rigidbody.transform.right);
            var rightSmooth = Vector3.RotateTowards(rigidbody.transform.up, projectRight,
                   50f * Time.deltaTime * Mathf.Deg2Rad, 1f);
            var rotationRight = Quaternion.FromToRotation(rigidbody.transform.up, rightSmooth);
            //-- commented
            //var projectRight = Vector3.ProjectOnPlane(hit.normal, rigidbody.transform.right);
            //var ang = Vector3.Angle(rigidbody.transform.up, projectRight);
            //var rotationRight = Quaternion.AngleAxis(ang, rigidbody.transform.right);
            //var rightSmooth = Vector3.RotateTowards(rigidbody.transform.up, projectRight,
            //       50f * Time.deltaTime * Mathf.Deg2Rad, 1f);
            //var rotationRight = Quaternion.FromToRotation(rigidbody.transform.up, rightSmooth);
            //Debug.Log($"norm: {hit.normal}, proj: {projectRight}, right: {rigidbody.transform.right}");

            //var rightSmooth = Quaternion.RotateTowards(rigidbody.rotation, rotationRight * rigidbody.rotation, 100f * Time.fixedDeltaTime);

            //if (joystickInput != Vector2.zero)
            //{
            //    var inputV3 = new Vector3(joystickInput.x, 0f, joystickInput.y);
            //    var projectedInput = Vector3.ProjectOnPlane(inputV3, rigidbody.transform.up);
            //    var inputSmooth = Vector3.RotateTowards(rigidbody.transform.forward, projectedInput,
            //        angularSpeed * Time.deltaTime * Mathf.Deg2Rad, 1f);
            //    var inputRot = Quaternion.FromToRotation(rigidbody.transform.forward, inputSmooth);
            //    //var rotFull = Quaternion.FromToRotation(rigidbody.transform.forward, projectedInput);
            //    //var inputRot = Quaternion.RotateTowards(rigidbody.rotation, rotFull, angularSpeed * Time.deltaTime);
            //    targetRbRotation = inputRot * targetRbRotation;
            //    Debug.Log("rotate");
            //}

            //качание вперед-назад ("дифферент")
            targetRbRotation = rotationRight * targetRbRotation;
            //rigidbody.rotation = rotationRight * rigidbody.transform.rotation;

            //if (inputRotation != Quaternion.identity)
            //{
            //    rbRotation = inputRotation * rbRotation;
            //}

            if (joystickInput != Vector2.zero && alloInp && AllowRotation)
            {
                //попробовать как-то отвязать от камеры?
                //зависимость скорости поворота от инпута? учитывать магнитуду в скорости поворота
                var inp = camera.rotation * new Vector3(joystickInput.x, joystickInput.y, 0f);
                var proj = Vector3.ProjectOnPlane(inp, rigidbody.transform.up).normalized;
                //var fwd = camera.rotation * Quaternion.Inverse(rigidbody.rotation) * rigidbody.transform.forward;
                var aF = Vector3.Angle(rigidbody.transform.forward, Vector3.forward);
                //Quaternion.AngleAxis(5, Vector3.up)
                var global = rigidbody.position + new Vector3(joystickInput.x, 0f, joystickInput.y);

                var newZ = Vector3.Angle(rigidbody.transform.forward, proj);
                if (newZ > 5f)
                {
                    var smooth = Vector3.RotateTowards(rigidbody.transform.forward, proj,
                        rotationSpeed * Time.deltaTime * Mathf.Deg2Rad, 1f);
                    var rot = Quaternion.FromToRotation(rigidbody.transform.forward, smooth);

                    //var smooth = Mathf.MoveTowardsAngle(0f, newZ, rotationSpeed * Time.deltaTime);
                    //var rot = Quaternion.AngleAxis(smooth, Vector3.up);
                    targetRbRotation = rot * targetRbRotation;

                    _rotating = true;
                }

                //var inputSmooth = Vector3.RotateTowards(rigidbody.transform.forward, projectedInput,
                //    angularSpeed * Time.deltaTime * Mathf.Deg2Rad, 1f);
                //var smooth = Quaternion.RotateTowards(mesh.rotation, rotationForward * mesh.rotation, 50f * Time.fixedDeltaTime)

                //var proj = Vector3.ProjectOnPlane(inp, rigidbody.transform.up).normalized;
                //var rot = Quaternion.FromToRotation(rigidbody.transform.forward, proj);

                //var rot = Quaternion.AngleAxis(5, Vector3.up);

                //var rot = Quaternion.AngleAxis(5, Vector3.up) * Vector3.forward;
                //rigidbody.rotation = rot * rigidbody.transform.rotation;
                //rigidbody.transform.Rotate(Vector3.up * 5f, Space.World);

                //targetRbRotation = rot * targetRbRotation;

                //alloInp = false;
                //StartCoroutine(AllowInput());
                Debug.Log($"fwd: {rigidbody.transform.forward} inp: {proj} ang: {newZ}");
            }

            //var slopeRotation = Quaternion.FromToRotation(rigidbody.transform.up, hit.normal);
            //rigidbody.transform.rotation = slopeRotation * rigidbody.transform.rotation;
            //rigidbody.MoveRotation(rigidbody.transform.rotation * rotationRight * rotationForward);

            //commented
            rigidbody.MoveRotation(targetRbRotation);

            if (joystickInput != Vector2.zero)
            {
                ////var aZ = Vector3.Angle(Vector3.forward, rigidbody.transform.forward);
                //var newZ = Quaternion.AngleAxis(rigidbody.rotation.x, Vector3.right) *  new Vector3(0f, 0f, joystickInput.y);
                //var newX = Quaternion.AngleAxis(rigidbody.rotation.y, Vector3.up) * new Vector3(joystickInput.x, 0f, 0f);
                ////var newZ = Quaternion.FromToRotation(Vector3.forward, rigidbody.transform.forward) * new Vector3(0f, 0f, joystickInput.y);
                ////var newX = Quaternion.FromToRotation(Vector3.right, rigidbody.transform.right) * new Vector3(joystickInput.x, 0f, 0f);
                //var inputV3 = newZ + newX;
                ////Quaternion.FromToRotation(new Vector3(0f, 0f, joystickInput.y).normalized, rigidbody.transform.forward) *
                ////Quaternion.r.FromToRotation(new Vector3(joystickInput.x, 0f, 0f).normalized, rigidbody.transform.forward)
                ////var inputV3 = new Vector3(joystickInput.x, 0f, joystickInput.y);
                ////var projectedInput = Vector3.ProjectOnPlane(inputV3, rigidbody.transform.up);
                //var inputSmooth = Vector3.RotateTowards(rigidbody.transform.forward, inputV3,
                //    angularSpeed * Time.deltaTime * Mathf.Deg2Rad, 1f);
                //var inputRot = Quaternion.FromToRotation(rigidbody.transform.forward, inputSmooth);
                ////var rotFull = Quaternion.FromToRotation(rigidbody.transform.forward, projectedInput);
                ////var inputRot = Quaternion.RotateTowards(rigidbody.rotation, rotFull, angularSpeed * Time.deltaTime);
                //rigidbody.MoveRotation(inputRot * rigidbody.transform.rotation);
                //Debug.Log("rotate");
            }

            //качание право-лево ("крен")
            var projectForward = Vector3.ProjectOnPlane(hit.normal, mesh.forward);
            var rotationForward = Quaternion.FromToRotation(mesh.up, projectForward);
            mesh.rotation = Quaternion.RotateTowards(mesh.rotation, rotationForward * mesh.rotation, 50f * Time.fixedDeltaTime);
            //mesh.rotation = rotationForward * mesh.rotation;
            //mesh.rotation = Quaternion.Slerp(mesh.rotation, slopeRotation, 5f * Time.deltaTime);

            
            //иcпользовать размеры коллайдера ?
            if (hit.distance > 0.6f || hit.distance < 0.2f) //"гравитация" - прилипание к поверхности
            {
                //Debug.Log($"rb pos: {rigidbody.position}, hit pos: {hit.point}, substr: {rigidbody.position - hit.point}");
                var target = hit.point + (rigidbody.position - hit.point).normalized * 0.55f;
                rigidbody.position = target;
                //rigidbody.position = hit.normal * 0.55f;
                //Debug.Log("stick to ground");
            }
            else //движение вперед
            {               
                var currMove = rigidbody.position + rigidbody.transform.forward * Time.fixedDeltaTime * speed;
                //Debug.Log($"pos: {rigidbody.position} curr: {currMove}, prev: {prevMove}");
                rigidbody.MovePosition(currMove);
                prevMove = currMove;
            }
        }
        else
            Debug.Log("no apple");

        //лог если нет яблока

        //погуглить smooth?
        //rigidbody.velocity = new Vector3(0f, rigidbody.velocity.y, velocityZ);
        //rigidbody.velocity = velocity;
        //rigidbody.MovePosition(rigidbody.position + Vector3.forward * Time.fixedDeltaTime * speed + -Vector3.up * Time.fixedDeltaTime * 9.81f);
        //rigidbody.AddForce(Vector3.forward * Time.fixedDeltaTime * speed, ForceMode.VelocityChange);
        //rigidbody.AddForce(Vector3.forward * speed - rigidbody.velocity, ForceMode.VelocityChange);
        //Debug.Log("Vel: " + rigidbody.velocity);
        //rigidbody.AddForce(transform.forward * speed * Time.fixedDeltaTime, ForceMode.Acceleration);
    }
}
