using System.Collections;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UIElements;
using static DebugDrawing;

[RequireComponent(typeof(Rigidbody), typeof(BoxCollider))]
public class SnakeMovement2 : MonoBehaviour
{
    static int GROUND_LAYER;

    [SerializeField] float raycastDistance;
    [SerializeField] float gravityForce;

    [SerializeField] Transform gravityCenter;
    [SerializeField] float moveSpeed;
    [Tooltip("Angular speed of steering by player input")]
    [SerializeField] float steeringSpeed = 5f;
    [Tooltip("Ignore player input if rotation is less than value (angle in degrees)")]
    [SerializeField] float inputAccuracy = 2f;
    [Tooltip("Coefficient to calculate angular speed of aligning to surface normal depending on move speed")]
    [SerializeField] float groundNormalAlignSpeed = 10f;
    [SerializeField] float slerpCoefficient = 1f;
    [SerializeField] float bodyPartsAngularSpeed = 1f;
    [SerializeField] SnakeController snakeController;

    private Rigidbody _rigidbody;
    private BoxCollider _collider;

    private bool _moving;
    //private float _headHalfLength;
    private float _snakeHalfHeight;
    private float _surfaceSmoothing; //ignore surface unevenness <= this value
    private Collider[] _groundCheckColliders;
    private Vector3 _forwardPlaneNormal;

    [SerializeField] private Transform[] bodyParts;
    [SerializeField] private Transform test;

    private RaycastHit[] _hits;
    float time;

    //private Vector3 CurrentPostion => _rigidbody.position;

    private void Awake()
    {
        GROUND_LAYER = LayerMask.GetMask("Ground");
        _rigidbody = GetComponent<Rigidbody>();
        _collider = GetComponent<BoxCollider>();
        _groundCheckColliders = new Collider[10];           
    }

    private void Start()
    {
        //_headHalfLength = _collider.size.z / 2f;
        _snakeHalfHeight = _collider.size.y / 2f;
        _surfaceSmoothing = _snakeHalfHeight * 0.2f;

        _hits = new RaycastHit[10];
        //Debug.Log("Before " + transform.position);

        var gravityDir = GetGravityDirection(transform.position);
        if (RaycastGround(transform.position, gravityDir, out RaycastHit hit))
        {
            //Debug.DrawLine(transform.position, hit.point, Color.red, 100f);
            //Debug.Log("Apple found");
            //_rigidbody.rotation = GetSurfaceAligninigRotation(hit.normal) * _rigidbody.rotation;
            transform.SetPositionAndRotation(hit.point + (-gravityDir * _snakeHalfHeight), 
                AlignToGroundNormal(transform.forward, hit.normal));

            //_forwardPlaneNormal = Vector3.Cross(appleDir, transform.forward);
            //if (Mathf.Abs(_forwardPlaneNormal.magnitude) <= Mathf.Epsilon)
            //{
            //    Debug.LogWarning("Can't calculate initial forward direction");
            //    return;
            //}

            var prevPart = transform;
            for (int i = 0; i < bodyParts.Length; i++)
            {
                //Debug.Log(i.ToString());
                //Debug.Log("fwd: " + prevPart.forward);
                //Debug.Log("pos: " + prevPart.position);
                var (positionSuccess, position, direction, closestGround) = FindNextPostion(
                    prevPart.transform.position,
                    1.7f,
                    -prevPart.forward
                );

                if (!positionSuccess)
                {
                    Debug.LogWarning("Can't position body segments on start");
                    return;
                }

                var (alignSuccess, targetRotation) = AlignToGroundNormal(
                    -direction, position, closestGround);

                if (!alignSuccess)
                {
                    Debug.LogWarning("Can't find ground for body segment on start");
                    return;
                }

                bodyParts[i].transform.position = position;
                bodyParts[i].transform.rotation = targetRotation;
                prevPart = bodyParts[i].transform;
            }

            _moving = true;
            Physics.SyncTransforms();
            //Debug.Log("After " + transform.position);

            //var rb = test.GetComponent<Rigidbody>();
            //Debug.Log("Before " + rb.position);
            //test.position += Vector3.forward;
            ////test.rotation = Quaternion.Euler(Vector3.up * 30f);
            //Physics.SyncTransforms();
            //Debug.Log("After " + rb.position);
        }
        else
            Debug.LogWarning("Apple not found on start!");

    }

    private void FixedUpdate()
    {
        if (!_moving)
            return;

        //var moveDirection = transform.forward;
        //var initialDirection = TurnByInput(transform.forward, snakeController.Input);
        //var playerInput = snakeController.Input;
        //if (playerInput != Vector3.zero)
        //{
        //    //ignore input magnitude (joystick deflection degree), consider only the direction
        //    var projected = Vector3.ProjectOnPlane(playerInput.normalized, transform.up); 
        //    var angle = Vector3.Angle(moveDirection, projected);
        //    if (angle > 2f)
        //    {
        //        moveDirection = projected.normalized;
        //        Debug.Log("change move direction");
        //    }
        //}

        Quaternion targetRotation;
        Vector3 steeredForward;

        var (steered, steerResult) = SteerByInput(transform.forward, snakeController.Input);
        if (steered)
        {
            targetRotation = Quaternion.LookRotation(steerResult, transform.up);
            steeredForward = steerResult;
        }
        else
        {
            targetRotation = transform.rotation;
            steeredForward = transform.forward;
        }

        //var moveDistance = moveSpeed * Time.fixedDeltaTime;
        var (nextPostionFound, nextPostion, moveDirection, closestGround) = FindNextPostion(
            transform.position,
            moveSpeed * Time.fixedDeltaTime,
            steeredForward
            //SteerByInput(transform.forward, snakeController.Input)
        );

        if (!nextPostionFound)
        {
            Debug.LogWarning("Can't find next postion");
            return;
        }

        //var nextPostion = GetPotentialPosition(moveDistance, moveDirection);
        //var validatePosition = ValidatePosition(nextPostion,
        //    out Vector3 closestGround);

        //if (validatePosition != ValidatePositionResult.Success)
        //{
        //    const int MAX_ATTEMTS = 90;
        //    const float ROTATION_STEP = 1f * Mathf.Deg2Rad;

        //    var gravityDir = GetGravityDirection();
        //    var verticalDelta = validatePosition switch
        //    {
        //        ValidatePositionResult.InsideGround or ValidatePositionResult.GroundTooClose
        //            => -ROTATION_STEP,  //"climb a hill"
        //        ValidatePositionResult.GroundNotFound
        //            => ROTATION_STEP,   //"go down the hill"
        //        _ => throw new System.InvalidOperationException("Can't process ValidateDirectionResult!")
        //    };

        //    for (int i = 0; i < MAX_ATTEMTS; i++)
        //    {
        //        moveDirection = Vector3.RotateTowards(moveDirection, gravityDir, verticalDelta, 0f);
        //        nextPostion = GetPotentialPosition(moveDistance, moveDirection);
        //        validatePosition = ValidatePosition(nextPostion, out closestGround);
        //        Debug.Log("Validate: " + validatePosition);
        //        if (validatePosition == ValidatePositionResult.Success)
        //            break;
        //    }
        //}

        //if (validatePosition != ValidatePositionResult.Success)
        //{
        //    Debug.LogWarning("Can't find next postion");
        //    return;
        //}
        //Debug.DrawLine(transform.position, nextPostion, Color.red);

        //var groundDir = (closestGround - nextPostion).normalized;
        //if (!RaycastGround(nextPostion, groundDir, out RaycastHit groundHit))
        //{
        //    Debug.LogWarning("Can't raycast ground from next postion");
        //    return;
        //}

        //var targetRotation = GetSurfaceAligningRotation(moveDirection, groundHit.normal);
        //var bodyNextPos = CurrentPostion;
        //var bodyNextRot = _rigidbody.rotation;

        var (groundFound, groundNormalRotation) = AlignToGroundNormal(
            moveDirection, nextPostion, closestGround);

        if (!groundFound)
        {
            Debug.LogWarning("Can't raycast ground from next postion");
            return;
        }

        targetRotation = Quaternion.RotateTowards(
            targetRotation,
            groundNormalRotation,
            Time.fixedDeltaTime * moveSpeed * groundNormalAlignSpeed);

        //Debug.Log("Before " + _rigidbody.position);
        _rigidbody.MovePosition(nextPostion);
        // Debug.Log("After " + _rigidbody.position);
        //var nextRotation = GetSurfaceAligninigRotation(groundHit.normal) * _rigidbody.rotation;

        //var groundDirN = (closestGroundN - nextPostionN).normalized;
        //RaycastGround(nextPostionN, groundDirN, out RaycastHit groundHitN);
        //Debug.Log("Test old: " + targetRotation.eulerAngles + ", new: " + GetSurfaceAligningRotation(moveDirection, groundHit.normal));

        //var rot = Quaternion.RotateTowards(_rigidbody.rotation, targetRotation,
        //    Time.fixedDeltaTime * moveSpeed * steeringSpeed * surfaceAlignCoefficient);
        //_rigidbody.MoveRotation(rot);
        _rigidbody.MoveRotation(targetRotation);

        //var prevPart = transform;


        if (bodyParts != null)
            foreach (var body in bodyParts)
            {
                var rb = body.GetComponent<Rigidbody>();
                var dist = Vector3.Distance(body.position, nextPostion);
                //var t = Time.fixedDeltaTime * moveSpeed /* dist*/ / (1.7f * moveSpeed) /*mindistance*/;
                //var t = moveSpeed / (1.7f * moveSpeed * slerpCoefficient);
                var t = (dist - 1.7f) / dist;
                if (t > 0f)
                {
                    var bodyPos = Vector3.Slerp(body.position, nextPostion, t);
                    var moveDir = bodyPos - body.position;
                    if (moveDir != Vector3.zero)
                    {
                        rb.MovePosition(bodyPos);
                        rb.MoveRotation(Quaternion.LookRotation(moveDir, body.up));
                    }
                    else
                        Debug.Log("Zero");

                    nextPostion = bodyPos;
                }
                else
                {
                    nextPostion = body.position;
                    Debug.Log("Skip");
                }

                //var bodyRot = Quaternion.RotateTowards(body.rotation, targetRotation,
                //    Time.fixedDeltaTime * moveSpeed * bodyPartsAngularSpeed);
                //rb.MoveRotation(bodyRot);
                //targetRotation = bodyRot;

                //var (tempPosition, tempRotation) = (body.position, body.rotation);
                //body.MovePosition(bodyNextPos);
                //body.MoveRotation(bodyNextRot);
                //bodyNextPos = tempPosition;
                //bodyNextRot = tempRotation;
            }

        #region Commented

        //Debug.Log("Validate: " + ValidatePosition(nextPostion));
        //var appleDir = GetAppleCenterDirection();

        //const int MAX_ATTEMTS = 90;
        //const float ROTATION_STEP = 1f * Mathf.Deg2Rad;
        //int i = 1;
        //Vector3 nextPosition;

        //while (true)
        //{
        //    var nextPostionRes = ValidatePosition(moveDirection, moveDistance, out nextPosition);
        //    Debug.Log("nextPostionRes: " + nextPostionRes);
        //    if (nextPostionRes == ValidatePositionResult.Success)
        //        break;

        //    var rotationDelta = nextPostionRes switch
        //    {
        //        ValidatePositionResult.PathNotClear or ValidatePositionResult.GroundTooClose
        //            => -ROTATION_STEP,
        //        ValidatePositionResult.GroundNotFound
        //            => ROTATION_STEP,
        //        _ => throw new System.InvalidOperationException("Can't process ValidateDirectionResult!")
        //    };

        //    moveDirection = Vector3.RotateTowards(moveDirection, appleDir, rotationDelta, 0f);

        //    i++;
        //    if (i > MAX_ATTEMTS)
        //    {
        //        Debug.LogWarning("Can't find next postion");
        //        return;
        //    }
        //}

        //DrawSphere(nextPosition, 0.1f, Color.red);

        #endregion

        #region Commented

        //var currentPosition = _rigidbody.position;
        //var pathIsClear = !Physics.Raycast(currentPosition, moveDirection, out _,
        //    moveDistance, APPLE_LAYER);

        //if (pathIsClear)
        //{
        //    var nextPosition = currentPosition + moveDistance * moveDirection;
        //    var groundCheck = Physics.OverlapSphereNonAlloc(nextPosition, _snakeHalfHeight + _surfaceSmoothing,
        //        _groundCheckColliders, APPLE_LAYER);

        //    if (groundCheck > 0)
        //    {
        //        for (int i = 0; i < groundCheck; i++)
        //        {
        //            var closestPoint = _groundCheckColliders[i].ClosestPoint(nextPosition);
        //            DrawSphere(closestPoint, 0.1f, Color.red);
        //            //Debug.D
        //            if (Vector3.Distance(closestPoint, nextPosition) < _snakeHalfHeight - _surfaceSmoothing)
        //            {
        //                //Quaternion.rot

        //                Debug.Log("too close " + closestPoint);
        //                break;
        //            }
        //        }
        //    }
        //    //Debug.Log("Draw " + Vector3.Distance(currentPosition, nextPosition));
        //    Debug.DrawLine(currentPosition, nextPosition, Color.red);
        //}

        //if (Time.time - time > 2f)
        //{
        //    Debug.Log("Before " + transform.forward);
        //    transform.Rotate(Vector3.right, 5f, Space.Self);
        //    Debug.Log("After " + transform.forward);
        //    time = Time.time;
        //}

        //for (var i = 0f; i < 360f; i++)
        //{
        //    var gravityDir = gravityCenter.position - transform.position;
        //    var gravityCross = Vector3.Cross(gravityDir, transform.forward);
        //    var rayDirection = Quaternion.AngleAxis(i, gravityCross) * transform.forward;

        //    if (Physics.Raycast(transform.position, rayDirection, out RaycastHit hit, raycastDistance))
        //    {
        //        Debug.DrawLine(transform.position, hit.point, Color.red);
        //    }
        //}

        //var gravityDir = gravityCenter.position - transform.position;

        //if (Physics.Raycast(transform.position, gravityDir, out RaycastHit hit, 
        //    raycastDistance, APPLE_LAYER))
        //{
        //    //Vector3.move
        //    Debug.DrawLine(transform.position, hit.point, Color.red);
        //}

        //if (Physics.Raycast(transform.position, -transform.up, out RaycastHit hitGround, raycastDistance))
        //{
        //    Debug.Log("hit ground " + hitGround.distance);
        //    Debug.DrawLine(transform.position, hitGround.point, Color.red);
        //    //Gizmos.DrawSphere(hitGround.point, 0.1f);
        //}

        //if (Physics.SphereCastNonAlloc(transform.position, raycastDistance, transform.forward, _hits, raycastDistance) > 0)
        //{
        //    Debug.Log("hit all " + _hits.First(h => h.collider != null).distance + " " 
        //        + string.Join(",", _hits.Where(h => h.collider != null).Select(h => h.collider.name)));

        //    Debug.DrawLine(transform.position, _hits[0].point, Color.green);
        //}

        //if (Physics.SphereCast(transform.position, raycastDistance - 1f, -transform.up, out RaycastHit sphereHit, raycastDistance))
        //{
        //    Debug.Log("hit sphere " + sphereHit.distance);
        //}

        #endregion
    }

    private (bool, Vector3) SteerByInput(Vector3 forward, Vector3 input)
    {
        if (input != Vector3.zero)
        {
            //ignore input magnitude (joystick deflection degree), consider only the direction
            var projectedInput = Vector3.ProjectOnPlane(input.normalized, transform.up);
            var angle = Vector3.Angle(forward, projectedInput);
            if (angle > inputAccuracy)
            {
                return (true, Vector3.RotateTowards(forward, projectedInput.normalized,
                    Time.fixedDeltaTime * steeringSpeed, 0f));
            }
        }
        return (false, forward);
    }

    private (bool success, Vector3 nextPostion, Vector3 moveDirection, Vector3 closestGround) FindNextPostion(
        Vector3 currentPosition, float moveDistance, Vector3 initialDirection)
    {
        var moveDirection = initialDirection;
        var nextPostion = GetPotentialPosition(currentPosition, moveDistance, moveDirection);
        var validatePosition = ValidatePosition(nextPostion,
            out Vector3 closestGround);

        //rotate move direction upwards or downwards
        if (validatePosition != ValidatePositionResult.Success)
        {
            const int MAX_ATTEMTS = 90;
            const float ROTATION_STEP = 1f * Mathf.Deg2Rad;

            var gravityDir = GetGravityDirection(currentPosition);
            var verticalDelta = validatePosition switch
            {
                ValidatePositionResult.InsideGround or ValidatePositionResult.GroundTooClose
                    => -ROTATION_STEP,  //"climb a hill"
                ValidatePositionResult.GroundNotFound
                    => ROTATION_STEP,   //"go down the hill"
                _ => throw new System.InvalidOperationException("Can't process ValidateDirectionResult!")
            };

            for (int i = 0; i < MAX_ATTEMTS; i++)
            {
                moveDirection = Vector3.RotateTowards(moveDirection, gravityDir, verticalDelta, 0f);
                nextPostion = GetPotentialPosition(currentPosition, moveDistance, moveDirection);
                validatePosition = ValidatePosition(nextPostion, out closestGround);
                //Debug.Log("Validate: " + validatePosition);
                if (validatePosition == ValidatePositionResult.Success)
                    break;
            }
        }

        if (validatePosition != ValidatePositionResult.Success)
            return default;

        return (true, nextPostion, moveDirection, closestGround);
    }

    private ValidatePositionResult ValidatePosition(Vector3 postion, out Vector3 closestGround)
    {
        closestGround = postion;

        var groundCheck = Physics.OverlapSphereNonAlloc(postion, _snakeHalfHeight + _surfaceSmoothing,
            _groundCheckColliders, GROUND_LAYER);

        if (groundCheck > 0)
        {
            for (int i = 0; i < groundCheck; i++)
            {
                closestGround = _groundCheckColliders[i].ClosestPoint(postion);

                if (postion.Equals(closestGround))
                    return ValidatePositionResult.InsideGround;

                if (Vector3.Distance(closestGround, postion) < _snakeHalfHeight - _surfaceSmoothing)
                    return ValidatePositionResult.GroundTooClose;
            }

            return ValidatePositionResult.Success;
        }
        
        return ValidatePositionResult.GroundNotFound;
    }

    private enum ValidatePositionResult
    {
        Success,
        GroundNotFound,
        GroundTooClose,
        InsideGround
    }

    #region Commented
    //private ValidatePositionResult ValidatePosition(Vector3 moveDirection, float moveDistance, 
    //    out Vector3 validatedNextPosition)
    //{
    //    validatedNextPosition = Vector3.zero;

    //    //var headForwardPoint = CurrentPostion + transform.forward * _headHalfLength;
    //    //DrawSphere(headForwardPoint, 0.1f, Color.green);
    //    //var pathIsClear = !Physics.Raycast(headForwardPoint, moveDirection, out var pathObstacle,
    //    //   moveDistance, APPLE_LAYER);

    //    //if (pathIsClear)
    //    //{
    //        var nextPosition = CurrentPostion + moveDistance * moveDirection;
    //        //Debug.Log("Draw blue");
    //        //Debug.DrawLine(headForwardPoint, nextPosition, Color.blue);
    //        //DrawSphere(nextPosition, 0.1f, Color.blue);
    //        var groundCheck = Physics.OverlapSphereNonAlloc(nextPosition, _snakeHalfHeight + _surfaceSmoothing,
    //            _groundCheckColliders, APPLE_LAYER);

    //        if (groundCheck > 0)
    //        {
    //            for (int i = 0; i < groundCheck; i++)
    //            {
    //                var closestPoint = _groundCheckColliders[i].ClosestPoint(nextPosition);

    //                if (Vector3.Distance(closestPoint, nextPosition) < _snakeHalfHeight - _surfaceSmoothing)
    //                    return ValidatePositionResult.GroundTooClose;
    //            }

    //            validatedNextPosition = nextPosition;
    //            return ValidatePositionResult.Success;
    //        }

    //        return ValidatePositionResult.GroundNotFound;
    //   // }

    //    //Debug.DrawLine(headForwardPoint, pathObstacle.point, Color.blue);
    //   // return ValidateDirectionResult.PathNotClear;
    //}
    #endregion

    private Vector3 GetPotentialPosition(Vector3 currentPostion, float distance, Vector3 direction)
        => currentPostion + distance * direction;

    private Vector3 GetGravityDirection(Vector3 currentPostion)
        => (gravityCenter.position - currentPostion).normalized;

    //private Vector3 GetHeightMargin(Vector3 centerDirection)
    //    => -centerDirection * _snakeHalfHeight;

    private (bool success, Quaternion rotation) AlignToGroundNormal(
        Vector3 forward, Vector3 position, Vector3 surfacePoint)
    {
        var surfaceDir = (surfacePoint - position).normalized;
        if (!RaycastGround(position, surfaceDir, out RaycastHit surfaceHit))
            return (false, default);

        return (true, AlignToGroundNormal(forward, surfaceHit.normal));
    }

    private Quaternion AlignToGroundNormal(Vector3 forward, Vector3 groundNormal)
        => Quaternion.LookRotation(forward, groundNormal);

    //private Quaternion GetSurfaceAligninigRotation(Vector3 surfaceNormal)
        //=> Quaternion.FromToRotation(transform.up, surfaceNormal);

    private bool RaycastGround(Vector3 origin, Vector3 direction, out RaycastHit groundHit)
        => Physics.Raycast(origin, direction, out groundHit, Mathf.Infinity, GROUND_LAYER);
}
