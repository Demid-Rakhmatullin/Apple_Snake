using System.Collections;
using System.Collections.Generic;
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
    [Tooltip("Angular speed of turning by player input")]
    [SerializeField] float turningSpeed = 6f;
    [Tooltip("Ignore player input if rotation is less than value (angle in degrees)")]
    [SerializeField] float inputAccuracy = 2f;
    [Tooltip("Coefficient to calculate angular speed of aligning to surface normal depending on move speed")]
    [SerializeField] float surfaceAlignCoefficient = 10f;
    [SerializeField] SnakeController snakeController;

    private Rigidbody _rigidbody;
    private BoxCollider _collider;

    private bool _moving;
    //private float _headHalfLength;
    private float _snakeHalfHeight;
    private float _surfaceSmoothing; //ignore surface unevenness <= this value
    private Collider[] _groundCheckColliders;
    private Vector3 _forwardPlaneNormal;

    private RaycastHit[] _hits;
    float time;

    private Vector3 CurrentPostion => _rigidbody.position;

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
        //Debug.Log("Before " + transform.forward);

        var gravityDir = GetGravityDirection();
        if (RaycastGround(transform.position, gravityDir, out RaycastHit hit))
        {
            //Debug.DrawLine(transform.position, hit.point, Color.red, 100f);
            //Debug.Log("Apple found");

            _rigidbody.position = hit.point + (-gravityDir * _snakeHalfHeight);
            //_rigidbody.rotation = GetSurfaceAligninigRotation(hit.normal) * _rigidbody.rotation;
            _rigidbody.rotation = GetSurfaceAligningRotation(transform.forward, hit.normal);

            //_forwardPlaneNormal = Vector3.Cross(appleDir, transform.forward);
            //if (Mathf.Abs(_forwardPlaneNormal.magnitude) <= Mathf.Epsilon)
            //{
            //    Debug.LogWarning("Can't calculate initial forward direction");
            //    return;
            //}

            _moving = true;
            //Debug.Log("After " + transform.forward);
        }
        else
            Debug.LogWarning("Apple not found on start!");

    }

    private void FixedUpdate()
    {
        if (!_moving)
            return;

        //var moveDirection = transform.forward;
        var moveDirection = AdjustDirection(transform.forward, snakeController.Input);
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

        var moveDistance = moveSpeed * Time.fixedDeltaTime;
        var (nextPostionFound, nextPostion, closestGround) = FindNextPostion(
            moveDistance, moveDirection);

        if (!nextPostionFound)
            return;

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

        var groundDir = (closestGround - nextPostion).normalized;
        if (!RaycastGround(nextPostion, groundDir, out RaycastHit groundHit))
        {
            Debug.LogWarning("Can't raycast ground from next postion");
            return;
        }

        _rigidbody.MovePosition(nextPostion);

        //var nextRotation = GetSurfaceAligninigRotation(groundHit.normal) * _rigidbody.rotation;
        var targetRotation = GetSurfaceAligningRotation(moveDirection, groundHit.normal);

        //var groundDirN = (closestGroundN - nextPostionN).normalized;
        //RaycastGround(nextPostionN, groundDirN, out RaycastHit groundHitN);
        //Debug.Log("Test old: " + targetRotation.eulerAngles + ", new: " + GetSurfaceAligningRotation(moveDirection, groundHit.normal));
        
        _rigidbody.MoveRotation(Quaternion.RotateTowards(_rigidbody.rotation, targetRotation,
            Time.fixedDeltaTime * moveSpeed * surfaceAlignCoefficient));
        //_rigidbody.MoveRotation(targetRotation);

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

    private Vector3 AdjustDirection(Vector3 forward, Vector3 input)
    {
        if (input != Vector3.zero)
        {
            //ignore input magnitude (joystick deflection degree), consider only the direction
            var projectedInput = Vector3.ProjectOnPlane(input.normalized, transform.up);
            var angle = Vector3.Angle(forward, projectedInput);
            if (angle > inputAccuracy)
            {
                forward = Vector3.RotateTowards(forward, projectedInput.normalized,
                    Time.fixedDeltaTime * turningSpeed, 0f);
            }
        }
        return forward;
    }

    private (bool success, Vector3 nextPostion, Vector3 closestGround) FindNextPostion(
        float moveDistance, Vector3 moveDirection)
    {
        var nextPostion = GetPotentialPosition(moveDistance, moveDirection);
        var validatePosition = ValidatePosition(nextPostion,
            out Vector3 closestGround);

        if (validatePosition != ValidatePositionResult.Success)
        {
            const int MAX_ATTEMTS = 90;
            const float ROTATION_STEP = 1f * Mathf.Deg2Rad;

            var gravityDir = GetGravityDirection();
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
                nextPostion = GetPotentialPosition(moveDistance, moveDirection);
                validatePosition = ValidatePosition(nextPostion, out closestGround);
                //Debug.Log("Validate: " + validatePosition);
                if (validatePosition == ValidatePositionResult.Success)
                    break;
            }
        }

        if (validatePosition != ValidatePositionResult.Success)
        {
            Debug.LogWarning("Can't find next postion");
            return default;
        }

        return (true, nextPostion, closestGround);
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

    private Vector3 GetPotentialPosition(float distance, Vector3 direction)
        => CurrentPostion + distance * direction;

    private Vector3 GetGravityDirection()
        => (gravityCenter.position - CurrentPostion).normalized;

    private bool RaycastGround(Vector3 origin, Vector3 direction, out RaycastHit groundHit)
        => Physics.Raycast(origin, direction, out groundHit, Mathf.Infinity, GROUND_LAYER);

    //private Vector3 GetHeightMargin(Vector3 centerDirection)
    //    => -centerDirection * _snakeHalfHeight;

    private Quaternion GetSurfaceAligningRotation(Vector3 snakeForward, Vector3 surfaceNormal)
        => Quaternion.LookRotation(snakeForward, surfaceNormal);

    //private Quaternion GetSurfaceAligninigRotation(Vector3 surfaceNormal)
        //=> Quaternion.FromToRotation(transform.up, surfaceNormal);
}
