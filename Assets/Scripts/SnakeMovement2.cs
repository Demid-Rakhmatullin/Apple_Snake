using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using static DebugDrawing;

[RequireComponent(typeof(Rigidbody), typeof(BoxCollider))]
public class SnakeMovement2 : MonoBehaviour
{
    static int APPLE_LAYER;

    [SerializeField] float raycastDistance;
    [SerializeField] float gravityForce;

    [SerializeField] Transform appleCenter;
    [SerializeField] float moveSpeed;

    private Rigidbody _rigidbody;
    private BoxCollider _collider;

    private bool _moving;
    private float _headHalfLength;
    private float _snakeHalfHeight;
    private float _surfaceSmoothing; //ignore surface unevenness <= this value
    private Collider[] _groundCheckColliders;
    private Vector3 _forwardPlaneNormal;

    private RaycastHit[] _hits;
    float time;

    private Vector3 CurrentPostion => _rigidbody.position;

    private void Awake()
    {
        APPLE_LAYER = LayerMask.GetMask("Apple");
        _rigidbody = GetComponent<Rigidbody>();
        _collider = GetComponent<BoxCollider>();
        _groundCheckColliders = new Collider[10];           
    }

    void Start()
    {
        _headHalfLength = _collider.size.z / 2f;
        _snakeHalfHeight = _collider.size.y / 2f;
        _surfaceSmoothing = _snakeHalfHeight * 0.2f;

        _hits = new RaycastHit[10];
        //Debug.Log("Before " + transform.forward);

        var appleDir = GetAppleCenterDirection();
        if (Physics.Raycast(transform.position, appleDir, out RaycastHit hit,
            100f, APPLE_LAYER))
        {
            //Debug.DrawLine(transform.position, hit.point, Color.red, 100f);
            //Debug.Log("Apple found");

            _rigidbody.position = hit.point + GetHeightMargin(appleDir);
            _rigidbody.rotation = GetSurfaceAligninigRotation(hit.normal) * _rigidbody.rotation;

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

    // Update is called once per frame
    void Update()
    {

    }

    private void FixedUpdate()
    {
        if (!_moving)
            return;

        var moveDistance = moveSpeed * Time.fixedDeltaTime;
        var moveDirection = transform.forward;
        var appleDir = GetAppleCenterDirection();

        const int MAX_ATTEMTS = 90;
        const float ROTATION_STEP = 1f * Mathf.Deg2Rad;
        int i = 1;
        Vector3 nextPosition;

        while (true)
        {
            var nextPostionRes = ValidateMoveDirection(moveDirection, moveDistance, out nextPosition);
            Debug.Log("nextPostionRes: " + nextPostionRes);
            if (nextPostionRes == ValidateDirectionResult.Success)
                break;

            var rotationDelta = nextPostionRes switch
            {
                ValidateDirectionResult.PathNotClear or ValidateDirectionResult.GroundTooClose
                    => -ROTATION_STEP,
                ValidateDirectionResult.GroundNotFound
                    => ROTATION_STEP,
                _ => throw new System.InvalidOperationException("Can't process ValidateDirectionResult!")
            };

            moveDirection = Vector3.RotateTowards(moveDirection, appleDir, rotationDelta, 0f);

            i++;
            if (i > MAX_ATTEMTS)
            {
                Debug.LogWarning("Can't find next postion");
                return;
            }
        }

        DrawSphere(nextPosition, 0.1f, Color.red);

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

    private ValidateDirectionResult ValidateMoveDirection(Vector3 moveDirection, float moveDistance, 
        out Vector3 validatedNextPosition)
    {
        validatedNextPosition = Vector3.zero;

        var headForwardPoint = CurrentPostion + transform.forward * _headHalfLength;
        DrawSphere(headForwardPoint, 0.1f, Color.green);
        var pathIsClear = !Physics.Raycast(headForwardPoint, moveDirection, out var pathObstacle,
           moveDistance, APPLE_LAYER);

        if (pathIsClear)
        {
            var nextPosition = CurrentPostion + moveDistance * moveDirection;
            //Debug.Log("Draw blue");
            Debug.DrawLine(headForwardPoint, nextPosition, Color.blue);
            //DrawSphere(nextPosition, 0.1f, Color.blue);
            var groundCheck = Physics.OverlapSphereNonAlloc(nextPosition, _snakeHalfHeight + _surfaceSmoothing,
                _groundCheckColliders, APPLE_LAYER);

            if (groundCheck > 0)
            {
                for (int i = 0; i < groundCheck; i++)
                {
                    var closestPoint = _groundCheckColliders[i].ClosestPoint(nextPosition);
                    
                    if (Vector3.Distance(closestPoint, nextPosition) < _snakeHalfHeight - _surfaceSmoothing)
                        return ValidateDirectionResult.GroundTooClose;
                }

                validatedNextPosition = nextPosition;
                return ValidateDirectionResult.Success;
            }

            return ValidateDirectionResult.GroundNotFound;
        }

        Debug.DrawLine(headForwardPoint, pathObstacle.point, Color.blue);
        return ValidateDirectionResult.PathNotClear;
    }

    //private Vector3 Rotate

    private Vector3 GetAppleCenterDirection()
        => (appleCenter.position - CurrentPostion).normalized;

    private Vector3 GetHeightMargin(Vector3 centerDirection)
        => -centerDirection * _snakeHalfHeight;

    private Quaternion GetSurfaceAligninigRotation(Vector3 surfaceNormal)
        => Quaternion.FromToRotation(transform.up, surfaceNormal);

    private enum ValidateDirectionResult
    {
        Success,
        PathNotClear,
        GroundNotFound,
        GroundTooClose
    }

}
