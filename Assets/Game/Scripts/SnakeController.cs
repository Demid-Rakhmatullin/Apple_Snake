using System.Collections.Generic;
using UnityEngine;
using static PhysicsHelpers;

[RequireComponent(typeof(SnakeSegment))]
public class SnakeController : MonoBehaviour
{
    [SerializeField] GameObject bodySegmentPrefab;
    [SerializeField] GameObject tailPrefab;
    [SerializeField] int initialSegments = 6;
    [SerializeField] float bodySegmentsPadding = 0f;
    [SerializeField] SnakeFollowCamera followCamera;
    [SerializeField] DynamicJoystick joystick;
    [SerializeField] Transform gravityCenter;
    [SerializeField] float moveSpeed = 10f;
    [Tooltip("Angular speed of steering by player input")]
    [SerializeField] float steeringSpeed = 5f;
    [Tooltip("Ignore player input if rotation is less than value (angle in degrees)")]
    [SerializeField] float inputAccuracy = 2f;
    [Tooltip("Coefficient to calculate angular speed of aligning to surface normal depending on move speed")]
    [SerializeField] float groundNormalAlignSpeed = 10f;

    private SnakeSegment _head;
    private LinkedList<SnakeSegment> _body;
    private Collider[] _groundCheckColliders;
    private bool _moving;
    private Vector3 _steeringInput;

    public Vector3 Input { get; private set; }

    void Awake()
    {
        _groundCheckColliders = new Collider[10];
        _head = GetComponent<SnakeSegment>();
        _body = new();
    }

    void Start()
    {
        _head.Init(gravityCenter);

        var gravityDir = _head.GetGravityDirection();
        if (_head.RaycastGround(gravityDir, out RaycastHit hit))
        {
            _head.transform.SetPositionAndRotation(hit.point + (-gravityDir * _head.HalfHeight),
                _head.AlignToGroundNormal(hit.normal));

            CreateInitialBody();

            _moving = true;
            followCamera.Init(_head.transform, gravityCenter);
            Physics.SyncTransforms();
        }
        else
            Debug.LogWarning("Apple not found on start!");
    }    

    void Update()
    {
        _steeringInput = followCamera.transform.TransformDirection(
            joystick.Horizontal, joystick.Vertical, 0f);
    }

    void FixedUpdate()
    {
        if (!_moving)
            return;

        var groundAlignMaxDelta = Time.fixedDeltaTime * moveSpeed * groundNormalAlignSpeed;

        var (headSuccess, nextPosition, nextForward, moveDistance) = 
            MoveHead(groundAlignMaxDelta);
        if (!headSuccess)
            return;

        MoveBody(nextPosition, nextForward, moveDistance, groundAlignMaxDelta);                            
    }

    private (bool, Vector3, Vector3, float) MoveHead(float groundAlignMaxDelta)
    {
        Quaternion nextRotation;
        Vector3 steeredForward;

        var (steered, steerResult) = SteerByInput(transform.forward, _steeringInput);
        if (steered)
        {
            nextRotation = Quaternion.LookRotation(steerResult, transform.up);
            steeredForward = steerResult;
        }
        else
        {
            nextRotation = transform.rotation;
            steeredForward = transform.forward;
        }

        var moveDistance = moveSpeed * Time.fixedDeltaTime;
        var (nextPostionFound, nextPostion, moveDirection, closestGround) = _head.FindNextPosition(
            _groundCheckColliders,
            moveDistance,
            steeredForward
        );

        if (!nextPostionFound)
        {
            Debug.LogWarning("Can't find next postion (head movement)");
            return default;
        }

        var (groundFound, groundNormalRotation) = AlignToGroundNormal(
            moveDirection, nextPostion, closestGround);

        if (!groundFound)
        {
            Debug.LogWarning("Can't find ground for next postion (head movement)");
            return default;
        }

        nextRotation = Quaternion.RotateTowards(
            nextRotation,
            groundNormalRotation,
            groundAlignMaxDelta);

        _head.Rigidbody.MovePosition(nextPostion);
        _head.Rigidbody.MoveRotation(nextRotation);

        return (true, nextPostion, moveDirection, moveDistance);
    }

    private void MoveBody(Vector3 headNextPosition, Vector3 headNextForward, float moveDistance, 
        float groundAlignMaxDelta)
    {
        var prevSegmentNextPosition = headNextPosition;
        var prevSegmentNextForward = headNextForward;
        var prevSegmentHalfLength = _head.HalfLength;

        foreach (var segment in _body)
        {
            var currentPostion = segment.transform.position;
            var padding = segment.HalfLength + prevSegmentHalfLength + bodySegmentsPadding;
            var actualDistance = Vector3.Distance(currentPostion, 
                prevSegmentNextPosition);
            var expectedSlerpDistance = moveDistance + padding;
            Vector3 slerpDestination = default;
            float slerpT = 0f;

            if (actualDistance < expectedSlerpDistance - Mathf.Epsilon)
            {
                //use Law of Sines to calculate next position for current segment if the snake moved straight along a plane
                //then use this speculative point as slerp destination
                //
                var prevSegmentNextBackward = -prevSegmentNextForward;
                var prevSegmentAngle = Vector3.Angle(prevSegmentNextPosition - currentPostion, 
                    prevSegmentNextBackward);
                var prevSegmentAngleSin = Mathf.Sin(prevSegmentAngle * Mathf.Deg2Rad);

                if (prevSegmentAngleSin > Mathf.Epsilon)
                {
                    var plainMovementAngleSin = actualDistance * prevSegmentAngleSin / expectedSlerpDistance;
                    if (plainMovementAngleSin <= 0f)
                        Debug.LogWarning("Unexpected negative sine during body move");

                    var currSegmentAngleSin = Mathf.Sin(
                        (180f - prevSegmentAngle - Mathf.Asin(plainMovementAngleSin) * Mathf.Rad2Deg) * Mathf.Deg2Rad);
                    if (currSegmentAngleSin > Mathf.Epsilon)
                    {
                        slerpDestination = prevSegmentNextPosition + prevSegmentNextBackward *
                            (actualDistance * currSegmentAngleSin / plainMovementAngleSin);
                        slerpT = moveDistance / expectedSlerpDistance;
                    }
                }
            }

            if (slerpT < Mathf.Epsilon)
            {
                //otherwise use previous segment next position as slerp destination
                //
                slerpDestination = prevSegmentNextPosition;
                slerpT = (actualDistance - padding) / actualDistance;
            }

            if (slerpT > 0f)
            {
                //todo: use gravity center to calculate slerp vectors (while the apple is located at the origin point (0, 0, 0) this doesn't matter)
                var nextPosition = Vector3.Slerp(currentPostion, slerpDestination, slerpT);
                var nextForward = (prevSegmentNextPosition - nextPosition).normalized;
                var nextRotation = segment.FindRoughNextRotation(_groundCheckColliders,
                    nextPosition, nextForward, groundAlignMaxDelta);

                segment.Rigidbody.MovePosition(nextPosition);
                segment.Rigidbody.MoveRotation(nextRotation);

                prevSegmentNextPosition = nextPosition;
                prevSegmentNextForward = nextForward;
            }
            else
            {
                prevSegmentNextPosition = segment.transform.position;
                prevSegmentNextForward = segment.transform.forward;
                Debug.LogWarning("Can't find next postion (body movement). Skip current segment.");
            }

            prevSegmentHalfLength = segment.HalfLength;
        }
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

    private void CreateInitialBody()
    {
        var prevSegment = _head;
        for (int i = 0; i < initialSegments; i++)
        {
            var (success, newSegment) = CreateSegment(bodySegmentPrefab, prevSegment);
            if (!success)
                break;

            _body.AddLast(newSegment);
            prevSegment = newSegment;
        }

        var (tailSuccess, tail) = CreateSegment(tailPrefab, prevSegment);
        if (tailSuccess)
            _body.AddLast(tail);
    }

    private (bool, SnakeSegment) CreateSegment(GameObject prefab, SnakeSegment prevSegment)
    {
        var go = Instantiate(prefab, prevSegment.transform.position,
               prevSegment.transform.rotation, _head.transform);
        var newSegment = go.GetComponent<SnakeSegment>();
        newSegment.Init(gravityCenter);
        
        var (positionSuccess, position, direction, closestGround) = newSegment.FindNextPosition(
            _groundCheckColliders,
            prevSegment.HalfLength + newSegment.HalfLength + bodySegmentsPadding,
            -prevSegment.transform.forward
        );

        if (!positionSuccess)
        {
            Debug.LogWarning("Can't position body segment on start");
            return default;
        }

        var (alignSuccess, targetRotation) = AlignToGroundNormal(
            -direction, position, closestGround);

        if (!alignSuccess)
        {
            Debug.LogWarning("Can't find ground for body segment on start");
            return default;
        }

        newSegment.transform.SetPositionAndRotation(position,
            targetRotation);
        return (true, newSegment);
    }
}
