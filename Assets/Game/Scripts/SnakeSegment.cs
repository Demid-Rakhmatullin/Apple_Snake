using UnityEngine;
using PH = PhysicsHelpers;

[RequireComponent(typeof(Rigidbody), typeof(BoxCollider))]
public class SnakeSegment : MonoBehaviour
{
    [Tooltip("Surface unevenness of this size will be ignored. Measured in percentages of collider height")]
    [SerializeField] float surfaceSmoothingPercent = 10f;

    private Transform _gravityCenter;
    private BoxCollider _collider;
    private float _surfaceSmoothingOut;
    private float _surfaceSmoothingIn;

    public Rigidbody Rigidbody { get; private set; }
    public float HalfHeight { get; private set; }
    public float HalfLength { get; private set; }

    public void Init(Transform gravityCenter)
    {
        _collider = GetComponent<BoxCollider>();
        Rigidbody = GetComponent<Rigidbody>();

        HalfLength = _collider.size.z / 2f;
        HalfHeight = _collider.size.y / 2f;

        var surfaceSmoothing = _collider.size.y * surfaceSmoothingPercent / 100f;
        _surfaceSmoothingOut = HalfHeight + surfaceSmoothing;
        _surfaceSmoothingIn = HalfHeight - surfaceSmoothing;

        _gravityCenter = gravityCenter;
    }

    public (bool success, Vector3 nextPostion, Vector3 moveDirection, Vector3 closestGround) FindNextPosition(
        Collider[] colliders, float moveDistance, Vector3 initialDirection)
    {
        var moveDirection = initialDirection;
        var nextPostion = GetPosition(moveDistance, moveDirection);
        var validatePosition = CheckGround(
            colliders, nextPostion, out Vector3 closestGround);

        //rotate move direction upwards or downwards
        if (validatePosition != CheckGroundResult.Success)
        {
            const int MAX_ATTEMTS = 90;
            const float ROTATION_STEP = 1f * Mathf.Deg2Rad;

            var gravityDir = GetGravityDirection();
            var verticalDelta = validatePosition switch
            {
                CheckGroundResult.InsideGround or CheckGroundResult.GroundTooClose
                    => -ROTATION_STEP,  //"climb a hill"
                CheckGroundResult.GroundNotFound
                    => ROTATION_STEP,   //"go down the hill"
                _ => throw new System.InvalidOperationException("Can't process ValidateDirectionResult!")
            };

            for (int i = 0; i < MAX_ATTEMTS; i++)
            {
                moveDirection = Vector3.RotateTowards(moveDirection, gravityDir, verticalDelta, 0f);
                nextPostion = GetPosition(moveDistance, moveDirection);
                validatePosition = CheckGround(colliders,nextPostion, out closestGround);

                if (validatePosition == CheckGroundResult.Success)
                    break;
            }
        }

        if (validatePosition != CheckGroundResult.Success)
            return default;

        return (true, nextPostion, moveDirection, closestGround);
    }

    public CheckGroundResult CheckGround(Collider[] colliders, 
        Vector3 position, out Vector3 closestGround)
    {
        closestGround = position;

        var groundCheck = Physics.OverlapSphereNonAlloc(position, _surfaceSmoothingOut,
            colliders, GameConfig.Instance.GroundLayer);

        if (groundCheck > 0)
        {
            for (int i = 0; i < groundCheck; i++)
            {
                closestGround = colliders[i].ClosestPoint(position);

                if (position.Equals(closestGround))
                    return CheckGroundResult.InsideGround;

                if (Vector3.Distance(closestGround, position) < _surfaceSmoothingIn)
                    return CheckGroundResult.GroundTooClose;
            }

            return CheckGroundResult.Success;
        }

        return CheckGroundResult.GroundNotFound;
    }

    public enum CheckGroundResult
    {
        Success,
        GroundNotFound,
        GroundTooClose,
        InsideGround
    }

    public Quaternion FindRoughNextRotation(Collider[] colliders, Vector3 nextPosition, Vector3 nextForward,
        float groundAlignMaxDelta)
    {
        var nextRotation = Quaternion.LookRotation(nextForward, -GetGravityDirection(nextPosition));

        Quaternion groundRotation = default;
        switch (CheckGround(colliders, nextPosition, out Vector3 closestGround))
        {
            case CheckGroundResult.Success:
            case CheckGroundResult.GroundTooClose:
                var (success, rot) = PH.AlignToGroundNormal(nextForward, nextPosition, closestGround);
                if (success)
                    groundRotation = rot;
                else
                    Debug.LogWarning("Can't align segment to ground normal (rough rotation)");
                break;
            case CheckGroundResult.GroundNotFound:
                if (PH.RaycastGround(nextPosition, GetGravityDirection(nextPosition),
                        out RaycastHit hit))
                    groundRotation = PH.AlignToGroundNormal(nextForward, hit.normal);
                else
                    Debug.LogWarning("Can't find ground for segment (rough rotation)");
                break;
        }

        if (!groundRotation.Equals(default))
        {
            nextRotation = Quaternion.RotateTowards(
                nextRotation,
                groundRotation,
                groundAlignMaxDelta);
        }

        return nextRotation;
    }

    public Quaternion AlignToGroundNormal(Vector3 groundNormal)
        => PH.AlignToGroundNormal(transform.forward, groundNormal);

    public Vector3 GetPosition(float distance, Vector3 direction)
        => transform.position + distance * direction;

    public Vector3 GetGravityDirection()
        => GetGravityDirection(transform.position);

    public Vector3 GetGravityDirection(Vector3 currentPostion)
        => (_gravityCenter.position - currentPostion).normalized;

    public bool RaycastGround(Vector3 direction, out RaycastHit groundHit)
        => PH.RaycastGround(transform.position, direction, out groundHit);
}
