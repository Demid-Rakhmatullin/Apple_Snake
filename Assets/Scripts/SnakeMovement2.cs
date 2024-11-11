using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

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
    private float _snakeHalfHeight;
    private float _surfaceSmoothing; //ignore surface unevenness <= this value
    private Collider[] _groundCheckColliders;

    private RaycastHit[] _hits;
    float time;

    private void Awake()
    {
        APPLE_LAYER = LayerMask.GetMask("Apple");
        _rigidbody = GetComponent<Rigidbody>();
        _collider = GetComponent<BoxCollider>();
        _groundCheckColliders = new Collider[10];           
    }

    void Start()
    {
        _snakeHalfHeight = _collider.size.y / 2f;
        _surfaceSmoothing = _snakeHalfHeight * 0.2f;

        _hits = new RaycastHit[10];
        Debug.Log("Before " + transform.forward);

        var appleDir = GetAppleCenterDirection();
        if (Physics.Raycast(transform.position, appleDir, out RaycastHit hit,
            100f, APPLE_LAYER))
        {
            //Debug.DrawLine(transform.position, hit.point, Color.red, 100f);
            //Debug.Log("Apple found");

            _rigidbody.position = hit.point + GetHeightMargin(appleDir);
            _rigidbody.rotation = GetSurfaceAligninigRotation(hit.normal) * _rigidbody.rotation;
            _moving = true;
            Debug.Log("After " + transform.forward);
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

        var currentPosition = _rigidbody.position;
        var moveDirection = transform.forward;
        var moveDistance = moveSpeed * Time.fixedDeltaTime;

        var pathIsClear = !Physics.Raycast(currentPosition, moveDirection, out _,
            moveDistance, APPLE_LAYER);

        if (pathIsClear)
        {
            var nextPosition = currentPosition + moveDistance * moveDirection;
            var groundCheck = Physics.OverlapSphereNonAlloc(nextPosition, _snakeHalfHeight + _surfaceSmoothing,
                _groundCheckColliders, APPLE_LAYER);

            if (groundCheck > 0)
            {

            }

            Debug.DrawLine(currentPosition, nextPosition, Color.red);
        }

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
    }   
    
    private Vector3 GetAppleCenterDirection()
        => (appleCenter.position - transform.position).normalized;

    private Vector3 GetHeightMargin(Vector3 centerDirection)
        => -centerDirection * _snakeHalfHeight;

    private Quaternion GetSurfaceAligninigRotation(Vector3 surfaceNormal)
        => Quaternion.FromToRotation(transform.up, surfaceNormal);
}
