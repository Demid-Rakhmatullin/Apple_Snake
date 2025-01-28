using UnityEngine;

public class SnakeFollowCamera : MonoBehaviour
{
    [SerializeField] float verticalOffset = 100f;

    private Transform _snakeHead;
    private Transform _apple;
    private bool _follow;

    public void Init(Transform snakeHead, Transform apple)
    {
        _snakeHead = snakeHead;
        _apple = apple;

        var direction = (_snakeHead.position - _apple.position).normalized;
        var position = _apple.position + direction * verticalOffset;

        transform.position = position;

        var dot1_abs = Mathf.Abs(Vector3.Dot(Vector3.up, direction));
        var dot2_abs = Mathf.Abs(Vector3.Dot(Vector3.forward, direction));
        var worldUp = dot1_abs < dot2_abs ? Vector3.up : Vector3.forward;

        transform.LookAt(_snakeHead.position, worldUp);
        _follow = true;
    }

    void LateUpdate()
    {
        if (!_follow)
            return;

        var snakeDirection = (_snakeHead.position - _apple.position).normalized;
        var nextPosition = _apple.position + snakeDirection * verticalOffset;

        transform.position = nextPosition;
        transform.LookAt(_snakeHead.position, transform.up);
    }
}
