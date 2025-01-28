
using UnityEngine;

public static class PhysicsHelpers
{

    public static bool RaycastGround(Vector3 origin, Vector3 direction, out RaycastHit groundHit)
        => Physics.Raycast(origin, direction, out groundHit, Mathf.Infinity, GameConfig.Instance.GroundLayer);

    public static Quaternion AlignToGroundNormal(Vector3 forward, Vector3 groundNormal)
        => Quaternion.LookRotation(forward, groundNormal);

    public static (bool success, Quaternion rotation) AlignToGroundNormal(
        Vector3 forward, Vector3 position, Vector3 surfacePoint)
    {
        var surfaceDir = (surfacePoint - position).normalized;
        if (!RaycastGround(position, surfaceDir, out RaycastHit surfaceHit))
            return (false, default);

        return (true, AlignToGroundNormal(forward, surfaceHit.normal));
    }
}
