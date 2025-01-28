using UnityEngine;

public class GameConfig : MonoBehaviour
{
    public static GameConfig Instance { get; private set; }

    void Awake()
    {
        if (Instance != null)
        {
            Destroy(this);
            return;
        }

        GroundLayer = LayerMask.GetMask(GroundLayerName);
        Instance = this;
    }

    [field: SerializeField]
    public string GroundLayerName { get; private set; }

    public int GroundLayer { get; private set; }
}
