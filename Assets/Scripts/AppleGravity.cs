using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AppleGravity : MonoBehaviour
{
    [SerializeField] private Collider gravityCollider;
    [SerializeField] private float gravityForce = 9.81f;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTriggerStay(Collider other)
    {
        //var center = gravityCollider.bounds.center;
        //var direction = (transform.position - other.transform.position).normalized;
        //string s = "grav: " + other.name + ", rb: " + (other.attachedRigidbody != null).ToString();
        //Debug.Log(s);
        //if (other.attachedRigidbody != null)
        //{
        //    other.attachedRigidbody.AddForce(direction * gravityForce * Time.fixedDeltaTime);
        //    Debug.Log("grav move");
        //}
    }
}
