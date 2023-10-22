using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SnakeMovement : MonoBehaviour
{
    [SerializeField] private Collider gravityCollider;
    [SerializeField] private new Rigidbody rigidbody;
    [SerializeField] private CharacterController characterController;
    [SerializeField] private float speed;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    private Vector3 playerVelocity;

    // Update is called once per frame
    void Update()
    {
        //var center = gravityCollider.bounds.center;
        //var direction = (center - transform.position).normalized;
        var grDir = (gravityCollider.transform.position - transform.position).normalized;


        //if (characterController.isGrounded && playerVelocity != Vector3.zero)
        //{
        //    playerVelocity = Vector3.zero;
        //    Debug.Log("grounded: " + characterController.isGrounded);
        //    //playerVelocity.y = 0f;
        //    //playerVelocity.y = -9.81f * Time.deltaTime;
        //}

        if (Physics.Raycast(transform.position, -transform.up, out RaycastHit hit))
        {
            var slopeRotation = Quaternion.FromToRotation(transform.up, hit.normal);
            //var excludeY = Quaternion.Euler(slopeRotation.eulerAngles.x, 0f, slopeRotation.eulerAngles.z);
            //var slerped = Quaternion.Slerp(transform.rotation, slopeRotation, 10f * Time.deltaTime);

            transform.rotation = slopeRotation * transform.rotation;

            //var zazor = hit.distance - (characterController.height / 2f);
            //if (zazor > 0.1f)
            //{
            //    transform.Translate(hit.point);
            //    //characterController.Move(10f * Time.deltaTime * -transform.up);
            //    //transform.position = transform.position - Vector3.up * zazor;
            //    Debug.Log("y moved");
            //}
            //transform.rotation = Quaternion.Slerp(transform.rotation, slopeRotation, 5f * Time.deltaTime);
            //transform.up = info.normal;
            //Debug.Log(/*"normal: " + hit.normal + */", object: " + hit.collider.name);

            var dir = speed * transform.forward + (-hit.normal * 9.8f);
            characterController.Move(dir * Time.deltaTime);
        }

        
        //characterController.Move(speed * Time.deltaTime * transform.forward);

        //var dest = speed * Time.deltaTime * transform.forward;
        //var check = Physics.OverlapSphere(dest, characterController.height / 2f + 0.1f);
        //while (!check.Any(c => c.name == "Apple"))
        //{
        //    dest = Vector3.Slerp(dest, gravityCollider.transform.position, 0.01f);
        //    check = Physics.OverlapSphere(dest, characterController.height / 2f + 0.1f);
        //}

        //characterController.Move(dest);      

        //var groundHits = Physics.OverlapSphere(transform.position, characterController.height / 2f + 0.1f);
        //if (!groundHits.Any(c => c.name == "Apple"))
        //    characterController.Move(grDir * (9.81f) * Time.deltaTime);
        //else
        //    characterController.Move(speed * Time.deltaTime * transform.forward);
        // Debug.Log("Hit! " + string.Join(",", groundHits.Select(c => c.name).ToArray()));

        //playerVelocity += direction * (9.81f) * Time.deltaTime;
        //characterController.Move(direction * (9.81f) * Time.deltaTime);

    }

    private void FixedUpdate()
    {
        //rigidbody.AddForce(transform.forward * speed * Time.fixedDeltaTime, ForceMode.Acceleration);
    }
}
