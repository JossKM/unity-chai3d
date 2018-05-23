using UnityEngine;
using System.Collections;

public class CameraControl : MonoBehaviour
{

    public GameObject WorkspaceOriginGameObject;
    public float moveSpeed = 0.5f;

    private float rotationX = 0f;
	private float rotationY = 0f;

    private Vector3 previousPosition;
    private Vector3 originalPosition;

    private Vector3 previousRotation;

    private Transform referenceTransform;

    private void Start()
    {
        referenceTransform = WorkspaceOriginGameObject.transform;
        referenceTransform.parent = transform;
        previousPosition = referenceTransform.position;
        originalPosition = referenceTransform.position;
        referenceTransform.localPosition = new Vector3(0f, 0f, 1f);//referenceTransform.position - transform.position; //new Vector3(0f, 0f, 1f);
        UpdateHapticPosition();
    }

	// Update is called once per frame
	void Update () {

		float scroll = Input.GetAxis("Mouse ScrollWheel");
		transform.Translate(0, 0f, scroll * moveSpeed, Space.Self);

		if (Input.GetMouseButton (1)) {
			rotationX += Input.GetAxis ("Mouse X") * 50f * Time.deltaTime;
			rotationY += Input.GetAxis ("Mouse Y") * 50f * Time.deltaTime;
			transform.localEulerAngles = new Vector3 (-rotationY, rotationX, 0);
		}
		else if (Input.GetMouseButton (2)) {
			var xMove = Input.GetAxis ("Mouse X") * -1f * Time.deltaTime;
			var yMove = Input.GetAxis ("Mouse Y") * -1f * Time.deltaTime;
			transform.Translate(xMove, yMove, 0f, Space.Self);
		}

		if (Input.GetKey(KeyCode.UpArrow))
			transform.Translate(0f, 0f, moveSpeed * Time.deltaTime, Space.World);
		if (Input.GetKey(KeyCode.DownArrow))
			transform.Translate(0f, 0f, -moveSpeed * Time.deltaTime, Space.World);
		if (Input.GetKey(KeyCode.LeftArrow))
			transform.Translate(-moveSpeed * Time.deltaTime, 0f, 0f, Space.World);
		if (Input.GetKey(KeyCode.RightArrow))
			transform.Translate(moveSpeed * Time.deltaTime, 0f, 0f, Space.World);
	    if (Input.GetKey(KeyCode.Space))
	        transform.Translate(0f, moveSpeed * Time.deltaTime, 0f, Space.World);
	    if (Input.GetKey(KeyCode.LeftControl))
	        transform.Translate(0f, -moveSpeed * Time.deltaTime, 0f, Space.World);

        UpdateHapticPosition();
        UpdateHapticRotation();
	}

    private void UpdateHapticPosition()
    {
        if (referenceTransform == null)
            return;
        if (previousPosition != referenceTransform.position)
        {
            HapticNativePlugin.SetHapticPosition((referenceTransform.position - originalPosition) / 0.1f);
        }
        previousPosition = referenceTransform.position;
    }

    private void UpdateHapticRotation()
    {
        if (previousRotation != this.transform.rotation.eulerAngles)
        {
            HapticNativePlugin.SetHapticRotation(this.transform.rotation.eulerAngles);
        }
        previousRotation = this.transform.rotation.eulerAngles;
    }

}
