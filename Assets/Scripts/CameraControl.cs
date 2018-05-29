using UnityEngine;
using System.Collections;

public class CameraControl : MonoBehaviour
{
    public float moveSpeed = 1.0f;
    public Transform workspace;

	private float rotationX = 0f;
	private float rotationY = 0f;

    private Vector3 previousPosition;
    private Vector3 originalPosition;

    private Vector3 previousRotation;


    private void Start()
    {
        workspace.parent = this.transform;
        previousPosition = workspace.position;
        originalPosition = workspace.position;
        UpdateHapticPosition();
    }

	// Update is called once per frame
	void Update () {

		float scroll = Input.GetAxis("Mouse ScrollWheel");
		transform.Translate(0, 0f, scroll * 0.5f * moveSpeed, Space.Self);

		if (Input.GetMouseButton (1)) {
			rotationX += Input.GetAxis ("Mouse X") * moveSpeed * 50f * Time.deltaTime;
			rotationY += Input.GetAxis ("Mouse Y") * moveSpeed * 50f * Time.deltaTime;
			transform.localEulerAngles = new Vector3 (-rotationY, rotationX, 0);
		}
		else if (Input.GetMouseButton (2)) {
			var xMove = Input.GetAxis ("Mouse X") * -moveSpeed * Time.deltaTime;
			var yMove = Input.GetAxis ("Mouse Y") * -moveSpeed * Time.deltaTime;
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
        if (workspace == null)
            return;
        if (previousPosition != workspace.position)
        {
            HapticNativePlugin.SetHapticPosition((workspace.position - originalPosition));
        }
        previousPosition = workspace.position;
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
