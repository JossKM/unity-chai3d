using UnityEngine;
using System.Collections;

// uses the object's position and orientation for the constraints. Only 1 may be active. Not to be attached to the proxy.

public class HapticAxialConstraint : MonoBehaviour
{
    [Header("Press C to toggle")]
    public bool enabled = false;
    public double minDist = 0.0d;
    public double maxDist = 0.2d;
    public double maxForce = 2.0d;

    private Transform origin;

    public Transform constraintSlave;

    private Quaternion originalSlaveOrientation;

    void Awake()
    {
        origin = GameObject.Find("Haptic Origin").transform;
    }

	void Start ()
	{
	    originalSlaveOrientation = constraintSlave.rotation;
	}
	
	void Update ()
	{
	    if (Input.GetKeyUp(KeyCode.C))
	    {
	        enabled = !enabled;

	    }

	    HapticNativePlugin.SetAxialConstraint(enabled, transform.position - origin.position, transform.forward, minDist, maxDist, maxForce);
	    constraintSlave.rotation = originalSlaveOrientation * transform.rotation;
	}
}


