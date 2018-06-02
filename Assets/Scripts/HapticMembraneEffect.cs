using UnityEngine;
using System.Collections;
using UnityEngine.Assertions;

[RequireComponent(typeof(TouchableObject))]
public class HapticMembraneEffect : MonoBehaviour
{
    private int objectId;

    public double resistance = 9.0;
    public double friction_static = 0.2;
    public double friction_dynamic = 0.5;
    public double maxForce = 9.0;
    public double springMass = 1.0;
    public double distanceToMaxForce = 0.01;


    // Use this for initialization
    void Start ()
    {
        Assert.AreNotEqual(springMass, 0.0);

        objectId = GetComponent<TouchableObject>().objectId;

        HapticNativePlugin.addViscosityEffect(objectId, GetComponent<TouchableObject>().viscosity);
        HapticNativePlugin.addMembraneEffect(objectId, resistance, friction_static, friction_dynamic, maxForce, distanceToMaxForce, springMass);

    }
	
	// Update is called once per frame
	void Update ()
	{
    }
}
