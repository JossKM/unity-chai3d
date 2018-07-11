using UnityEngine;
using System.Collections;

public class PenetrableMaterial : MonoBehaviour {

    public double   m_stiffness = 6000.0;
    public double   m_stiffnessExponent = 1.5;

	public double 	m_maxFrictionForce = 0.1;

	public double 	m_penetrationThreshold = 8.0;
	public double 	m_malleability = 1.0;

	// Use this for initialization
	void Start ()
    {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
