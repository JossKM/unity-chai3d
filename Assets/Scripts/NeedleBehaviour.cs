using UnityEngine;
using System.Collections.Generic;

public struct NeedlePunctureData
{
    public Vector3 entryPoint;
    public Vector3 entrySurfaceNormal;
    public Vector3 punctureVector; // displacement from entry point to tip (or exit point)
}

public class NeedleBehaviour : MonoBehaviour
{
    // Properties /////////////////////////////////////////////////
    public float needleLength = 0.1f;

    // how far ahead to cast rays beyond the needle itself
    public float castAheadLength = 0.01f;

    //public HapticAxialConstraint axialConstraint;
    public Ray needleRay;
    public NeedlePunctureData punctureData;

    // applies when penetrating
    public HapticAxialConstraint constraint;

    public bool isPuncturing = false;

    // Use this for initialization
    void Start()
    {
        needleRay = new Ray();
    }

    // Update is called once per frame
    void Update()
    {
        float castLength = needleLength + castAheadLength;

        Vector3 direction;
        direction = transform.TransformDirection(Vector3.forward).normalized;

        needleRay.origin = transform.position - (direction * needleLength);
        needleRay.direction = direction * castLength;

        //raycast to determine what is in the needle's path
        RaycastHit[] hits = Physics.RaycastAll(needleRay, needleLength);

        //ensure only haptic objects are evaluated, and sort them by distance
        LinkedList<RaycastHit> sortedHits = SortAndFilterPenetrations(hits);
        int numLayers = sortedHits.Count;

        // Tell plugin how many layers were encountered
        HapticNativePlugin.setPatientNumLayersToUse(numLayers);

        // iterate through layers, in order of closest to farthest
        var iterator = sortedHits.First;
        for (int i = 0; i < numLayers; i++)
        {
            RaycastHit hit = iterator.Value;

            double punctureDepth;

            if (i == 0) // for the first collision
            {
                punctureData.entryPoint = hit.point;
                punctureData.entrySurfaceNormal = hit.normal;
                punctureDepth = 0.0;

                // move the constraint position as long as the needle is not inside anything
                if (!isPuncturing) // they ray casts a little farther than the needle itself
                {
                    constraint.transform.position = punctureData.entryPoint;

                    if (hit.distance < needleLength) // if needle has hit the skin
                    {
                        isPuncturing = true;
                        constraint.constraintEnabled = true;
                    }
                }

            }
            else
            {
                punctureDepth = hit.distance - (sortedHits.First.Value.distance);  // depth of penetration from the first contact point
            }

            // Send new layer data to plugin
            PenetrableMaterial material = hit.collider.gameObject.GetComponent<PenetrableMaterial>();

            HapticNativePlugin.setHapticLayerProperties(
               i,
               material.m_stiffness,
               material.m_stiffnessExponent,
               material.m_maxFrictionForce,
               material.m_penetrationThreshold,
               material.m_malleability,
               punctureDepth);

            iterator = iterator.Next;
        }


        HapticNativePlugin.setPatientLayersEnabled(true);
        if (sortedHits.Count > 0)
        {
            Vector3 entryPoint = punctureData.entryPoint;

            // debug drawing
            {
                // unpenetrating portion
                Debug.DrawLine(needleRay.origin,
                    entryPoint,
                    Color.green);

                // penetrating portion
                Debug.DrawLine(entryPoint, //needleRay.origin + needleRay.direction * ((RaycastHit)sortedHits[0]).distance,
                    transform.position, //needleRay.direction * (float)punctureData.collisions[(punctureData.collisions.Length - 1)].depth, 
                    Color.red);

                // surface normal
                Debug.DrawRay(entryPoint,
                    punctureData.entrySurfaceNormal * 0.01f,
                    Color.yellow);
            }
            
        }
        else
        {
            isPuncturing = false;
            constraint.constraintEnabled = false;
            //HapticNativePlugin.setPatientLayersEnabled(false);

            // debug drawing
            {
                Debug.DrawRay(needleRay.origin, needleRay.direction * needleLength, Color.green);
            }
        }
    }

    // ensures only haptic objects are returned by the raycast, and also sorts them by distance
    LinkedList<RaycastHit> SortAndFilterPenetrations(RaycastHit[] hits)
    {
        LinkedList<RaycastHit> hitList = new LinkedList<RaycastHit>();
        
        float maxDistance = Mathf.NegativeInfinity;

        for(int i = 0; i < hits.Length; i++)
        {
            // if it is a haptic material, 
            if (hits[i].collider.gameObject.GetComponent<PenetrableMaterial>() != null)
            {
                // find its place, then add it to the list there

                if (hits[i].distance > maxDistance) // if more than the maximum, add it to the end of the list. Also handles the initial case, of adding the first node.
                {
                    hitList.AddLast(hits[i]);
                    maxDistance = hits[i].distance;
                } else
                {
                    var node = hitList.Last.Previous;

                    while (true)
                    {
                        if (node == null)
                        {
                            // it is the first hit. Add it at the beginning.
                            hitList.AddFirst(hits[i]);
                            break;
                        }
                        else
                        {
                            if(hits[i].distance < node.Value.distance)
                            {
                                // the correct place for it is found. Add it here.
                                hitList.AddAfter(node, hits[i]);
                                break;
                            }
                        }

                        node = node.Previous;
                    }
                }
            }
        }

        return hitList;
    }
}
