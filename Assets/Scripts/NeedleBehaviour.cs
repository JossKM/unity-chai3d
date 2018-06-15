using UnityEngine;
using System.Collections;

public struct NeedlePunctureData
{
    public Vector3 entryPoint;
    public Vector3 entrySurfaceNormal;
    public Vector3 punctureVector; // displacement from entry point to tip (or exit point)
    public NeedlePuncture[] collisions;
}

public struct NeedlePuncture
{
    //public int materialType;
    public double depth;
}

public class NeedleBehaviour : MonoBehaviour
{
    // Properties /////////////////////////////////////////////////
    public float needleLength = 0.1f;

    // how far ahead to cast rays beyond the needle itself
    public float castAheadLength = 0.02f;

    //public HapticAxialConstraint axialConstraint;
    public Ray needleRay;
    public NeedlePunctureData punctureData;

    // applies when penetrating
    public HapticAxialConstraint constraint;

    public bool isPuncturing = false;

    // Use this for initialization
    void Start () {
        needleRay = new Ray();
    }

	// Update is called once per frame
	void Update ()
    {
        float castLength = needleLength + castAheadLength;

        Vector3 direction;
        direction = transform.TransformDirection(Vector3.forward).normalized;

        needleRay.origin = transform.position - (direction * needleLength);
        needleRay.direction = direction * castLength;

        //raycast to determine what is in the needle's path
        RaycastHit[] hits = Physics.RaycastAll(needleRay, needleLength);

        //ensure only haptic objects are evaluated, and sort them by distance
        ArrayList sortedHits = SortAndFilterPenetrations(hits);
        
        punctureData.collisions = new NeedlePuncture[sortedHits.Count];

        for (int i = 0; i < sortedHits.Count; i++)
        {
            NeedlePuncture materialEntry;
            RaycastHit hit = (RaycastHit)sortedHits[i];
        
            if (i == 0) // for the first collision
            {
                punctureData.entryPoint = hit.point;
                punctureData.entrySurfaceNormal = hit.normal;
                materialEntry.depth = 0.0;
               
                // move the constraint position as long as the needle is not inside anything
                if(!isPuncturing)
                {
                    constraint.transform.position = punctureData.entryPoint;
                }
                isPuncturing = true;
                constraint.enabled = true;

            } else
            {
                materialEntry.depth = hit.distance - ((RaycastHit)sortedHits[0]).distance;  // depth of penetration
            }
        
            punctureData.collisions[i] = materialEntry;
        }


        // debug drawing
        if(sortedHits.Count > 0)
        {
            Vector3 entryPoint = punctureData.entryPoint;

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
        } else
        {
            isPuncturing = false;
            constraint.enabled = false;

            Debug.DrawRay(needleRay.origin, needleRay.direction * needleLength, Color.green);
        }
    }

    // ensures only haptic objects are returned, and also sorts them by distance
    ArrayList SortAndFilterPenetrations(RaycastHit[] hits)
    {
        ArrayList hitList = new ArrayList(hits);

        for (int i = hitList.Count - 1; i >= 0; i--)
        {
            // if not a haptic object, remove it. It is not part of the haptic simulation
            if (((RaycastHit)hitList[i]).collider.gameObject.GetComponent<PenetrableMaterial>() == null)
            {
                hitList.Remove(i);
            }
            //else
            //{
            //    Debug.Log("skin collision!");
            //}
        }
        
        hitList.Sort();

        return hitList;
    }
}
