using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System.IO;
using System;
using System.Threading;

public class HapticManager : MonoBehaviour
{
    public double worldScale = 1.0d;

    public bool useHaptic;
    public static bool isHapticAvail;
   
    private Vector3 originalPosition;
    public GameObject workspaceVisualizer;

    // Use this for initialization
    private void Awake()
    {
        if (!useHaptic)
            isHapticAvail = false;

        isHapticAvail = HapticNativePlugin.prepareHaptics(worldScale);

        if (workspaceVisualizer != null)
        {
            workspaceVisualizer.transform.localScale = new Vector3((float)worldScale, (float)worldScale, (float)worldScale) * 2.0f;
        }
    }


    private void Start()
    {
        if(isHapticAvail)
            HapticNativePlugin.startHaptics();
    }

    private void OnDestroy()
    {
        if (isHapticAvail)
            HapticNativePlugin.stopHaptics();
    }

    private void Update()
    {
        if (isHapticAvail)
            this.transform.localPosition = HapticNativePlugin.GetProxyPosition();
    }
    
}