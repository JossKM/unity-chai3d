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

    // for debugging. will scale accorfing to the chai3d haptic world scale
    public GameObject workspaceVisualizer;

    // shows the device or "goal" position. Different from the proxy position reflected in this script's parent gameObject
    public GameObject devicePositionGameObject;

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

        if(devicePositionGameObject!= null)
             devicePositionGameObject.transform.localPosition = HapticNativePlugin.GetDevicePosition();
    }
    
}