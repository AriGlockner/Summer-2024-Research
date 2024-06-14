using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraLayers : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        // Assign your layers here
        int leftLayer = LayerMask.NameToLayer("LeftEye");
        int rightLayer = LayerMask.NameToLayer("RightEye");

        // Set up left and right cameras
        Camera leftCamera = new GameObject("Left Camera").AddComponent<Camera>();
        Camera rightCamera = new GameObject("Right Camera").AddComponent<Camera>();

        // Configure the cameras for each eye
        leftCamera.cullingMask = 1 << leftLayer;
        rightCamera.cullingMask = 1 << rightLayer;

        // Set the target eye for each camera
        leftCamera.stereoTargetEye = StereoTargetEyeMask.Left;
        rightCamera.stereoTargetEye = StereoTargetEyeMask.Right;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
