using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DuplicateCamera : MonoBehaviour
{
    public Camera otherCamera;
    private Camera local_camera;

    /**
     * Initializes the camera components
     */
    void Start()
    {
        local_camera = GetComponent<Camera>();

        if (otherCamera == null)
        {
            Debug.LogError("No camera to duplicate");
            enabled = false;
            return;
        }

        // Update the background and other settings

    }

    /**
     * Update the camera attached to this object to be the same as the camera chosen
     */
    void LateUpdate()
    {
        local_camera.fieldOfView = otherCamera.fieldOfView;
        local_camera.aspect = otherCamera.aspect;
        local_camera.nearClipPlane = otherCamera.nearClipPlane;
        local_camera.farClipPlane = otherCamera.farClipPlane;
    }
}
