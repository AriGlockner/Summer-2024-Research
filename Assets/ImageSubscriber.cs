using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class ImageSubscriber : MonoBehaviour
{
    // ROS Topics
    public string leftEyeTopic = "/image2";
    public string rightEyeTopic = "/image1";

    // Render Textures
    public RenderTexture leftEyeRenderTexture;
    public RenderTexture rightEyeRenderTexture;
    private Texture2D leftEyeTexture2D;
    private Texture2D rightEyeTexture2D;

    // ROS Connection
    private ROSConnection ros;

    void Start()
    {
        // Get the ROSConnection instance
        ros = ROSConnection.GetOrCreateInstance();

        // Initialize the textures with an arbitrary size, they will be resized based on the incoming images
        leftEyeTexture2D = new Texture2D(2, 2);
        rightEyeTexture2D = new Texture2D(2, 2);

        // Subscribe to the image topics
        ros.Subscribe<ImageMsg>(leftEyeTopic, ReceiveLeftEyeImage);
        ros.Subscribe<ImageMsg>(rightEyeTopic, ReceiveRightEyeImage);
    }

    // Update is called once per frame
    void Update()
    {
        
    }


    // Callback function to process received left eye image messages
    void ReceiveLeftEyeImage(ImageMsg imageMessage)
    {
        UpdateTexture(imageMessage, leftEyeTexture2D, leftEyeRenderTexture);
    }

    // Callback function to process received right eye image messages
    void ReceiveRightEyeImage(ImageMsg imageMessage)
    {
        UpdateTexture(imageMessage, rightEyeTexture2D, rightEyeRenderTexture);
    }

    // Function to update texture and render texture
    void UpdateTexture(ImageMsg imageMessage, Texture2D texture, RenderTexture renderTexture)
    {
        // Get the width and height from the message
        int width = (int) imageMessage.width;
        int height = (int) imageMessage.height;

        // Update the texture size if necessary
        if (texture.width != width || texture.height != height)
        {
            texture.Reinitialize(width, height);
        }

        // Copy the image data to the texture
        byte[] imageData = imageMessage.data;
        Color32[] pixels = new Color32[width * height];

        for (int i = 0; i < pixels.Length; i++)
        {
            byte r = imageData[i * 3];
            byte g = imageData[i * 3 + 1];
            byte b = imageData[i * 3 + 2];
            pixels[i] = new Color32(r, g, b, 255);
        }

        texture.SetPixels32(pixels);
        texture.Apply();

        // Copy the texture to the render texture
        RenderTexture.active = renderTexture;
        Graphics.Blit(texture, renderTexture);
        RenderTexture.active = null;
    }
}
