using System;
using UnityEngine;
using UnityEngine.UI;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using System.Runtime.InteropServices;

public class ROS2_Image_Subscriber : MonoBehaviour
{
    public string topicName = "/image"; // ROS topic to subscribe to
    public RawImage imageDisplay; // UI element to display the image

    private ROSConnection ros;
    private Texture2D texture2D;

    // Start is called before the first frame update
    void Start()
    {
        // Get the ROSConnection instance
        ros = ROSConnection.GetOrCreateInstance();

        // Initialize the texture with an arbitrary size, it will be resized based on the incoming image
        texture2D = new Texture2D(2, 2);

        // Subscribe to the image topic
        ros.Subscribe<ImageMsg>(topicName, ReceiveImage);
    }

    // Callback function to process received image messages
    void ReceiveImage(ImageMsg imageMessage)
    {
        // Get the width and height from the message
        int width = (int) imageMessage.width;
        int height = (int) imageMessage.height;

        // Update the texture size if necessary
        if (texture2D.width != width || texture2D.height != height)
        {
            texture2D.Reinitialize(width, height);
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

        texture2D.SetPixels32(pixels);
        texture2D.Apply();

        // Set the texture to the RawImage UI component
        imageDisplay.texture = texture2D;
    }
}
