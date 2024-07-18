using System;
using UnityEngine;
using ROS2;
using System.Collections.Concurrent;

namespace ROS2
{
    /// <summary>
    /// Subscribe image messages in ROS2
    /// </summary>
    public class ROS2ImageSubscriber : MonoBehaviour
    {
        // ROS2 Connection components
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private ISubscription<sensor_msgs.msg.Image> image_sub;
        public String topic_Name = "chatter";
        
        // Textures / Objects
        private Texture2D texture;
        public GameObject quad;
        private Renderer quadRenderer;

        // Queue
        private ConcurrentQueue<sensor_msgs.msg.Image> imageQueue = new ConcurrentQueue<sensor_msgs.msg.Image>();


        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();            
            quadRenderer = quad.GetComponent<Renderer>();
        }

        void Update()
        {
            // Subscribe the data from the incoming image
            if (ros2Node == null && ros2Unity.Ok())
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityImageListenerNode");
                image_sub = ros2Node.CreateSubscription<sensor_msgs.msg.Image>(
                    topic_Name, msg => {
                        if (msg.Data.Length > 0)
                            imageQueue.Enqueue(msg);
                        else
                            Debug.LogWarning("Recieved image message with empty data");
                    });
            }

            // Display the image onto the object
            displayImage();
        }

        void displayImage()
        {
            //texture = new Texture2D(640, 480, TextureFormat.RGBAHalf, false);

            if (imageQueue.TryDequeue(out sensor_msgs.msg.Image msg))
            {
                // Create a new Texture if the size of the texture is not correct
                int width = (int) msg.Width;
                int height = (int) msg.Height;
                if (texture == null || texture.width != width || texture.height != height)
                    texture = new Texture2D(width, height, TextureFormat.RGB24, false);

                // Create and apply the texture
                Color32[] pixels = new Color32[width * height];
                for (int i = 0; i < pixels.Length; i++)
                {
                    int idx = i * 3;
                    pixels[i] = new Color32(msg.Data[idx], msg.Data[idx + 1], msg.Data[idx + 2], 255);
                }

                texture.SetPixels32(pixels);
                texture.Apply();

                // Set the quad to the texture
                quadRenderer.material.mainTexture = texture;
            }
        }
    }
}
