using System;
using UnityEngine;
using ROS2;

namespace ROS2
{
    /// <summary>
    /// An example class for subscribing to image messages in ROS2
    /// </summary>
    public class ROS2ImageSubscriber : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private ISubscription<sensor_msgs.msg.Image> image_sub;
        public String topic_Name = "chatter";
        private Texture2D texture;
        public GameObject quad;
        private Renderer quadRenderer;

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            texture = new Texture2D(1, 1); // Initialize a texture
            quadRenderer = quad.GetComponent<Renderer>();
        }

        void Update()
        {
            if (ros2Node == null && ros2Unity.Ok())
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityImageListenerNode");
                image_sub = ros2Node.CreateSubscription<sensor_msgs.msg.Image>(
                    topic_Name, msg => ProcessImage(msg));
            }
        }

        private void ProcessImage(sensor_msgs.msg.Image msg)
        {
            if (msg.Data.Length > 0)
            {
                // Assuming the image is in RGB8 format
                texture.LoadImage(msg.Data);
                // Apply the texture to a GameObject's material, e.g., a Quad
                //GameObject quad = GameObject.Find("ImageQuad");
                if (quad != null)
                {
                    //Renderer renderer = quad.GetComponent<Renderer>();
                    quadRenderer.material.mainTexture = texture;
                }
                Debug.Log("Image received and applied to quad.");
            }
            else
            {
                Debug.LogWarning("Received image message with empty data.");
            }
        }
    }
}
