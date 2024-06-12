using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using UnityEngine;

public class ROS2Subscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "ros_to_unity";
    public string imageTopic = "ros_to_unity";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        //ros.Subscribe<StringMsg>(topicName, ReceiveMessage);
        ros.Subscribe<ImageMsg>(imageTopic, ReceiveImage);
    }

    void ReceiveMessage(StringMsg message)
    {
        Debug.Log("Received from ROS: " + message.data);
    }

    void ReceiveImage(ImageMsg imageMessage)
    {
        Debug.Log("Image recieved");

        /*
        // Example: Convert ROS image to a Unity Texture2D
        Texture2D texture = new Texture2D(imageMessage.width, imageMessage.height, TextureFormat.RGB24, false);
        texture.LoadImage(imageMessage.data);

        // Apply the texture to a material or a UI element in your scene
        // For example, assuming you have a GameObject with a Renderer component:
        GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.transform.localScale = new Vector3(imageMessage.width, imageMessage.height, 1);
        Renderer renderer = quad.GetComponent<Renderer>();
        renderer.material.mainTexture = texture;
        */
    }
}
