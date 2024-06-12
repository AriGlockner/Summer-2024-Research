using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using UnityEngine;

public class ROS2_Message_Subscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "chatter";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(topicName, ReceiveMessage);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void ReceiveMessage(StringMsg message)
    {
        Debug.Log("Received from ROS: " + message.data);
    }
}
