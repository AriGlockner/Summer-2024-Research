using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace ROS2
{
    /// <summary>
    /// Subscribe transform data in ROS2
    /// </summary>

    public class RotateOrigin : MonoBehaviour
    {
        // ROS2 Connection components
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private ISubscription<geometry_msgs.msg.Pose> subscriber;
        public string topic_Name = "chatter";

        // Unity Objects
        public GameObject left;
        public GameObject right;

        // Queue
        private ConcurrentQueue<geometry_msgs.msg.Pose> pose_queue = new ConcurrentQueue<geometry_msgs.msg.Pose>();

        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
        }

        private void Update()
        {
            // Subscribe the incoming data
            if (ros2Node == null && ros2Unity.Ok())
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityTransformNode" +  topic_Name);
                subscriber = ros2Node.CreateSubscription<geometry_msgs.msg.Pose>(topic_Name, msg =>
                {
                    // Update position
                    Vector3 pos = new Vector3((float) msg.Position.X, (float)msg.Position.Y, (float)msg.Position.Z);
                    left.transform.position = pos;
                    right.transform.position = pos;
                    
                    // Update rotation
                    Quaternion rot = new Quaternion((float) msg.Orientation.X, (float)(msg.Orientation.Y), (float)(msg.Orientation.Z), (float) (msg.Orientation.W));
                    left.transform.rotation = rot;
                    right.transform.rotation = rot;
                });
            }
        }

        void OnApplicationQuit()
        {
            subscriber?.Dispose();
            Debug.Log("ROS2 subscription disposed");
        }

        void OnDisable()
        {
            if (subscriber != null)
            {
                subscriber.Dispose();
                subscriber = null;
                Debug.Log("ROS2 subscription disposed");
            }
        }
    }

}
