using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ROS2
{
    public class ROS2TransformPublisher : MonoBehaviour
    {
        // ROS2 connection
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<geometry_msgs.msg.Transform> transform_pub;

        // Unity Scene
        public string channel = "chatter";
        public GameObject Camera;
        private Vector3 position;
        private Quaternion rotation;

        // Initialize the ros2 components
        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            ros2Node = ros2Unity.CreateNode(channel);
            transform_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Transform>(channel);
        }

        // Update is called once per frame
        void Update()
        {
            if (ros2Unity.Ok())
            {
                // Get the Position Data
                geometry_msgs.msg.Vector3 pos = new geometry_msgs.msg.Vector3();
                position = Camera.transform.position;
                pos.X = position.x;
                pos.Y = position.y;
                pos.Z = position.z;

                // Get the Rotation Data
                geometry_msgs.msg.Quaternion rot = new geometry_msgs.msg.Quaternion();
                rotation = Camera.transform.rotation;
                rot.X = rotation.x;
                rot.Y = rotation.y;
                rot.Z = rotation.z;
                rot.W = rotation.w;

                // Publish the Transform Data
                geometry_msgs.msg.Transform transform = new geometry_msgs.msg.Transform();
                transform.Translation = pos;
                transform.Rotation = rot;
                transform_pub.Publish(transform);
            }
        }
    }
}  // namespace ROS2
