using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ROS2
{
    public class ROS2TransformPublisher : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        //private IPublisher<geometry_msgs.msg.Vector3> position_pub;
        //private IPublisher<geometry_msgs.msg.Quaternion> rotation_pub;
        private IPublisher<geometry_msgs.msg.Transform> transform_pub;

        private int i;
        public string cameraEye;
        public GameObject Camera;
        public Vector3 position;
        public Quaternion rotation;


        // Start is called before the first frame update
        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
        }

        // Update is called once per frame
        void Update()
        {
            if (ros2Unity.Ok())
            {
                if (ros2Node == null)
                {
                    ros2Node = ros2Unity.CreateNode(cameraEye + "ROS2UnityTransformNode");
                    //position_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Vector3>("chatter/" + cameraEye + "/position");
                    //rotation_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Quaternion>("chatter/" + cameraEye + "/rotation");
                    transform_pub = ros2Node.CreatePublisher<geometry_msgs.msg.Transform>("chatter/" + cameraEye + "/transform");
                }

                i++;
                //std_msgs.msg.String msg = new std_msgs.msg.String();
                //msg.Data = message + i;
                //chatter_pub.Publish(msg);

                
                rotation = Camera.transform.rotation;

                // Publish Position Data
                geometry_msgs.msg.Vector3 pos = new geometry_msgs.msg.Vector3();
                position = Camera.transform.position;
                pos.X = position.x;
                pos.Y = position.y;
                pos.Z = position.z;
                //position_pub.Publish(pos);

                // Publish Position Data
                geometry_msgs.msg.Quaternion rot = new geometry_msgs.msg.Quaternion();
                rotation = Camera.transform.rotation;
                rot.X = rotation.x;
                rot.Y = rotation.y;
                rot.Z = rotation.z;
                rot.W = rotation.w;
                //rotation_pub.Publish(rot);

                // Transform Data
                geometry_msgs.msg.Transform transform = new geometry_msgs.msg.Transform();
                transform.Translation = pos;
                transform.Rotation = rot;
                transform_pub.Publish(transform);
            }
        }
    }
}  // namespace ROS2
