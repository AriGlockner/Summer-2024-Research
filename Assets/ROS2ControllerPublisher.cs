using Oculus.Interaction.Editor;
using ROS2;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using std_msgs.msg;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;

namespace ROS2
{
    public class ROS2ControllerPublisher : MonoBehaviour
    {
        // ROS2 connection
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private readonly string channel = "controller";

        // Right Controller
        private IPublisher<sensor_msgs.msg.Joy> left_joy_pub;
        private IPublisher<sensor_msgs.msg.Joy> right_joy_pub;

        // Input Devices
        public List<InputDevice> leftHandDevices = new();
        public List<InputDevice> rightHandDevices = new();

        // Start is called before the first frame update
        void Start()
        {
            // Set the controllers
            UpdateDevices();

            // ROS2 setup
            ros2Unity = GetComponent<ROS2UnityComponent>();
            ros2Node = ros2Unity.CreateNode(channel);

            // Initialize joystick publishers
            left_joy_pub = ros2Node.CreatePublisher<sensor_msgs.msg.Joy>(channel + "_left");
            right_joy_pub = ros2Node.CreatePublisher<sensor_msgs.msg.Joy>(channel + "_right");
        }

        // Update is called once per frame
        void Update()
        {
            UpdateDevices();
            
            if (ros2Unity.Ok())
            {
                PublishJoysticks(leftHandDevices.First(), left_joy_pub);
                PublishJoysticks(rightHandDevices.First(), right_joy_pub);
            }
        }

        void UpdateDevices()
        {
            InputDevices.GetDevicesWithCharacteristics(InputDeviceCharacteristics.Left | InputDeviceCharacteristics.Controller, leftHandDevices);
            InputDevices.GetDevicesWithCharacteristics(InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller, rightHandDevices);
        }

        void PublishJoysticks(InputDevice joystick, IPublisher<sensor_msgs.msg.Joy> publisher)
        {
            // Get Joystick and Button Values
            List<float> axes = GetJoystickData(joystick);
            List<int> buttons = GetButtonData(joystick);

            // Convert the data into a message
            sensor_msgs.msg.Joy joy_msg = new()
            {
                Axes = axes.ToArray(),
                Buttons = buttons.ToArray()
            };

            // Publish the message
            publisher.Publish(joy_msg);
        }

        /**
         * Function that returns the joystick values from unity in the form of a List of floats
         */
        List<float> GetJoystickData(InputDevice device)
        {
            List<float> axes = new();

            // Gets the Primary Axis Values and adds it to the list
            if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 primary))
            {
                axes.Add(primary.x);
                axes.Add(primary.y);
            } else
            {
                axes.Add(0f);
                axes.Add(0f);
            }

            // Gets the Secondary Axis Values and adds it to the list
            if (device.TryGetFeatureValue(CommonUsages.secondary2DAxis, out Vector2 secondary))
            {
                axes.Add(secondary.x);
                axes.Add(secondary.y);
            } else
            {
                axes.Add(0f);
                axes.Add(0f);
            }

            return axes;
        }

        List<int> GetButtonData(InputDevice device)
        {
            List<int> buttons = new();

            // Primary Button
            if (device.TryGetFeatureValue(CommonUsages.primaryButton, out bool primary))
            {
                buttons.Add(primary ? 1 : 0);
            } else
            {
                buttons.Add(0);
            }

            // Secondary Button
            if (device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondary))
            {
                buttons.Add(secondary ? 1 : 0);
            }
            else
            {
                buttons.Add(0);
            }

            // Grip Button
            if (device.TryGetFeatureValue(CommonUsages.gripButton, out bool grip))
            {
                buttons.Add(grip ? 1 : 0);
            }
            else
            {
                buttons.Add(0);
            }

            // Trigger Button
            if (device.TryGetFeatureValue(CommonUsages.triggerButton, out bool trigger))
            {
                buttons.Add(trigger ? 1 : 0);
            }
            else
            {
                buttons.Add(0);
            }

            return buttons;
        }

        void OnApplicationQuit()
        {
            left_joy_pub?.Dispose();
            right_joy_pub?.Dispose();
            Debug.Log("ROS2 publisher disposed");
        }

        void OnDisable()
        {
            if (left_joy_pub != null)
            {
                left_joy_pub.Dispose();
                left_joy_pub = null;
                Debug.Log("ROS2 publisher disposed");
            }

            if (right_joy_pub != null)
            {
                right_joy_pub.Dispose();
                right_joy_pub = null;
                Debug.Log("ROS2 publisher disposed");
            }
        }
    }
}