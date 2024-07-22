using ROS2;
using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
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
        private string channel = "controller";

        // Right Controller
        private IPublisher<sensor_msgs.msg.Joy> right_controller_joyPub;
        private IPublisher<std_msgs.msg.Bool> right_controller_aButtonPub;
        private IPublisher<std_msgs.msg.Bool> right_controller_bButtonPub;
        private IPublisher<std_msgs.msg.Bool> right_controller_frontTriggerPub;
        private IPublisher<std_msgs.msg.Bool> right_controller_backTriggerPub;

        // Left Controller
        private IPublisher<sensor_msgs.msg.Joy> left_controller_joyPub;
        private IPublisher<std_msgs.msg.Bool> left_controller_xButtonPub;
        private IPublisher<std_msgs.msg.Bool> left_controller_yButtonPub;
        private IPublisher<std_msgs.msg.Bool> left_controller_frontTriggerPub;
        private IPublisher<std_msgs.msg.Bool> left_controller_backTriggerPub;

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

            // Initialize right controller channels
            right_controller_joyPub = ros2Node.CreatePublisher<sensor_msgs.msg.Joy>(channel + "/Right/joystick");
            right_controller_aButtonPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>(channel + "/Right/A");
            right_controller_bButtonPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>(channel + "/Right/B");
            right_controller_frontTriggerPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>(channel + "/Right/frontTrigger");
            right_controller_backTriggerPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>(channel + "/Right/backTrigger");

            // Initialize left controller channels
            left_controller_joyPub = ros2Node.CreatePublisher<sensor_msgs.msg.Joy>(channel + "/Left/joystick");
            left_controller_xButtonPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>(channel + "/Left/X");
            left_controller_yButtonPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>(channel + "/Left/Y");
            left_controller_frontTriggerPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>(channel + "/Left/frontTrigger");
            left_controller_backTriggerPub = ros2Node.CreatePublisher<std_msgs.msg.Bool>(channel + "/Left/backTrigger");
        }

        // Update is called once per frame
        void Update()
        {
            UpdateDevices();
            
            if (ros2Unity.Ok())
            {
                // Publish right controller data
                foreach (var device in leftHandDevices)
                {
                    PublishJoysticks(device, right_controller_joyPub);
                    //PublishButtons(device, right_controller_aButtonPub, right_controller_bButtonPub,
                    //    right_controller_frontTriggerPub, right_controller_backTriggerPub);
                }

                // Publish left controller data
                foreach (var device in leftHandDevices)
                {
                    PublishJoysticks(device, left_controller_joyPub);
                    //PublishButtons(device, left_controller_xButtonPub, left_controller_yButtonPub,
                    //    left_controller_frontTriggerPub, left_controller_backTriggerPub);
                }
            }
        }

        void UpdateDevices()
        {
            InputDevices.GetDevicesWithCharacteristics(InputDeviceCharacteristics.Left | InputDeviceCharacteristics.Controller, leftHandDevices);
            InputDevices.GetDevicesWithCharacteristics(InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller, rightHandDevices);
        }

        void PublishJoysticks(InputDevice device, IPublisher<sensor_msgs.msg.Joy> pub)
        {
            Vector2 primary, secondary;

            if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out primary)
                && device.TryGetFeatureValue(CommonUsages.secondary2DAxis, out secondary))
            {
                sensor_msgs.msg.Joy joyMsg = new sensor_msgs.msg.Joy();
                joyMsg.Axes = new float[] { primary.x, primary.y, secondary.x, secondary.y };
                pub.Publish(joyMsg);
            }
        }

        void PublishButtons(InputDevice device, IPublisher<std_msgs.msg.Bool> primary, IPublisher<std_msgs.msg.Bool> secondary,
            IPublisher<std_msgs.msg.Bool> frontTrigger, IPublisher<std_msgs.msg.Bool> backTrigger)
        {
            // primary button
            bool primaryButtonValue;
            if (device.TryGetFeatureValue(CommonUsages.primaryButton, out primaryButtonValue))
            {
                std_msgs.msg.Bool btn = new std_msgs.msg.Bool();
                btn.Data = primaryButtonValue;
                primary.Publish(btn);                
            }

            // secondary button
            bool secondaryButtonValue;
            if (device.TryGetFeatureValue(CommonUsages.secondaryButton, out secondaryButtonValue))
            {
                std_msgs.msg.Bool btn = new std_msgs.msg.Bool();
                btn.Data = secondaryButtonValue;
                secondary.Publish(btn);
            }

            // front trigger button
            bool frontTriggerValue;
            if (device.TryGetFeatureValue(CommonUsages.triggerButton, out frontTriggerValue))
            {
                std_msgs.msg.Bool btn = new std_msgs.msg.Bool();
                btn.Data = frontTriggerValue;
                frontTrigger.Publish(btn);
            }

            // back trigger button
            bool backTriggerValue;
            if (device.TryGetFeatureValue(CommonUsages.gripButton, out backTriggerValue))
            {
                std_msgs.msg.Bool btn = new std_msgs.msg.Bool();
                btn.Data = backTriggerValue;
                backTrigger.Publish(btn);
            }
        }
    }
}