using ROS2;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using std_msgs.msg;
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
        private readonly string channel = "controller";

        // Right Controller
        private IPublisher<sensor_msgs.msg.Joy> left_joy_pub;
        private IPublisher<sensor_msgs.msg.Joy> right_joy_pub;

        /*
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
        */

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

            /*
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
            */
        }

        // Update is called once per frame
        void Update()
        {
            UpdateDevices();
            
            if (ros2Unity.Ok())
            {
                Debug.Log("# of devices in hand: " + rightHandDevices.Count);
                foreach (var device in leftHandDevices)
                {
                    PublishJoysticks(device, left_joy_pub);
                }

                foreach (var device in rightHandDevices)
                {
                    PublishJoysticks(device, right_joy_pub);
                }
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

            // Publish Joystick data
            sensor_msgs.msg.Joy joy_msg = new()
            {
                Axes = axes.ToArray(),
                Buttons = buttons.ToArray()
            };
            //Debug.Log(joy_msg.ToString());
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
                Debug.Log(primary);
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

            //Debug.Log("Buttons: " + buttons.ToString());
            return buttons;
        }


        /*
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
        */
    }
}