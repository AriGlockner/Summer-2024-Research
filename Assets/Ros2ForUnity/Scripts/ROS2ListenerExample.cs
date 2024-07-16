// Copyright 2019-2021 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using UnityEngine;

namespace ROS2
{

/// <summary>
/// An example class provided for testing of basic ROS2 communication
/// </summary>
public class ROS2ListenerExample : MonoBehaviour
{
    public string topicName = "/chatter";
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<std_msgs.msg.String> string_sub;
    private ISubscription<std_msgs.msg.Bool> bool_sub;
    private ISubscription<std_msgs.msg.Float32> float_sub;
    private ISubscription<geometry_msgs.msg.Point> point_sub;
    private ISubscription<diagnostic_msgs.msg.KeyValue> key_value_sub;

    void Start()
    {
        Application.runInBackground = true;
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("ROS2UnityListenerNode");
            string_sub = ros2Node.CreateSubscription<std_msgs.msg.String>(
              "string", msg => Debug.Log("Unity listener heard: [" + msg.Data + "]"));
                bool_sub = ros2Node.CreateSubscription<std_msgs.msg.Bool>(
                  "bool", msg => Debug.Log("Unity listener heard: [" + msg.Data + "]"));
                float_sub = ros2Node.CreateSubscription<std_msgs.msg.Float32>(
              "float", msg => Debug.Log("Unity listener heard: [" + msg.Data + "]"));
                point_sub = ros2Node.CreateSubscription<geometry_msgs.msg.Point>(
              "point", msg => Debug.Log("Unity listener heard: [ x=" + msg.X + ", y=" + msg.Y + ", z=" + msg.Z + "]"));
                key_value_sub = ros2Node.CreateSubscription<diagnostic_msgs.msg.KeyValue>(
              "key_value", msg => Debug.Log("Unity listener heard: [ key=" + msg.Key + ", value=" + msg.Value + "]"));
                Debug.Log("sub established");
        }
    }
}

}  // namespace ROS2
