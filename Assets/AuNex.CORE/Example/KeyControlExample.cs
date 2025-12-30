using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using System;

public class KeyControlExample : MonoBehaviour
{
    public String node_name = "controller_node";
    public String cmd_vel_topic_name = "/cmd_vel";
    public float max_linear_velocity = 1.0f;
    public float max_angular_velocity = 1.0f;
    private ROS2UnityComponent ros2Unity;
    private ROS2Node node;
    private IPublisher<geometry_msgs.msg.Twist> cmd_publisher;

    // Start is called before the first frame update
    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    // Update is called once per frame
    void Update()
    {
        if(node == null)
        {
            node = ros2Unity.CreateNode(node_name);

            cmd_publisher = node.CreatePublisher<geometry_msgs.msg.Twist>(cmd_vel_topic_name);
        }
        else
        {
            var twist_msg = new geometry_msgs.msg.Twist();
            
            twist_msg.Linear.Y = max_linear_velocity * Input.GetAxis("Vertical");

            twist_msg.Angular.Z = max_angular_velocity * Input.GetAxis("Horizontal");

            cmd_publisher.Publish(twist_msg);
        }
    }
}
