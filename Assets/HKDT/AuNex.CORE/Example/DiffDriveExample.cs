using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using System;

public class DiffDriveExample : MonoBehaviour
{
    public String node_name = "diff_drive_node";
    public String cmd_vel_topic_name = "/cmd_vel";
    public String left_velocity_topic_name = "/left_wheel_velocity";
    public String right_velocity_topic_name = "/right_wheel_velocity";
    public float wheel_width = 0.1f;
    public float wheel_radius = 0.1f;
    private ROS2UnityComponent ros2Unity;
    private ROS2Node node;
    private ISubscription<geometry_msgs.msg.Twist> cmd_subscriber;
    private IPublisher<std_msgs.msg.Float32> left_velocity_publisher;
    private IPublisher<std_msgs.msg.Float32> right_velocity_publisher;

    private static float RAD2RPM = 60.0f / (2.0f*Mathf.PI);

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

            cmd_subscriber = node.CreateSubscription<geometry_msgs.msg.Twist>(
                cmd_vel_topic_name,
                CalcTwist
            );

            left_velocity_publisher = node.CreatePublisher<std_msgs.msg.Float32>(left_velocity_topic_name);
            right_velocity_publisher = node.CreatePublisher<std_msgs.msg.Float32>(right_velocity_topic_name);
        }
        else
        {
            
        }
    }

    void CalcTwist(geometry_msgs.msg.Twist msg)
    {
        float linear_vel = (float)msg.Linear.Y;
        float angular_vel = (float)msg.Angular.Z;

        var left_msg = new std_msgs.msg.Float32();
        var right_msg = new std_msgs.msg.Float32();

        float left_vel = 0.5f*linear_vel + wheel_width * angular_vel;
        float right_vel = 0.5f*linear_vel - wheel_width * angular_vel;

        float left_omega = left_vel / wheel_radius;
        float right_omega = right_vel / wheel_radius;

        left_msg.Data = left_omega * RAD2RPM;
        right_msg.Data = right_omega * RAD2RPM;

        if(node != null)
        {
            left_velocity_publisher.Publish(left_msg);
            right_velocity_publisher.Publish(right_msg);
        }
    }
}
