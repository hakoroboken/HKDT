using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using System;

public class DifferentialDriver : MonoBehaviour
{
    public String node_name = "differential_drive_node";
    public String left_topic_name = "/left_wheel_velocity";
    public String right_topic_name = "/right_wheel_velocity";
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;


    private ROS2UnityComponent ros2Unity;
    private ROS2Node node;
    private ISubscription<std_msgs.msg.Float32> left_subscription;
    private ISubscription<std_msgs.msg.Float32> right_subscription;

    private float left_wheel_velocity = 0.0f;
    private float right_wheel_velocity = 0.0f;

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
            
            left_subscription = node.CreateSubscription<std_msgs.msg.Float32>
            (
                left_topic_name, 
                DriveLeftWheel
            );

            right_subscription = node.CreateSubscription<std_msgs.msg.Float32>
            (
                right_topic_name, 
                DriveRightWheel
            );
        }

        leftWheel.SetDriveTargetVelocity(ArticulationDriveAxis.X, left_wheel_velocity);
        rightWheel.SetDriveTargetVelocity(ArticulationDriveAxis.X, right_wheel_velocity);
    }

    void DriveLeftWheel(std_msgs.msg.Float32 msg)
    {
        float velocity = msg.Data;
        left_wheel_velocity = velocity;
    }

    void DriveRightWheel(std_msgs.msg.Float32 msg)
    {
        float velocity = msg.Data;
        right_wheel_velocity = velocity;
    }
}
