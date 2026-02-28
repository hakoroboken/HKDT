using System;
using ROS2;
using UnityEngine;

public class Robot : MonoBehaviour
{
    public String node_name = "omni_sim_node";
    public String fl_motor_topic = "/front_left_motor";
    public String fr_motor_topic = "/front_right_motor";
    public String rl_motor_topic = "/rear_left_motor";
    public String rr_motor_topic = "/rear_right_motor"; 

    public ArticulationBody moto_joint_1;
    public ArticulationBody moto_joint_2;
    public ArticulationBody moto_joint_3;
    public ArticulationBody moto_joint_4;
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private float[] targetSpeed;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        targetSpeed = new float[4];
    }

    void Update()
    {
        if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode(node_name);
                ros2Node.CreateSubscription<std_msgs.msg.Float32>(fl_motor_topic, msg => { targetSpeed[0] = msg.Data; });
                ros2Node.CreateSubscription<std_msgs.msg.Float32>(rl_motor_topic, msg => { targetSpeed[1] = msg.Data; });
                ros2Node.CreateSubscription<std_msgs.msg.Float32>(rr_motor_topic, msg => { targetSpeed[2] = msg.Data; });
                ros2Node.CreateSubscription<std_msgs.msg.Float32>(fr_motor_topic, msg => { targetSpeed[3] = msg.Data; });
            }

            moto_joint_1.SetDriveTargetVelocity(ArticulationDriveAxis.X, CheckError(targetSpeed[0]) * 200 * UnityEngine.Random.Range(0.9f, 1.1f));
            moto_joint_2.SetDriveTargetVelocity(ArticulationDriveAxis.X, CheckError(targetSpeed[1]) * 200 * UnityEngine.Random.Range(0.9f, 1.1f)); 
            moto_joint_3.SetDriveTargetVelocity(ArticulationDriveAxis.X, CheckError(targetSpeed[2]) * 200 * UnityEngine.Random.Range(0.9f, 1.1f));
            moto_joint_4.SetDriveTargetVelocity(ArticulationDriveAxis.X, CheckError(targetSpeed[3]) * 200 * UnityEngine.Random.Range(0.9f, 1.1f));
        }
    }

    float CheckError(float input)
    {
        if(input > 1.0){return (float)1.0;}
        if (input < -1.0){return (float)-1.0;}

        return (float)input;
    }
}
