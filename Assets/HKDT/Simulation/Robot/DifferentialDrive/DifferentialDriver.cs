using UnityEngine;
using ROS2;

using HKDT.Auto.Common;

public class DifferentialDriver : MonoBehaviour
{
    public string node_name = "diff_bot_node";
    public string left_topic_name = "/left_wheel_velocity";
    public string right_topic_name = "/right_wheel_velocity";
    public string posture_topic_name = "/robot_posture";
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;
    public GameObject robotBody;


    private ROS2UnityComponent ros2Unity;
    private ROS2Node node;
    private ISubscription<std_msgs.msg.Float32> left_subscription;
    private ISubscription<std_msgs.msg.Float32> right_subscription;
    private IPublisher<geometry_msgs.msg.Quaternion> posture_publisher;

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

            posture_publisher = node.CreatePublisher<geometry_msgs.msg.Quaternion>(posture_topic_name);
        }
        else
        {
            leftWheel.SetDriveTargetVelocity(ArticulationDriveAxis.X, left_wheel_velocity);
            rightWheel.SetDriveTargetVelocity(ArticulationDriveAxis.X, right_wheel_velocity);

            var posture_msg = TransformUtils.Unity2Ros(robotBody.transform.rotation);
            posture_publisher.Publish(posture_msg);
        }
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
