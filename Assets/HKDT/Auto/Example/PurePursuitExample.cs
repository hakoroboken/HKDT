using UnityEngine;
using ROS2;

using HKDT.Auto.Common;
using HKDT.Auto.Control;

public class PurePursuitExample : MonoBehaviour
{
    public string node_name = "pure_pursuit_node";
    public string cmd_vel_topic_name = "/cmd_vel";
    public string path_topic_name = "/path";
    public float look_ahead_distance = 0.1f;
    public float velocity_gain = 1.0f;
    private ROS2UnityComponent ros2Unity;
    private ROS2Node node;
    private IPublisher<geometry_msgs.msg.Twist> cmd_publisher;
    private IPublisher<geometry_msgs.msg.PoseStamped> ref_pose_publisher;
    private ISubscription<nav_msgs.msg.Path> path_subscriber;
    private TFListener tfListener;
    private nav_msgs.msg.Path path_;

    private PurePursuit pure_pursuit;

    // Start is called before the first frame update
    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        pure_pursuit = new PurePursuit(look_ahead_distance, velocity_gain);
    }

    // Update is called once per frame
    void Update()
    {
        if(node == null)
        {
            node = ros2Unity.CreateNode(node_name);

            cmd_publisher = node.CreatePublisher<geometry_msgs.msg.Twist>(cmd_vel_topic_name);

            ref_pose_publisher = node.CreatePublisher<geometry_msgs.msg.PoseStamped>("/ref_pose");

            path_subscriber = node.CreateSubscription<nav_msgs.msg.Path>(
                path_topic_name,
                PathCallback
            );

            tfListener = new TFListener(node);
        }
        else
        {
            if(path_ != null)
            {
                var current_pose = tfListener.GetDynamicTransform("map", "base_link");

                var cmd_vel = pure_pursuit.Compute(current_pose);

                var ref_pose = path_.Poses[pure_pursuit.GetTargetIdx()];
                ref_pose_publisher.Publish(ref_pose);

                cmd_publisher.Publish(cmd_vel);
            }
        }
    }

    void PathCallback(nav_msgs.msg.Path msg)
    {
        path_ = msg;
        pure_pursuit.SetPath(path_);
    }
}
