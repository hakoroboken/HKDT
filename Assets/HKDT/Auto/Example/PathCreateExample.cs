using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using ROS2;

public class PathCreateExample : MonoBehaviour
{
    public String node_name = "path_create_example";
    public String path_topic_name = "/path";
    public int N = 50;
    private ROS2UnityComponent ros2Unity;
    private ROS2Node node;
    private AuNex.Common.TFListener tfListener;
    private IPublisher<nav_msgs.msg.Path> path_publisher;

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

            tfListener = new AuNex.Common.TFListener(node);

            path_publisher = node.CreatePublisher<nav_msgs.msg.Path>(path_topic_name);
        }
        else
        {
            // 現在位置を取得
            var current = tfListener.GetDynamicTransform("map", "base_link");

            // JumpはSpaceでCancelはescape（おそらくデフォルト？）
            float user_input = Input.GetAxis("Jump") - Input.GetAxis("Cancel");

            var target_pose = new geometry_msgs.msg.PoseStamped();

            if(user_input > 0.0)
            {
                target_pose.Pose.Position.X = -2.0;
                target_pose.Pose.Position.Y = 2.0;
                target_pose.Pose.Position.Z = 0.0;

                target_pose.Pose.Orientation = AuNex.Common.TransformUtils.YawToQuat(90.0f);

                var path = AuNex.Planning.CubicSpline.CreatePath(current, target_pose, N, "map");
                path_publisher.Publish(path);
            }
            else if(user_input < 0.0)
            {
                target_pose.Pose.Position.X = 2.0;
                target_pose.Pose.Position.Y = 4.0;
                target_pose.Pose.Position.Z = 0.0;

                target_pose.Pose.Orientation = AuNex.Common.TransformUtils.YawToQuat(0.0f);

                var path = AuNex.Planning.CubicSpline.CreatePath(current, target_pose, N, "map");
                path_publisher.Publish(path);
            }
        }
    }
}
