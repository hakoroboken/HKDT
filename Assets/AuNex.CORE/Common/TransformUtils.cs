using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using System;

namespace AuNex
{
    namespace Common
    {
        public static class TransformUtils
        {
            /// <summary>
            /// ROS2座標をUnity座標に変換
            /// </summary>
            /// <param name="vector3"></param>
            /// <returns></returns>
            public static Vector3 Ros2Unity(this Vector3 vector3)
            {
                return new Vector3(-vector3.y, vector3.z, vector3.x);
            }

            /// <summary>
            /// Unity座標をROS2座標に変換
            /// </summary>
            /// <param name="vector3"></param>
            /// <returns></returns>
            public static Vector3 Unity2Ros(this Vector3 vector3)
            {
                return new Vector3(vector3.z, -vector3.x, vector3.y);
            }

            /// <summary>
            /// ROS2姿勢をUnity姿勢に変換
            /// </summary>
            /// <param name="quaternion"></param>
            /// <returns></returns>
            public static Quaternion Ros2Unity(this Quaternion quaternion)
            {
                return new Quaternion(quaternion.y, -quaternion.z, -quaternion.x, quaternion.w);
            }

            /// <summary>
            /// Unity姿勢をROS2姿勢に変換
            /// </summary>
            /// <param name="quaternion"></param>
            /// <returns></returns>
            public static Quaternion Unity2Ros(this Quaternion quaternion)
            {
                return new Quaternion(-quaternion.z, quaternion.x, -quaternion.y, quaternion.w);
            }

            public static tf2_msgs.msg.TFMessage CreateTFMessage(geometry_msgs.msg.TransformStamped transform)
            {
                var tf = new tf2_msgs.msg.TFMessage();

                tf.Transforms = new[]{transform};

                return tf;
            }

            public static float QuatToYaw(geometry_msgs.msg.Quaternion q)
            {
                var unity_q = new Quaternion((float)q.Y, (float)(-1.0*q.Z), (float)(-1.0*q.X), (float)(q.W));

                return unity_q.eulerAngles.y * Mathf.Deg2Rad;
            }          
        }

        public class TFBroadCaster
        {
            private IPublisher<tf2_msgs.msg.TFMessage> publisher;

            public TFBroadCaster(ROS2Node node)
            {
                publisher = node.CreatePublisher<tf2_msgs.msg.TFMessage>("/tf");
            }

            public void SendTransform(geometry_msgs.msg.TransformStamped t)
            {
                var tf = TransformUtils.CreateTFMessage(t);

                publisher.Publish(tf);
            }
        }

        public class TFListener
        {
            private ISubscription<tf2_msgs.msg.TFMessage> dynamic_subscription;
            private ISubscription<tf2_msgs.msg.TFMessage> static_subscription;
            private tf2_msgs.msg.TFMessage dynamic_;
            private tf2_msgs.msg.TFMessage static_;

            public TFListener(ROS2Node node)
            {
                dynamic_ = new();
                static_ = new();

                dynamic_subscription = node.CreateSubscription<tf2_msgs.msg.TFMessage>(
                    "/tf",
                    CallbackOfDynamicTF
                );

                static_subscription = node.CreateSubscription<tf2_msgs.msg.TFMessage>(
                    "/tf_static",
                    CallbackOfStaticTF
                );
            }

            public geometry_msgs.msg.PoseStamped GetDynamicTransform(String frame_id, String child_frame_id)
            {
                var p = new geometry_msgs.msg.PoseStamped();
                bool found = false;

                foreach(var t in dynamic_.Transforms)
                {
                    bool correct_frame = frame_id == t.Header.Frame_id;
                    bool correct_child = child_frame_id == t.Child_frame_id;

                    if(correct_frame && correct_child)
                    {
                        found = true;

                        p.Pose.Position.X = t.Transform.Translation.X;
                        p.Pose.Position.Y = t.Transform.Translation.Y;
                        p.Pose.Position.Z = t.Transform.Translation.Z;
                        p.Pose.Orientation = t.Transform.Rotation;
                    }
                }

                if(!found)Debug.LogWarning("指定されたTransformが見つかりませんでした。");

                return p;
            }

            public geometry_msgs.msg.PoseStamped GetStaticTransform(String frame_id, String child_frame_id)
            {
                var p = new geometry_msgs.msg.PoseStamped();
                bool found = false;

                foreach(var t in static_.Transforms)
                {
                    bool correct_frame = frame_id == t.Header.Frame_id;
                    bool correct_child = child_frame_id == t.Child_frame_id;

                    if(correct_frame && correct_child)
                    {
                        found = true;

                        p.Pose.Position.X = t.Transform.Translation.X;
                        p.Pose.Position.Y = t.Transform.Translation.Y;
                        p.Pose.Position.Z = t.Transform.Translation.Z;
                        p.Pose.Orientation = t.Transform.Rotation;
                    }
                }

                if(!found)Debug.LogWarning("指定されたTransformが見つかりませんでした。");

                return p;
            }

            private void CallbackOfDynamicTF(tf2_msgs.msg.TFMessage msg)
            {
                dynamic_ = msg;
            }

            private void CallbackOfStaticTF(tf2_msgs.msg.TFMessage msg)
            {
                static_ = msg;
            }
        }
    }// end of namespace Common
}// end of namespace AuNex


