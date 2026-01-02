using System;
using System.Collections;
using System.Collections.Generic;
using AuNex.Common;
using UnityEngine;

namespace AuNex
{
    namespace Control
    {
        public class PurePursuit
        {
            private float lookAheadDistance = 0.0f;
            private int targetIdx;
            private nav_msgs.msg.Path path_;
            private float velocityGain = 0.0f;

            public PurePursuit(float look_ahead_distance, float velocity_gain)
            {
                lookAheadDistance = look_ahead_distance;
                velocityGain = velocity_gain;
            }

            public void SetPath(nav_msgs.msg.Path path)
            {
                path_ = path;
                targetIdx = 0;
            }

            public geometry_msgs.msg.Twist Compute(geometry_msgs.msg.PoseStamped current_pose)
            {
                var target_pose = new geometry_msgs.msg.PoseStamped();
                
                for(int i = targetIdx; i < path_.Poses.Length; i++)
                {
                    var p = path_.Poses[i];

                    double _dx = p.Pose.Position.X - current_pose.Pose.Position.X;
                    double _dy = p.Pose.Position.Y - current_pose.Pose.Position.Y;

                    if((_dx*_dx+_dy*_dy) > lookAheadDistance*lookAheadDistance)
                    {
                        break;
                    }

                    targetIdx++;
                }

                if(targetIdx > path_.Poses.Length-1)targetIdx = path_.Poses.Length-1;

                target_pose = path_.Poses[targetIdx];

                double dx = target_pose.Pose.Position.X - current_pose.Pose.Position.X;
                double dy = target_pose.Pose.Position.Y - current_pose.Pose.Position.Y;

                double distance = Math.Sqrt(dx*dx + dy*dy);

                double alpha = Math.Atan2(dy, dx) - TransformUtils.QuatToYaw(current_pose.Pose.Orientation)- 90.0f*Mathf.Deg2Rad;

                var cmd_vel = new geometry_msgs.msg.Twist();
                cmd_vel.Linear.Y = velocityGain * distance;
                cmd_vel.Angular.Z = distance * (2.0 * Math.Sin(alpha) / lookAheadDistance);

                return cmd_vel;
            }

            public int GetTargetIdx()
            {
                return targetIdx;
            }
        }
    }
}