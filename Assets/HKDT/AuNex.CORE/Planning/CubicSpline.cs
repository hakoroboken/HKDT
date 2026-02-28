using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using AuNex.Algorithm;
using AuNex.Common;
using System;

namespace AuNex
{
    namespace Planning
    {
        /// <summary>
        /// ３次補間したスプライン曲線によって経路計画する。主に差動２輪用.
        //  xとyがtの関数であるとみなして解く
        /// </summary>
        public class CubicSpline
        {
            /// <summary>
            /// ３次多項式を扱うための構造体。abcdは各係数
            /// </summary>
            struct Cubic
            {
                public float a, b, c, d;

                /// <summary>
                /// ３次多項式にtを代入して計算する
                /// </summary>
                /// <param name="t"></param>
                /// <returns></returns>
                public float Eval(float t)
                {
                    return a * t * t * t + b * t * t + c * t + d;
                }
            }

            /// <summary>
            /// ３次多項式の係数を決定する
            /// </summary>
            /// <param name="p0">現在位置</param>
            /// <param name="p1">目的位置</param>
            /// <param name="v0">現在の向き</param>
            /// <param name="v1">目的の向き</param>
            /// <returns>決定された係数</returns>
            static private Cubic SolveCubic(float p0, float p1, float v0, float v1)
            {
                Cubic cubic = new();

                cubic.d = p0;
                cubic.c = v0;
                cubic.a = 2.0f * p0 -2.0f * p1 + v0 + v1;
                cubic.b = -3.0f * p0 + 3.0f * p1 - 2.0f * v0 - v1;

                return cubic;
            }


            /// <summary>
            /// 現在位置と目的位置から３次スプライン補間された経路を得る
            /// </summary>
            /// <param name="current">現在位置</param>
            /// <param name="target">目的位置</param>
            /// <param name="N">分割数</param>
            /// <param name="frame_id">フレームID</param>
            /// <returns></returns>
            public static nav_msgs.msg.Path CreatePath(
                geometry_msgs.msg.PoseStamped current,
                geometry_msgs.msg.PoseStamped target,
                int N,
                String frame_id
            )
            {
                // 現在のロボットの向きと目的のロボットの向きを取得
                float current_yaw = TransformUtils.QuatToYaw(current.Pose.Orientation) + 90.0f * Mathf.Deg2Rad;
                float target_yaw = TransformUtils.QuatToYaw(target.Pose.Orientation);
                // 現在位置と目的位置を２次元にする
                Vector2 current2d = new((float)current.Pose.Position.X, (float)current.Pose.Position.Y);
                Vector2 target2d = new((float)target.Pose.Position.X, (float)target.Pose.Position.Y);
                // 距離ベクトルを計算する
                Vector2 delta2d = target2d - current2d;
                // 距離の絶対値を計算
                float distance = delta2d.magnitude;

                /// x(t)を解く
                Cubic cx = SolveCubic(
                    current2d.x, 
                    target2d.x,
                    distance * Mathf.Cos(current_yaw),
                    distance * Mathf.Cos(target_yaw)
                );
                /// y(t)を解く
                Cubic cy = SolveCubic(
                    current2d.y, 
                    target2d.y,
                    distance * Mathf.Sin(current_yaw),
                    distance * Mathf.Sin(target_yaw)
                );


                geometry_msgs.msg.PoseStamped[] poses = new geometry_msgs.msg.PoseStamped[N+1];
                // tを0から1まで進めてposesに代入
                for(int i = 0; i <= N; i++)
                {
                    float t = (float)i / (float)N;

                    float x = cx.Eval(t);
                    float y = cy.Eval(t);

                    var pose = new geometry_msgs.msg.PoseStamped();
                    pose.Header.Frame_id = frame_id;
                    pose.Pose.Position.X = x;
                    pose.Pose.Position.Y = y;
                    pose.Pose.Position.Z = 0.0;

                    poses[i] = pose;
                }

                var path = new nav_msgs.msg.Path();
                path.Header.Frame_id = frame_id;
                path.Poses = poses;

                return path;
            }
        }
    }
}