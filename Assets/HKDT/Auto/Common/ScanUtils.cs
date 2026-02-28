using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AuNex
{
    namespace Common
    {
        class ScanUtils
        {
            /// <summary>
            /// LaserScanメッセージを2Dポイントクラウドに変換する
            /// </summary>
            /// <param name="scan">LaserScanメッセージ</param>
            /// <returns>2Dポイントクラウド</returns>
            public static void LaserScanToPointCloud(sensor_msgs.msg.LaserScan scan, ref List<Vector2> pointCloud)
            {
                float angle = scan.Angle_min;
                for (int i = 0; i < scan.Ranges.Length; i++)
                {
                    float range = scan.Ranges[i];
                    if (range >= scan.Range_min && range <= scan.Range_max)
                    {
                        pointCloud.Add(new Vector2(range * Mathf.Cos(angle), range * Mathf.Sin(angle)));
                    }
                    angle += scan.Angle_increment;
                }
            }
        }
    }
}