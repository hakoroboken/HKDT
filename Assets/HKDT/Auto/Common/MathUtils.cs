using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

namespace HKDT.Auto.Common
{
    public class MathUtils
    {
        /// <summary>
        /// 回転行列をQuaternionに変換する
        /// </summary>
        /// <param name="rot_matrix">回転行列(２次元)</param>
        /// <returns>Quaternion</returns>
        public static Quaternion ToQuat(Matrix<double> rot_matrix)
        {
            float angle = Mathf.Atan2((float)rot_matrix[1, 0], (float)rot_matrix[0, 0]);
            return Quaternion.Euler(0, 0, angle);
        }

        /// <summary>
        /// ラジアンをQuaternionに変換する
        /// </summary>
        /// <param name="rad"></param>
        /// <returns>Quaternion</returns>
        public static Quaternion ToQuatUnity(float rad)
        {
            return Quaternion.Euler(0, rad* Mathf.Rad2Deg, 0);
        }

        /// <summary>
        /// Quaternionを回転行列に変換する
        /// </summary>
        /// <param name="quat">Quaternion</param>
        /// <returns>回転行列(2x2)</returns>
        public static Matrix<double> ToRotMatrix(Quaternion quat)
        {
            float angle = quat.eulerAngles.z * Mathf.Deg2Rad;
            Matrix<double> rot_matrix = Matrix<double>.Build.Dense(2, 2);
            rot_matrix[0, 0] = Mathf.Cos(angle);
            rot_matrix[0, 1] = -Mathf.Sin(angle);
            rot_matrix[1, 0] = Mathf.Sin(angle);
            rot_matrix[1, 1] = Mathf.Cos(angle);
            return rot_matrix;
        }

        /// <summary>
        /// Quaternionをラジアンに変換する
        /// </summary>
        /// <param name="quat"></param>
        /// <returns>ラジアン</returns>
        public static float toRadUnity(Quaternion quat)
        {
            return -1.0f*quat.eulerAngles.y * Mathf.Deg2Rad;
        }
    }
}