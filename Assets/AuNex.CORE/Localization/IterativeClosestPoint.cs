using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using AuNex.Algorithm;

using MathNet.Numerics.LinearAlgebra;
using System;

namespace AuNex
{
    namespace Localization
    {
        public class IterativeClosestPoint
        {
            private Vector2 position;
            private Matrix<double> rotation;
            private KdTree targetKdTree;
            private bool setTargetPointsFlag = false;
            public IterativeClosestPoint()
            {
                position = Vector2.zero;
                rotation = Matrix<double>.Build.DenseIdentity(2);
                setTargetPointsFlag = false;
            }

            public void SetInitialPose(Vector2 initialPosition, Matrix<double> initialRotation)
            {
                position = initialPosition;
                rotation = initialRotation;
            }

            public void SetTargetPoints(List<Vector2> targetPoints)
            {
                // ターゲット(マップ)点群からKdTreeを構築
                targetKdTree = new KdTree(targetPoints);
                setTargetPointsFlag = true;
            }

            public bool Align(List<Vector2> sourcePoints, int maxIterations = 20, double tolerance = 1e-4)
            {
                if (!setTargetPointsFlag)
                {
                    Debug.LogError("ターゲット点群が設定されていません。");
                    return false;
                }

                for (int iter = 0; iter < maxIterations; iter++)
                {
                    List<Vector2> transformedPoints = new List<Vector2>();
                    foreach (Vector2 p in sourcePoints)
                    {
                        var rotated = rotation * Vector<double>.Build.DenseOfArray(new double[] { p.x, p.y });
                        transformedPoints.Add(new Vector2((float)(rotated[0] + position.x), (float)(rotated[1] + position.y)));
                    }

                    List<Vector2> correspondences = new List<Vector2>();
                    foreach (var p in transformedPoints)
                    {
                        var nearest = targetKdTree.FindNearest(p);
                        correspondences.Add(nearest);
                    }

                    Matrix<double> dR = Matrix<double>.Build.DenseIdentity(2);
                    Vector2 dt = Vector2.zero;
                    estimateTransformation(transformedPoints, correspondences, ref dR, ref dt);

                    rotation = dR * rotation;
                    position += dt;

                    float delta_trans = dt.magnitude;
                    double delta_rot = Math.Abs(Math.Atan2(dR[1, 0], dR[0, 0]));

                    if (delta_trans < tolerance && delta_rot < tolerance)
                    {
                        return true;
                    }
                }
                return false;
            }

            public Vector2 GetEstimatedPosition()
            {
                return position;
            }

            public Matrix<double> GetEstimatedRotation()
            {
                return rotation;
            }

            private void estimateTransformation(List<Vector2> sourcePoints, List<Vector2> targetPoints, ref Matrix<double> R, ref Vector2 t)
            {
                int N = sourcePoints.Count;
                if(N == 0 || targetPoints.Count == 0)
                {
                    Debug.LogError("対応点が存在しません。");
                    return;
                }

                // 重心を計算
                Vector2 sourceCentroid = Vector2.zero;
                Vector2 targetCentroid = Vector2.zero;
                for (int i = 0; i < N; i++)
                {
                    sourceCentroid += sourcePoints[i];
                    targetCentroid += targetPoints[i];
                }
                sourceCentroid /= N;
                targetCentroid /= N;

                // 中心化
                Matrix<double> srcCentered = Matrix<double>.Build.Dense(2, N);
                Matrix<double> tgtCentered = Matrix<double>.Build.Dense(2, N);
                for (int i = 0; i < N; i++)
                {
                    srcCentered[0, i] = sourcePoints[i].x - sourceCentroid.x;
                    srcCentered[1, i] = sourcePoints[i].y - sourceCentroid.y;
                    tgtCentered[0, i] = targetPoints[i].x - targetCentroid.x;
                    tgtCentered[1, i] = targetPoints[i].y - targetCentroid.y;
                }

                // SVDによる回転行列の推定
                Matrix<double> W = tgtCentered * srcCentered.Transpose();
                var svd = W.Svd();
                R = svd.VT * svd.U.Transpose();

                if(R.Determinant() < 0)
                {
                    Matrix<double> diag = Matrix<double>.Build.DiagonalOfDiagonalArray(new double[] { 1, -1 });
                    R = svd.VT * diag * svd.U.Transpose();
                }
                // 並進ベクトルの推定
                Vector<double> targetCentroidVec = Vector<double>.Build.DenseOfArray(new double[] { targetCentroid.x, targetCentroid.y });
                Vector<double> sourceCentroidVec = Vector<double>.Build.DenseOfArray(new double[] { sourceCentroid.x, sourceCentroid.y });

                Vector<double> tVec = targetCentroidVec - R * sourceCentroidVec;
                t = new Vector2((float)tVec[0], (float)tVec[1]);
            }
        }
    }
}