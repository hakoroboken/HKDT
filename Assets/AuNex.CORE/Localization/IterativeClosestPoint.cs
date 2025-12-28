using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using AuNex.Algorithm;

using MathNet.Numerics.LinearAlgebra;
using System;
using Unity.VisualScripting;

namespace AuNex
{
    namespace Localization
    {
        public class ICP_KdTree
        {   
            private Vector2 position;
            private Matrix<double> rotation;
            private KdTree targetKdTree;
            private bool setTargetPointsFlag = false;
            public ICP_KdTree()
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
                    EstimateTransformation(transformedPoints, correspondences, ref dR, ref dt);

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

            private void EstimateTransformation(List<Vector2> sourcePoints, List<Vector2> targetPoints, ref Matrix<double> R, ref Vector2 t)
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

        public class ICP_Hash
        {   
            // 推定した並進
            private Vector2 position;
            // 推定した回転（ラジアン）
            private float rotation;
            // 空間ハッシュにより最近傍探索を行うクラス
            private SpatialHash spatialHash;
            // ターゲット点群が設定されたかどうかのフラグ
            private bool setTargetPointsFlag = false;
            // 最大対応点距離
            private float maxDistance = 0.3f;
            // ソース点群の一時バッファ（GC回避）
            private List<Vector2> srcCorr = new();
            // ターゲット点群の一時バッファ（GC回避）
            private List<Vector2> tgtCorr = new();

            /// <summary>
            /// コンストラクタ
            /// </summary>
            /// <param name="cellSize"></param>
            /// <param name="maxCorrespondenceDistance"></param>
            public ICP_Hash(float cellSize, float maxCorrespondenceDistance = 0.3f)
            {
                position = Vector2.zero;
                rotation = 0.0f;
                spatialHash = new SpatialHash(cellSize);
                setTargetPointsFlag = false;
                maxDistance = maxCorrespondenceDistance;
            }

            /// <summary>
            /// 初期姿勢の設定
            /// </summary>
            /// <param name="initialPosition"></param>
            /// <param name="initialRotation"></param>
            public void SetInitialPose(Vector2 initialPosition, float initialRotation)
            {
                position = initialPosition;
                rotation = initialRotation;
            }

            /// <summary>
            /// ターゲット点群の設定
            /// </summary>
            /// <param name="targetPoints"></param>
            public void SetTargetPoints(List<Vector2> targetPoints)
            {
                // ターゲット(マップ)点群からSpatialHashを構築
                spatialHash.Build(targetPoints);
                setTargetPointsFlag = true;
            }

            /// <summary>
            /// ICPによる位置合わせ
            /// </summary>
            /// <param name="sourcePoints"></param>
            /// <param name="maxIterations"></param>
            /// <param name="tolerance"></param>
            /// <returns>収束したかどうか</returns>
            public bool Align(List<Vector2> sourcePoints, int maxIterations = 20, double tolerance = 1e-4)
            {
                if (!setTargetPointsFlag)
                {
                    Debug.LogError("ターゲット点群が設定されていません。");
                    return false;
                }

                for (int iter = 0; iter < maxIterations; iter++)
                {
                    srcCorr.Clear();
                    tgtCorr.Clear();

                    float cos = Mathf.Cos(rotation);
                    float sin = Mathf.Sin(rotation);

                    // 点群を現在の推定姿勢で変換し最近傍探索
                    foreach (var p in sourcePoints)
                    {
                        //　初期推定により点群を変換
                        Vector2 rotated = new Vector2(
                            cos * p.x - sin * p.y + position.x,
                            sin * p.x + cos * p.y + position.y
                        );
                        // 最近傍点を探索
                        if(spatialHash.FindNearest(rotated, maxDistance, out var nearestPoint))
                        {
                            srcCorr.Add(rotated);
                            tgtCorr.Add(nearestPoint);
                        }
                    }

                    if(srcCorr.Count < 3)
                    {
                        Debug.LogError("対応点が不足しています。");
                        return false;
                    }


                    EstimateTransformation(srcCorr, tgtCorr, out float dTheta, out Vector2 dPos);

                    rotation += dTheta;
                    position += dPos;
                    float delta_trans = dPos.magnitude;
                    double delta_rot = Math.Abs(dTheta);

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

            public float GetEstimatedRotation()
            {
                return rotation;
            }

            private void EstimateTransformation(List<Vector2> sourcePoints, List<Vector2> targetPoints, out float dRot, out Vector2 dPos)
            {
                int N = targetPoints.Count;

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

                float num = 0.0f;
                float den = 0.0f;
                // 点群を中心化してから回転角を推定
                for (int i = 0; i < N; i++)
                {
                    Vector2 ps = sourcePoints[i] - sourceCentroid;
                    Vector2 pt = targetPoints[i] - targetCentroid;

                    num += ps.x * pt.y - ps.y * pt.x;
                    den += ps.x * pt.x + ps.y * pt.y;
                }

                // 最小二乗解としての回転角
                dRot = Mathf.Atan2(num, den);

                // 並進ベクトルの推定
                float cos = Mathf.Cos(dRot);
                float sin = Mathf.Sin(dRot);

                Vector2 rotatedCentroid = new Vector2(
                    cos * sourceCentroid.x - sin * sourceCentroid.y,
                    sin * sourceCentroid.x + cos * sourceCentroid.y
                );

                dPos = targetCentroid - rotatedCentroid;
            }
        }
    }
}