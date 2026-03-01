using System;
using System.Collections.Generic;
using UnityEngine;

using HKDT.Auto.Algorithm;

namespace AuNex
{
    namespace Localization
    {
        public class ICP_KdTree
        {   
            // 推定した並進
            private Vector2 position;
            // 推定した回転（ラジアン）
            private float rotation;
            // KdTreeにより最近傍探索を行うクラス
            private KdTree kdTree;
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
            public ICP_KdTree(float maxCorrespondenceDistance = 0.3f)
            {
                position = Vector2.zero;
                rotation = 0.0f;
                setTargetPointsFlag = false;
                kdTree = null;
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
                // ターゲット(マップ)点群からKdTreeを構築
                kdTree = new KdTree(targetPoints);
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

                        // ICPの進行に応じて対応点探索の最大距離を減少
                        float max_dist_coef = (float)iter / (float)maxIterations;
                        float max_dist = 0.0f;
                        if(max_dist_coef < 0.25)max_dist = maxDistance;
                        else if(max_dist_coef < 0.5)max_dist = maxDistance * 0.75f;
                        else if(max_dist_coef < 0.75)max_dist = maxDistance * 0.5f;
                        else max_dist = maxDistance * 0.25f;

                        // 最近傍点を探索
                        Vector2 nearest = kdTree.FindNearest(rotated);
                        // 最大距離以内なら対応点として追加
                        if (Vector2.SqrMagnitude(rotated - nearest) < max_dist * max_dist)
                        {
                            srcCorr.Add(rotated);
                            tgtCorr.Add(nearest);
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

                    // 距離が近いほど信頼する
                    float w = 1.0f - (Vector2.SqrMagnitude(ps - pt) / (maxDistance * maxDistance));
                    num += w * (ps.x * pt.y - ps.y * pt.x);
                    den += w * (ps.x * pt.x + ps.y * pt.y);
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

                    // 距離が近いほど信頼する
                    float w = 1.0f - (Vector2.SqrMagnitude(ps - pt) / (maxDistance * maxDistance));
                    num += w * (ps.x * pt.y - ps.y * pt.x);
                    den += w * (ps.x * pt.x + ps.y * pt.y);
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