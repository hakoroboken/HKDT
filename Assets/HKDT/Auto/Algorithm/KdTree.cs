using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Linq;

namespace HKDT.Auto.Algorithm
{
    /// <summary>
    /// ２次元点群の最近傍探索をするKdTreeの実装
    /// </summary>
    public class KdTree
    {
        private KdNode root;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="points">点群</param>
        public KdTree(List<Vector2> points)
        {
            root = Build(points, 0);
        }

        private KdNode Build(List<Vector2> points, int depth)
        {
            if (points == null || points.Count == 0)
            {
                return null;
            }

            int axis = depth % 2; // 0: x軸, 1: y軸

            // 点群を軸に基づいてソート
            if (axis == 0)
            {
                points = points.OrderBy(p => p.x).ToList();
            }
            else
            {
                points = points.OrderBy(p => p.y).ToList();
            }

            int medianIndex = points.Count / 2;
            Vector2 medianPoint = points[medianIndex];

            KdNode node = new KdNode(medianPoint, axis);

            // 左右の部分木を再帰的に構築
            node.left = Build(points.GetRange(0, medianIndex), depth + 1);
            node.right = Build(points.GetRange(medianIndex + 1, points.Count - (medianIndex + 1)), depth + 1);

            return node;
        }

        public Vector2 FindNearest(Vector2 target)
        {
            Vector2 best = root.point;
            float bestDist = Vector2.SqrMagnitude(target - best);
            Search(root, target, ref best, ref bestDist);
            return best;
        }

        private void Search(KdNode node, Vector2 target, ref Vector2 best, ref float bestDist)
        {
            if (node == null) return;

            float dist = Vector2.SqrMagnitude(target - node.point);
            if (dist < bestDist)
            {
                bestDist = dist;
                best = node.point;
            }

            KdNode near, far;
            float diff;

            if (node.axis == 0)
            {
                diff = target.x - node.point.x;
            }
            else
            {
                diff = target.y - node.point.y;
            }

            if (diff < 0)
            {
                near = node.left;
                far = node.right;
            }
            else
            {
                near = node.right;
                far = node.left;
            }

            Search(near, target, ref best, ref bestDist);

            if (diff * diff < bestDist)
            {
                Search(far, target, ref best, ref bestDist);
            }
        }
    }

    public class KdNode
    {
        public Vector2 point;
        public KdNode left;
        public KdNode right;
        public int axis; // 偶数: x軸, 奇数: y軸

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="point"></param>
        public KdNode(Vector2 point, int axis)
        {
            this.point = point;
            this.left = null;
            this.right = null;
            this.axis = axis;
        }
    }
}