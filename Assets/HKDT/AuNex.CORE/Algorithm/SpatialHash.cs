using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AuNex
{
    namespace Algorithm
    {
        public class SpatialHash
        {
            private readonly float cellSize;
            private readonly float invCellSize;

            private readonly Dictionary<long, List<Vector2>> grid =new Dictionary<long, List<Vector2>>(1024);

            // 一時バッファ（GC回避）
            private readonly List<Vector2> searchBuffer = new List<Vector2>(32);

            public SpatialHash(float cellSize)
            {
                this.cellSize = cellSize;
                this.invCellSize = 1f / cellSize;
            }

            /// <summary>
            /// 点群を登録（初回 or 更新時のみ）
            /// </summary>
            public void Build(IList<Vector2> points)
            {
                grid.Clear();

                for (int i = 0; i < points.Count; i++)
                {
                    var p = points[i];
                    long key = Hash(p);

                    if (!grid.TryGetValue(key, out var list))
                    {
                        list = new List<Vector2>(4);
                        grid[key] = list;
                    }
                    list.Add(p);
                }
            }

            /// <summary>
            /// 最近傍探索（最大距離制限付き）
            /// </summary>
            public bool FindNearest(Vector2 target, float maxDistance, out Vector2 nearest)
            {
                float maxDistSq = maxDistance * maxDistance;
                nearest = Vector2.zero;
                bool found = false;

                searchBuffer.Clear();

                int cx = Mathf.FloorToInt(target.x * invCellSize);
                int cy = Mathf.FloorToInt(target.y * invCellSize);

                // 周囲3x3セルのみ探索
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        long key = Hash(cx + dx, cy + dy);
                        if (grid.TryGetValue(key, out var list))
                        {
                            searchBuffer.AddRange(list);
                        }
                    }
                }

                for (int i = 0; i < searchBuffer.Count; i++)
                {
                    Vector2 p = searchBuffer[i];
                    float d = (p - target).sqrMagnitude;

                    if (d < maxDistSq)
                    {
                        maxDistSq = d;
                        nearest = p;
                        found = true;
                    }
                }

                return found;
            }


            private long Hash(Vector2 p)
            {
                int x = Mathf.FloorToInt(p.x * invCellSize);
                int y = Mathf.FloorToInt(p.y * invCellSize);
                return Hash(x, y);
            }

            private long Hash(int x, int y)
            {
                // 32bit + 32bit → 64bit
                return ((long)x << 32) ^ (uint)y;
            }
        }
    }
}

