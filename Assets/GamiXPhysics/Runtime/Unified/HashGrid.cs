using System;
using UnityEngine;

namespace GamiX.Physics.Unified
{
    /// <summary>
    /// Uniform hash grid for particle neighbour search (paper, section 9; chapter 4 slide 51).
    /// Built with a counting sort: hash every particle to a cell, count particles per bucket, prefix-sum,
    /// scatter the indices. A query visits the 27 cells around a point. O(n) build, no allocation per frame.
    /// </summary>
    public sealed class HashGrid
    {
        private float cellSize = 1f;
        private float invCellSize = 1f;
        private int tableMask;
        private int[] bucketStart = new int[2];
        private int[] sortedIndices = new int[0];
        private int[] particleBucket = new int[0];
        private int count;

        public float CellSize => cellSize;

        private static int Hash(int x, int y, int z)
        {
            unchecked
            {
                return (x * 92837111) ^ (y * 689287499) ^ (z * 283923481);
            }
        }

        private static int NextPowerOfTwo(int v)
        {
            int p = 1;
            while (p < v) p <<= 1;
            return p;
        }

        public void Build(Vector3[] positions, bool[] active, int n, float cell)
        {
            count = n;
            cellSize = cell;
            invCellSize = 1f / cell;
            int tableSize = NextPowerOfTwo(Math.Max(64, 2 * n));
            tableMask = tableSize - 1;

            if (bucketStart.Length != tableSize + 1)
                bucketStart = new int[tableSize + 1];
            else
                Array.Clear(bucketStart, 0, bucketStart.Length);
            if (sortedIndices.Length < n)
            {
                sortedIndices = new int[n];
                particleBucket = new int[n];
            }

            // Count.
            for (int i = 0; i < n; i++)
            {
                if (active != null && !active[i])
                {
                    particleBucket[i] = -1;
                    continue;
                }
                Vector3 p = positions[i];
                int b = Hash(Floor(p.x * invCellSize), Floor(p.y * invCellSize), Floor(p.z * invCellSize)) & tableMask;
                particleBucket[i] = b;
                bucketStart[b + 1]++;
            }
            // Prefix sum.
            for (int b = 0; b < tableSize; b++)
                bucketStart[b + 1] += bucketStart[b];
            // Scatter (use a copy of the starts as write cursors).
            int[] cursor = Scratch(tableSize);
            Array.Copy(bucketStart, cursor, tableSize);
            for (int i = 0; i < n; i++)
            {
                int b = particleBucket[i];
                if (b >= 0)
                    sortedIndices[cursor[b]++] = i;
            }
        }

        private int[] scratch = new int[0];

        private int[] Scratch(int n)
        {
            if (scratch.Length < n)
                scratch = new int[n];
            return scratch;
        }

        private static int Floor(float v)
        {
            int i = (int)v;
            return v < i ? i - 1 : i;
        }

        /// <summary>
        /// Writes into <paramref name="result"/> (starting at <paramref name="offset"/>) the indices of the particles
        /// closer than <paramref name="radius"/> to <paramref name="point"/>, at most <paramref name="max"/>.
        /// <paramref name="exclude"/> is skipped (use -1 for none). Thread safe (read only).
        /// </summary>
        public int Query(Vector3 point, float radius, Vector3[] positions, int[] result, int offset, int max, int exclude)
        {
            float r2 = radius * radius;
            int cx = Floor(point.x * invCellSize), cy = Floor(point.y * invCellSize), cz = Floor(point.z * invCellSize);
            int found = 0;
            // Buckets already visited (hash collisions between the 27 cells would give duplicates).
            Span<int> visited = stackalloc int[27];
            int visitedCount = 0;
            int range = radius <= cellSize ? 1 : (int)Math.Ceiling(radius * invCellSize);
            if (range > 1)
            {
                // Large query radius: no duplicate protection buffer big enough, fall back to a per-index check.
                return QueryLarge(point, radius, positions, result, offset, max, exclude, range);
            }
            for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dz = -1; dz <= 1; dz++)
            {
                int b = Hash(cx + dx, cy + dy, cz + dz) & tableMask;
                bool seen = false;
                for (int v = 0; v < visitedCount; v++)
                {
                    if (visited[v] == b) { seen = true; break; }
                }
                if (seen) continue;
                visited[visitedCount++] = b;

                int end = bucketStart[b + 1];
                for (int k = bucketStart[b]; k < end; k++)
                {
                    int j = sortedIndices[k];
                    if (j == exclude) continue;
                    Vector3 d = positions[j] - point;
                    if (d.x * d.x + d.y * d.y + d.z * d.z < r2)
                    {
                        result[offset + found++] = j;
                        if (found >= max) return found;
                    }
                }
            }
            return found;
        }

        private int QueryLarge(Vector3 point, float radius, Vector3[] positions, int[] result, int offset, int max, int exclude, int range)
        {
            float r2 = radius * radius;
            int cx = Floor(point.x * invCellSize), cy = Floor(point.y * invCellSize), cz = Floor(point.z * invCellSize);
            int found = 0;
            for (int dx = -range; dx <= range; dx++)
            for (int dy = -range; dy <= range; dy++)
            for (int dz = -range; dz <= range; dz++)
            {
                int b = Hash(cx + dx, cy + dy, cz + dz) & tableMask;
                int end = bucketStart[b + 1];
                for (int k = bucketStart[b]; k < end; k++)
                {
                    int j = sortedIndices[k];
                    if (j == exclude) continue;
                    Vector3 d = positions[j] - point;
                    if (d.x * d.x + d.y * d.y + d.z * d.z >= r2) continue;
                    bool dup = false;
                    for (int u = 0; u < found; u++)
                        if (result[offset + u] == j) { dup = true; break; }
                    if (dup) continue;
                    result[offset + found++] = j;
                    if (found >= max) return found;
                }
            }
            return found;
        }
    }
}
