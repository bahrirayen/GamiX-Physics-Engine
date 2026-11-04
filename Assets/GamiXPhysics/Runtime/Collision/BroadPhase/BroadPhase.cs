using System.Collections.Generic;
using GamiX.Physics.Geometry;
using UnityEngine;

namespace GamiX.Physics.Collision
{
    /// <summary>Pair of object indices (a &lt; b) that may collide.</summary>
    public struct BodyPair
    {
        public int a, b;

        public BodyPair(int a, int b)
        {
            if (a < b) { this.a = a; this.b = b; }
            else { this.a = b; this.b = a; }
        }
    }

    public enum BroadPhaseType
    {
        BruteForce,
        SweepAndPrune,
        UniformGrid
    }

    /// <summary>
    /// Broad phase interface (chapter 4, slide 72: "interface IBroadPhase pour pouvoir comparer les algorithmes").
    /// Input: one AABB per object. Output: the list of pairs whose AABBs overlap.
    /// </summary>
    public interface IBroadPhase
    {
        string Name { get; }

        /// <summary>Number of AABB overlap tests done during the last call (to compare the methods).</summary>
        int TestsLastFrame { get; }

        void FindPairs(IReadOnlyList<Aabb> boxes, List<BodyPair> pairs);
    }

    public static class BroadPhaseFactory
    {
        public static IBroadPhase Create(BroadPhaseType type)
        {
            switch (type)
            {
                case BroadPhaseType.BruteForce: return new BruteForceBroadPhase();
                case BroadPhaseType.UniformGrid: return new UniformGridBroadPhase();
                default: return new SweepAndPruneBroadPhase();
            }
        }
    }

    /// <summary>
    /// Exhaustive search (chapter 4, slide 28): (n^2 - n) / 2 tests, no self test, no repeated (A, B)/(B, A).
    /// </summary>
    public sealed class BruteForceBroadPhase : IBroadPhase
    {
        public string Name => "Brute force";
        public int TestsLastFrame { get; private set; }

        public void FindPairs(IReadOnlyList<Aabb> boxes, List<BodyPair> pairs)
        {
            pairs.Clear();
            int tests = 0;
            int n = boxes.Count;
            for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
            {
                tests++;
                if (boxes[i].Overlaps(boxes[j]))
                    pairs.Add(new BodyPair(i, j));
            }
            TestsLastFrame = tests;
        }
    }

    /// <summary>
    /// Sweep and Prune (chapter 4, slides 57-61): objects are sorted by their min on one axis, then swept while a
    /// list of active intervals is kept. The order is kept between frames and re-sorted with an insertion sort,
    /// which is almost O(n) thanks to temporal coherence. The axis of largest spread is chosen automatically.
    /// </summary>
    public sealed class SweepAndPruneBroadPhase : IBroadPhase
    {
        public string Name => "Sweep and Prune";
        public int TestsLastFrame { get; private set; }

        private int[] order = new int[0];
        private int axis;

        public void FindPairs(IReadOnlyList<Aabb> boxes, List<BodyPair> pairs)
        {
            pairs.Clear();
            int n = boxes.Count;
            if (order.Length != n)
            {
                order = new int[n];
                for (int i = 0; i < n; i++)
                    order[i] = i;
            }

            axis = ChooseAxis(boxes);

            // Insertion sort on min[axis] (fast on nearly sorted data).
            for (int i = 1; i < n; i++)
            {
                int key = order[i];
                float keyMin = boxes[key].min[axis];
                int j = i - 1;
                while (j >= 0 && boxes[order[j]].min[axis] > keyMin)
                {
                    order[j + 1] = order[j];
                    j--;
                }
                order[j + 1] = key;
            }

            int tests = 0;
            for (int i = 0; i < n; i++)
            {
                Aabb bi = boxes[order[i]];
                float maxI = bi.max[axis];
                for (int j = i + 1; j < n; j++)
                {
                    Aabb bj = boxes[order[j]];
                    if (bj.min[axis] > maxI)
                        break; // Sorted: no later interval can overlap on this axis.
                    tests++;
                    if (bi.Overlaps(bj))
                        pairs.Add(new BodyPair(order[i], order[j]));
                }
            }
            TestsLastFrame = tests;
        }

        private static int ChooseAxis(IReadOnlyList<Aabb> boxes)
        {
            int n = boxes.Count;
            if (n < 2)
                return 0;
            Vector3 sum = VMath.Zero, sum2 = VMath.Zero;
            for (int i = 0; i < n; i++)
            {
                Vector3 c = boxes[i].Center;
                sum += c;
                sum2 += VMath.Mul(c, c);
            }
            Vector3 variance = sum2 / n - VMath.Mul(sum / n, sum / n);
            return variance.x >= variance.y && variance.x >= variance.z ? 0 : (variance.y >= variance.z ? 1 : 2);
        }
    }

    /// <summary>
    /// Uniform grid / spatial hashing (chapter 4, slide 51): cells of side s, an object is registered in every cell
    /// its AABB touches, and only objects sharing a cell are tested. A pair is reported only in the cell that contains
    /// the min corner of the two boxes' intersection, so it is never reported twice.
    /// </summary>
    public sealed class UniformGridBroadPhase : IBroadPhase
    {
        public string Name => "Uniform grid";
        public int TestsLastFrame { get; private set; }

        /// <summary>Cell size. When &lt;= 0, it is set to twice the average largest half-extent (~ object size).</summary>
        public float cellSize;

        private readonly Dictionary<long, List<int>> cells = new Dictionary<long, List<int>>();
        private readonly List<List<int>> pool = new List<List<int>>();
        private int poolUsed;

        public UniformGridBroadPhase(float cellSize = 0f)
        {
            this.cellSize = cellSize;
        }

        public void FindPairs(IReadOnlyList<Aabb> boxes, List<BodyPair> pairs)
        {
            pairs.Clear();
            int n = boxes.Count;
            float s = cellSize;
            if (s <= 0f)
            {
                float avg = 0f;
                for (int i = 0; i < n; i++)
                    avg += VMath.MaxComponent(boxes[i].Extents);
                s = n > 0 ? Mathf.Max(2f * avg / n, 1e-3f) : 1f;
            }
            float inv = 1f / s;

            cells.Clear();
            poolUsed = 0;
            for (int i = 0; i < n; i++)
            {
                Aabb b = boxes[i];
                int x0 = Mathf.FloorToInt(b.min.x * inv), x1 = Mathf.FloorToInt(b.max.x * inv);
                int y0 = Mathf.FloorToInt(b.min.y * inv), y1 = Mathf.FloorToInt(b.max.y * inv);
                int z0 = Mathf.FloorToInt(b.min.z * inv), z1 = Mathf.FloorToInt(b.max.z * inv);
                for (int x = x0; x <= x1; x++)
                for (int y = y0; y <= y1; y++)
                for (int z = z0; z <= z1; z++)
                {
                    long key = Key(x, y, z);
                    if (!cells.TryGetValue(key, out List<int> list))
                    {
                        list = Rent();
                        cells[key] = list;
                    }
                    list.Add(i);
                }
            }

            int tests = 0;
            foreach (KeyValuePair<long, List<int>> kv in cells)
            {
                List<int> list = kv.Value;
                for (int p = 0; p < list.Count; p++)
                for (int q = p + 1; q < list.Count; q++)
                {
                    int i = list[p], j = list[q];
                    Aabb bi = boxes[i], bj = boxes[j];
                    // Report the pair only in the cell holding the min corner of the overlap region.
                    Vector3 m = VMath.Max(bi.min, bj.min);
                    if (Key(Mathf.FloorToInt(m.x * inv), Mathf.FloorToInt(m.y * inv), Mathf.FloorToInt(m.z * inv)) != kv.Key)
                        continue;
                    tests++;
                    if (bi.Overlaps(bj))
                        pairs.Add(new BodyPair(i, j));
                }
            }
            TestsLastFrame = tests;
        }

        private List<int> Rent()
        {
            if (poolUsed == pool.Count)
                pool.Add(new List<int>(8));
            List<int> list = pool[poolUsed++];
            list.Clear();
            return list;
        }

        private static long Key(int x, int y, int z)
        {
            // 21 bits per axis, enough for +/- 1 million cells.
            return ((long)(x & 0x1FFFFF) << 42) | ((long)(y & 0x1FFFFF) << 21) | (long)(z & 0x1FFFFF);
        }
    }

    /// <summary>
    /// One-dimensional Sweep and Prune on intervals [b_i, e_i] (chapter 4, slides 58-61): the sorted list of all
    /// starts and ends is swept while a list of active intervals is maintained.
    /// </summary>
    public static class SweepAndPrune1D
    {
        private struct Event
        {
            public float value;
            public int index;
            public bool isStart;
        }

        public static void FindOverlaps(IReadOnlyList<float> starts, IReadOnlyList<float> ends, List<BodyPair> pairs)
        {
            pairs.Clear();
            int n = starts.Count;
            Event[] events = new Event[2 * n];
            for (int i = 0; i < n; i++)
            {
                events[2 * i] = new Event { value = starts[i], index = i, isStart = true };
                events[2 * i + 1] = new Event { value = ends[i], index = i, isStart = false };
            }
            // Sort by value; at equal values a start comes before an end (touching intervals overlap).
            System.Array.Sort(events, (x, y) =>
            {
                int c = x.value.CompareTo(y.value);
                if (c != 0) return c;
                return x.isStart == y.isStart ? 0 : (x.isStart ? -1 : 1);
            });

            List<int> active = new List<int>();
            for (int k = 0; k < events.Length; k++)
            {
                Event ev = events[k];
                if (ev.isStart)
                {
                    for (int a = 0; a < active.Count; a++)
                        pairs.Add(new BodyPair(active[a], ev.index));
                    active.Add(ev.index);
                }
                else
                {
                    active.Remove(ev.index);
                }
            }
        }
    }
}
