using System.Collections.Generic;
using GamiX.Physics;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Cloth
{
    /// <summary>
    /// Real tearing of a grid cloth built by <see cref="ClothBuilder.CreateCloth"/>: when the distance constraint of a
    /// mesh edge breaks, the edge is marked torn, and every vertex whose ring of triangles is no longer connected
    /// through intact edges is split into one particle per connected group of triangles. The two sides of the tear
    /// then move independently and the rendered mesh opens.
    ///
    /// The topology is kept on the original grid vertices ("original" ids), while the triangles store the particle
    /// currently used at each corner. Every distance constraint remembers which triangle gives each of its two
    /// endpoints, so after a split the constraint is re-attached to the copy on its own side.
    /// </summary>
    public sealed class ClothTearing
    {
        private enum LinkType : byte { Edge, Cross, Bend, Other }

        /// <summary>A distance constraint of the cloth, described on the original grid.</summary>
        private struct Link
        {
            public int constraint;
            public LinkType type;
            public int a, b;          // original vertices of the endpoints (a = constraint.i, b = constraint.j)
            public int triA, triB;    // triangles whose corners give the particle of a and of b
            public int mid;           // middle vertex of a bending constraint
            public long edge;         // mesh edge of an Edge link, diagonal edge crossed by a Cross link
        }

        private readonly UnifiedSolver solver;
        private readonly int resU;
        private readonly int vertexCount;
        private readonly float[] baseInvMass;     // per original vertex (pinned = 0)
        private readonly Vector2[] uvs;           // per original vertex

        private readonly int[] triOrig;           // 3 per triangle: original vertex of each corner (fixed)
        private readonly int[] triPart;           // 3 per triangle: particle used by each corner (changes when splitting)
        private readonly List<int>[] vertexTris;  // triangles around each original vertex
        private readonly Dictionary<long, List<int>> edgeTris = new Dictionary<long, List<int>>();
        private readonly HashSet<long> torn = new HashSet<long>();
        private readonly bool[] detached;          // triangles with two or more torn edges (cut loose)
        private bool detachedChanged;

        private readonly List<Link> links = new List<Link>();
        private readonly bool[] linkBroken;
        private readonly List<int>[] vertexLinks;                                   // links with an endpoint or middle at a vertex
        private readonly Dictionary<long, List<int>> edgeDependents = new Dictionary<long, List<int>>(); // links broken with an edge
        private readonly Dictionary<long, int> edgeLink = new Dictionary<long, int>();                  // mesh edge -> its stretch link
        private readonly float spacing;

        private readonly List<int> meshParticles = new List<int>();   // render vertex -> particle
        private readonly List<int> meshOrig = new List<int>();        // render vertex -> original vertex
        private readonly Dictionary<int, int> particleVertex = new Dictionary<int, int>();

        /// <summary>Incremented every time the topology changes (the renderer rebuilds its mesh).</summary>
        public int Version { get; private set; }
        public int TornEdgeCount => torn.Count;
        /// <summary>Particles created by vertex splits.</summary>
        public int SplitCount { get; private set; }
        /// <summary>Raised for each new particle: (copy, source particle).</summary>
        public event System.Action<int, int> ParticleSplit;

        /// <summary>
        /// An edge tears when it is stretched beyond tearRatio x its rest length (&lt;= 1 disables stretch tearing).
        /// Only the <see cref="maxTearsPerUpdate"/> most stretched edges tear per update: the stress then moves to the
        /// tip of the crack, which runs through the cloth instead of shattering the whole stretched area.
        /// </summary>
        public float tearRatio;
        public int maxTearsPerUpdate = 3;
        private readonly List<KeyValuePair<float, int>> candidates = new List<KeyValuePair<float, int>>();

        /// <param name="firstConstraint">Index of the first distance constraint created for this cloth.</param>
        /// <param name="constraintCount">Number of distance constraints created for this cloth.</param>
        public ClothTearing(UnifiedSolver solver, ClothHandle cloth, int firstConstraint, int constraintCount)
        {
            this.solver = solver;
            resU = cloth.width;
            int resV = cloth.height;
            vertexCount = cloth.particles.Length;

            var origOf = new Dictionary<int, int>(vertexCount);
            baseInvMass = new float[vertexCount];
            uvs = new Vector2[vertexCount];
            for (int k = 0; k < vertexCount; k++)
            {
                int p = cloth.particles[k];
                origOf[p] = k;
                baseInvMass[k] = solver.invMass[p];
                uvs[k] = new Vector2((float)(k % resU) / Mathf.Max(1, resU - 1), (float)(k / resU) / Mathf.Max(1, resV - 1));
                particleVertex[p] = meshParticles.Count;
                meshParticles.Add(p);
                meshOrig.Add(k);
            }

            // Triangles and their edges.
            int triCount = cloth.triangles.Length / 3;
            triOrig = new int[cloth.triangles.Length];
            triPart = new int[cloth.triangles.Length];
            vertexTris = new List<int>[vertexCount];
            vertexLinks = new List<int>[vertexCount];
            for (int k = 0; k < vertexCount; k++)
            {
                vertexTris[k] = new List<int>(6);
                vertexLinks[k] = new List<int>(12);
            }
            detached = new bool[triCount];
            for (int t = 0; t < triCount; t++)
            {
                for (int c = 0; c < 3; c++)
                {
                    int p = cloth.triangles[3 * t + c];
                    triPart[3 * t + c] = p;
                    triOrig[3 * t + c] = origOf[p];
                    vertexTris[origOf[p]].Add(t);
                }
                for (int c = 0; c < 3; c++)
                {
                    long e = EdgeKey(triOrig[3 * t + c], triOrig[3 * t + (c + 1) % 3]);
                    if (!edgeTris.TryGetValue(e, out List<int> list))
                        edgeTris[e] = list = new List<int>(2);
                    list.Add(t);
                }
            }

            // Classify the cloth's distance constraints on the grid.
            for (int c = firstConstraint; c < firstConstraint + constraintCount; c++)
            {
                DistanceConstraint dc = solver.DistanceConstraints[c];
                if (!origOf.TryGetValue(dc.i, out int a) || !origOf.TryGetValue(dc.j, out int b)) continue;
                var link = new Link { constraint = c, a = a, b = b, mid = -1, type = LinkType.Other };
                int du = b % resU - a % resU, dv = b / resU - a / resU;
                long ab = EdgeKey(a, b);
                if (edgeTris.TryGetValue(ab, out List<int> tris))
                {
                    // Stretch edges and the diagonal used by the triangles.
                    link.type = LinkType.Edge;
                    link.edge = ab;
                    link.triA = link.triB = tris[0];
                    AddDependent(ab, links.Count);
                    edgeLink[ab] = links.Count;
                }
                else if (System.Math.Abs(du) == 1 && System.Math.Abs(dv) == 1)
                {
                    // Shear across the quad: it crosses the quad's other diagonal.
                    int u0 = System.Math.Min(a % resU, b % resU), v0 = System.Math.Min(a / resU, b / resU);
                    int p = v0 * resU + u0 + 1, q = (v0 + 1) * resU + u0; // the quad's other diagonal
                    if (!edgeTris.TryGetValue(EdgeKey(p, q), out List<int> diag) || diag.Count < 2) continue;
                    link.type = LinkType.Cross;
                    link.edge = EdgeKey(p, q);
                    link.triA = Contains(diag[0], a) ? diag[0] : diag[1];
                    link.triB = Contains(diag[0], b) ? diag[0] : diag[1];
                    AddDependent(link.edge, links.Count);
                }
                else if ((System.Math.Abs(du) == 2 && dv == 0) || (du == 0 && System.Math.Abs(dv) == 2))
                {
                    // Bending: a - mid - b along a grid line.
                    int mid = (a + b) / 2;
                    long e1 = EdgeKey(a, mid), e2 = EdgeKey(mid, b);
                    if (!edgeTris.TryGetValue(e1, out List<int> t1) || !edgeTris.TryGetValue(e2, out List<int> t2)) continue;
                    link.type = LinkType.Bend;
                    link.mid = mid;
                    link.triA = t1[0];
                    link.triB = t2[0];
                    AddDependent(e1, links.Count);
                    AddDependent(e2, links.Count);
                    vertexLinks[mid].Add(links.Count);
                }
                else
                {
                    continue;
                }
                vertexLinks[a].Add(links.Count);
                vertexLinks[b].Add(links.Count);
                links.Add(link);
            }
            linkBroken = new bool[links.Count];
            spacing = resU > 1 ? VMath.Distance(solver.positions[cloth.particles[0]], solver.positions[cloth.particles[1]]) : 0.1f;
        }

        /// <summary>Rest distance between neighboring particles of the grid.</summary>
        public float Spacing => spacing;

        // ================================================================ queries for the renderer

        /// <summary>Number of render vertices (one per particle of the cloth, copies included).</summary>
        public int MeshVertexCount => meshParticles.Count;
        public int MeshParticle(int vertex) => meshParticles[vertex];
        public Vector2 MeshUv(int vertex) => uvs[meshOrig[vertex]];

        /// <summary>
        /// Fills the render triangles (3 render vertex indices per triangle). Triangles with two or more torn edges
        /// are left out: nothing holds their corners together any more.
        /// </summary>
        public void GetMeshTriangles(List<int> result)
        {
            result.Clear();
            for (int t = 0; t < detached.Length; t++)
            {
                if (detached[t]) continue;
                for (int c = 0; c < 3; c++)
                    result.Add(particleVertex[triPart[3 * t + c]]);
            }
        }

        /// <summary>Number of separate pieces (triangles connected through edges whose two corners are shared).</summary>
        public int CountPieces()
        {
            int n = triPart.Length / 3;
            var parent = new int[n];
            for (int t = 0; t < n; t++) parent[t] = t;
            var byEdge = new Dictionary<long, int>();
            for (int t = 0; t < n; t++)
            {
                if (detached[t]) continue;
                for (int c = 0; c < 3; c++)
                {
                    long e = EdgeKey(triPart[3 * t + c], triPart[3 * t + (c + 1) % 3]);
                    if (byEdge.TryGetValue(e, out int other))
                        Union(parent, t, other);
                    else
                        byEdge[e] = t;
                }
            }
            int pieces = 0;
            for (int t = 0; t < n; t++)
                if (!detached[t] && Find(parent, t) == t) pieces++;
            return pieces;
        }

        // ================================================================ tearing

        /// <summary>
        /// Cuts the cloth along a blade moved from a to b. Both points are snapped to the closest vertex of the cloth
        /// (within <paramref name="snapDistance"/>), and the grid edges of a path between the two vertices are torn.
        /// Tearing along mesh edges gives a clean slit: the vertices of the path split in two and no triangle is lost.
        /// The split itself happens in the next <see cref="Update"/>. Returns the number of edges torn.
        /// </summary>
        public int Cut(Vector3 a, Vector3 b, float snapDistance)
        {
            if (!NearestVertex(a, snapDistance, out int from) || !NearestVertex(b, snapDistance, out int to))
                return 0;
            int u = from % resU, v = from / resU, ut = to % resU, vt = to / resU;
            int cut = 0;
            while (u != ut || v != vt)
            {
                int du = System.Math.Sign(ut - u), dv = System.Math.Sign(vt - v);
                int nu = u, nv = v;
                if (du != 0 && dv != 0 && du == -dv)
                {
                    // The triangles' diagonal goes from (u+1, v) to (u, v+1): one step covers both directions.
                    nu += du;
                    nv += dv;
                }
                else if (System.Math.Abs(ut - u) >= System.Math.Abs(vt - v))
                    nu += du;
                else
                    nv += dv;
                if (edgeLink.TryGetValue(EdgeKey(v * resU + u, nv * resU + nu), out int l))
                {
                    ref DistanceConstraint dc = ref solver.DistanceConstraints[links[l].constraint];
                    if (!dc.broken)
                    {
                        dc.broken = true;
                        cut++;
                    }
                }
                u = nu;
                v = nv;
            }
            return cut;
        }

        /// <summary>Original vertex whose particle (or copy) is closest to p, within maxDistance.</summary>
        public bool NearestVertex(Vector3 p, float maxDistance, out int vertex)
        {
            vertex = -1;
            float best = maxDistance * maxDistance;
            for (int k = 0; k < meshParticles.Count; k++)
            {
                float d = VMath.DistanceSq(solver.positions[meshParticles[k]], p);
                if (d < best)
                {
                    best = d;
                    vertex = meshOrig[k];
                }
            }
            return vertex >= 0;
        }

        /// <summary>
        /// Turns newly broken constraints into torn edges and splits the vertices that became disconnected.
        /// Call it outside the solver step (before it). Returns true when the mesh changed.
        /// </summary>
        public bool Update()
        {
            if (tearRatio > 1f)
                TearMostStretched();
            var dirty = new HashSet<int>();
            for (int l = 0; l < links.Count; l++)
            {
                if (linkBroken[l] || !solver.DistanceConstraints[links[l].constraint].broken) continue;
                linkBroken[l] = true;
                if (links[l].type == LinkType.Edge)
                    TearEdge(links[l].edge, dirty);
            }
            if (dirty.Count == 0)
                return false;

            bool changed = detachedChanged;
            detachedChanged = false;
            foreach (int v in dirty)
                changed |= SplitVertex(v);

            // Re-attach the constraints around the split vertices to the copy on their side.
            var touched = new HashSet<int>();
            foreach (int v in dirty)
                foreach (int l in vertexLinks[v])
                    touched.Add(l);
            foreach (int l in touched)
                Reattach(l);

            if (changed)
                Version++;
            return changed;
        }

        private void TearMostStretched()
        {
            candidates.Clear();
            float limit2 = tearRatio * tearRatio;
            for (int l = 0; l < links.Count; l++)
            {
                if (links[l].type != LinkType.Edge || linkBroken[l]) continue;
                DistanceConstraint dc = solver.DistanceConstraints[links[l].constraint];
                if (dc.broken) continue;
                float ratio2 = VMath.DistanceSq(solver.positions[dc.i], solver.positions[dc.j]) / (dc.restLength * dc.restLength);
                if (ratio2 > limit2)
                    candidates.Add(new KeyValuePair<float, int>(ratio2, l));
            }
            if (candidates.Count == 0) return;
            candidates.Sort((x, y) => y.Key.CompareTo(x.Key));
            for (int k = 0; k < candidates.Count && k < maxTearsPerUpdate; k++)
                solver.DistanceConstraints[links[candidates[k].Value].constraint].broken = true;
        }

        private void TearEdge(long edge, HashSet<int> dirty)
        {
            if (!torn.Add(edge))
                return;
            dirty.Add((int)(edge >> 32));
            dirty.Add((int)(edge & 0xffffffff));
            if (edgeTris.TryGetValue(edge, out List<int> sides))
            {
                foreach (int t in sides)
                {
                    if (detached[t]) continue;
                    // A triangle that lost two edges hangs by one edge only: the corner between the two torn edges
                    // has no constraint left and would drift away, stretching the triangle. It is cut loose.
                    int tornEdges = 0;
                    for (int c = 0; c < 3; c++)
                        if (torn.Contains(EdgeKey(triOrig[3 * t + c], triOrig[3 * t + (c + 1) % 3]))) tornEdges++;
                    if (tornEdges >= 2)
                    {
                        detached[t] = true;
                        detachedChanged = true;
                        for (int c = 0; c < 3; c++)
                            dirty.Add(triOrig[3 * t + c]);
                    }
                }
            }
            // Shear and bending constraints that rely on this edge break with it, otherwise they would hold the
            // two sides of the tear together.
            if (edgeDependents.TryGetValue(edge, out List<int> deps))
            {
                foreach (int l in deps)
                {
                    if (links[l].type == LinkType.Edge) continue;
                    ref DistanceConstraint dc = ref solver.DistanceConstraints[links[l].constraint];
                    dc.broken = true;
                    linkBroken[l] = true;
                }
            }
        }

        /// <summary>
        /// Groups the triangles around v that are connected through intact edges (union-find), and gives each group
        /// its own particle. The largest group keeps the particle it already uses.
        /// </summary>
        private bool SplitVertex(int v)
        {
            var tris = new List<int>(vertexTris[v].Count);
            foreach (int t in vertexTris[v])
                if (!detached[t]) tris.Add(t);
            int n = tris.Count;
            if (n < 2)
                return false;
            var parent = new int[n];
            for (int i = 0; i < n; i++) parent[i] = i;
            for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
            {
                // Triangles i and j touch v; they are connected if they share an intact edge (v, w).
                int w = SharedOtherVertex(tris[i], tris[j], v);
                if (w >= 0 && !torn.Contains(EdgeKey(v, w)))
                    Union(parent, i, j);
            }

            var groups = new Dictionary<int, List<int>>();
            for (int i = 0; i < n; i++)
            {
                int root = Find(parent, i);
                if (!groups.TryGetValue(root, out List<int> g))
                    groups[root] = g = new List<int>();
                g.Add(tris[i]);
            }
            if (groups.Count < 2)
                return false;

            var ordered = new List<List<int>>(groups.Values);
            ordered.Sort((x, y) => y.Count.CompareTo(x.Count));
            var claimed = new HashSet<int>();
            bool changed = false;
            foreach (List<int> group in ordered)
            {
                // Keep the particle most of this group already uses, unless another group took it.
                var votes = new Dictionary<int, int>();
                foreach (int t in group)
                {
                    int p = Corner(t, v);
                    votes.TryGetValue(p, out int count);
                    votes[p] = count + 1;
                }
                int best = -1, bestVotes = -1, source = -1;
                foreach (KeyValuePair<int, int> kv in votes)
                {
                    if (source < 0 || kv.Value > votes[source]) source = kv.Key;
                    if (claimed.Contains(kv.Key)) continue;
                    if (kv.Value > bestVotes) { best = kv.Key; bestVotes = kv.Value; }
                }
                if (best < 0)
                {
                    best = CopyParticle(source, v);
                    changed = true;
                }
                claimed.Add(best);
                foreach (int t in group)
                {
                    if (Corner(t, v) != best)
                    {
                        SetCorner(t, v, best);
                        changed = true;
                    }
                }
            }
            return changed;
        }

        private int CopyParticle(int source, int v)
        {
            int copy = solver.AddParticle(solver.positions[source], solver.velocities[source], baseInvMass[v], solver.phases[source]);
            solver.predicted[copy] = solver.predicted[source];
            // Same rest position: the two sides of the tear do not collide with each other (self-collision filter).
            solver.restPositions[copy] = solver.restPositions[source];
            particleVertex[copy] = meshParticles.Count;
            meshParticles.Add(copy);
            meshOrig.Add(v);
            SplitCount++;
            ParticleSplit?.Invoke(copy, source);
            return copy;
        }

        private void Reattach(int l)
        {
            Link link = links[l];
            ref DistanceConstraint dc = ref solver.DistanceConstraints[link.constraint];
            if (dc.broken) return;
            // The endpoints are read from triangles that are still part of the cloth.
            int ta = link.triA, tb = link.triB;
            switch (link.type)
            {
                case LinkType.Edge:
                    ta = tb = LiveTriangle(link.edge);
                    break;
                case LinkType.Bend:
                    ta = LiveTriangle(EdgeKey(link.a, link.mid));
                    tb = LiveTriangle(EdgeKey(link.mid, link.b));
                    break;
                case LinkType.Cross:
                    if (detached[ta] || detached[tb]) ta = -1;
                    break;
            }
            // No triangle left on one side, or the middle vertex of a bend split between its two halves:
            // the constraint no longer holds anything.
            if (ta < 0 || tb < 0 || (link.type == LinkType.Bend && Corner(ta, link.mid) != Corner(tb, link.mid)))
            {
                dc.broken = true;
                linkBroken[l] = true;
                return;
            }
            dc.i = Corner(ta, link.a);
            dc.j = Corner(tb, link.b);
        }

        // ================================================================ helpers

        private void AddDependent(long edge, int link)
        {
            if (!edgeDependents.TryGetValue(edge, out List<int> list))
                edgeDependents[edge] = list = new List<int>(4);
            list.Add(link);
        }

        /// <summary>A triangle of the edge that is not cut loose, or -1.</summary>
        private int LiveTriangle(long edge)
        {
            if (!edgeTris.TryGetValue(edge, out List<int> list)) return -1;
            foreach (int t in list)
                if (!detached[t]) return t;
            return -1;
        }

        private bool Contains(int t, int v) => triOrig[3 * t] == v || triOrig[3 * t + 1] == v || triOrig[3 * t + 2] == v;

        private int Corner(int t, int v)
        {
            for (int c = 0; c < 3; c++)
                if (triOrig[3 * t + c] == v)
                    return triPart[3 * t + c];
            return -1;
        }

        private void SetCorner(int t, int v, int particle)
        {
            for (int c = 0; c < 3; c++)
                if (triOrig[3 * t + c] == v)
                    triPart[3 * t + c] = particle;
        }

        /// <summary>The vertex other than v shared by triangles t1 and t2, or -1.</summary>
        private int SharedOtherVertex(int t1, int t2, int v)
        {
            for (int c = 0; c < 3; c++)
            {
                int w = triOrig[3 * t1 + c];
                if (w != v && Contains(t2, w))
                    return w;
            }
            return -1;
        }

        private static long EdgeKey(int a, int b) => a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;

        private static int Find(int[] parent, int i)
        {
            while (parent[i] != i)
            {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            return i;
        }

        private static void Union(int[] parent, int i, int j)
        {
            int a = Find(parent, i), b = Find(parent, j);
            if (a != b) parent[a] = b;
        }
    }
}
