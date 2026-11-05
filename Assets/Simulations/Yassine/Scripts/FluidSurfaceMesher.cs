using System;
using System.Collections.Generic;
using GamiX.Physics;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Yassine
{
    /// <summary>
    /// Turns fluid particles into a smooth closed surface, for display only (the physics stays on the particles).
    ///
    /// 1. Each particle splats a smooth bump w = (1 - d^2/R^2)^3 onto a regular grid of nodes, giving a scalar field
    ///    that is large inside the fluid and zero far from it.
    /// 2. The surface is the iso-level f = iso, extracted with Surface Nets: every grid cell that the surface
    ///    crosses gets one vertex (the average of the crossing points on its 12 edges), and every grid edge that
    ///    crosses the surface becomes a quad joining the 4 cells around it. No lookup tables, and the mesh is closed
    ///    and has no cracks.
    /// 3. A few Laplacian smoothing passes remove the lattice ripples, then area-weighted normals are computed with
    ///    our own cross products.
    /// </summary>
    public sealed class FluidSurfaceMesher
    {
        /// <summary>Grid cell size as a fraction of the fluid rest spacing.</summary>
        public float cellScale = 0.5f;
        /// <summary>Splat radius R as a multiple of the fluid rest spacing.</summary>
        public float radiusScale = 1.4f;
        /// <summary>Iso-level of the surface (a lone particle has f = 1 at its center).</summary>
        public float iso = 0.7f;
        public int smoothingIterations = 2;
        /// <summary>Max grid nodes per axis; the cells grow if the fluid is larger.</summary>
        public int maxNodesPerAxis = 140;

        public readonly List<Vector3> vertices = new List<Vector3>();
        public readonly List<Vector3> normals = new List<Vector3>();
        public readonly List<int> triangles = new List<int>();

        private float[] field = new float[0];
        private int[] cellVertex = new int[0];
        private Vector3[] smoothBuffer = new Vector3[0];
        private int[] neighborCount = new int[0];
        private readonly List<int> particles = new List<int>();

        private int nx, ny, nz;
        private Vector3 origin;
        private float cell;

        public int GridNodes => nx * ny * nz;

        /// <summary>Rebuilds the mesh from the given particles (inactive ones are skipped).</summary>
        public void Build(UnifiedSolver s, IEnumerable<int> indices)
        {
            vertices.Clear();
            normals.Clear();
            triangles.Clear();
            particles.Clear();
            foreach (int i in indices)
                if (s.active[i])
                    particles.Add(i);
            if (particles.Count == 0)
                return;

            float spacing = s.parameters.FluidRestDistance;
            float R = radiusScale * spacing;
            Splat(s, spacing, R);
            ExtractVertices();
            ExtractQuads();
            Smooth();
            ComputeNormals();
        }

        // ---------------------------------------------------------------- 1. scalar field

        private void Splat(UnifiedSolver s, float spacing, float R)
        {
            Vector3 lo = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector3 hi = -lo;
            foreach (int i in particles)
            {
                lo = VMath.Min(lo, s.positions[i]);
                hi = VMath.Max(hi, s.positions[i]);
            }
            cell = cellScale * spacing;
            float margin = R + 2f * cell;
            lo -= VMath.One * margin;
            hi += VMath.One * margin;
            // Grow the cells if the grid would be too large.
            float largest = VMath.MaxComponent(hi - lo);
            if (largest / cell + 1f > maxNodesPerAxis)
                cell = largest / (maxNodesPerAxis - 1);
            origin = lo;
            nx = Mathf.CeilToInt((hi.x - lo.x) / cell) + 1;
            ny = Mathf.CeilToInt((hi.y - lo.y) / cell) + 1;
            nz = Mathf.CeilToInt((hi.z - lo.z) / cell) + 1;
            int n = nx * ny * nz;
            if (field.Length < n)
            {
                field = new float[n];
                cellVertex = new int[n];
            }
            Array.Clear(field, 0, n);

            float r2 = R * R, invR2 = 1f / r2, invCell = 1f / cell;
            foreach (int i in particles)
            {
                Vector3 p = s.positions[i] - origin;
                int x0 = Math.Max(0, Mathf.CeilToInt((p.x - R) * invCell)), x1 = Math.Min(nx - 1, Mathf.FloorToInt((p.x + R) * invCell));
                int y0 = Math.Max(0, Mathf.CeilToInt((p.y - R) * invCell)), y1 = Math.Min(ny - 1, Mathf.FloorToInt((p.y + R) * invCell));
                int z0 = Math.Max(0, Mathf.CeilToInt((p.z - R) * invCell)), z1 = Math.Min(nz - 1, Mathf.FloorToInt((p.z + R) * invCell));
                for (int z = z0; z <= z1; z++)
                {
                    float dz = z * cell - p.z;
                    for (int y = y0; y <= y1; y++)
                    {
                        float dy = y * cell - p.y;
                        float dyz = dy * dy + dz * dz;
                        if (dyz >= r2) continue;
                        int row = (z * ny + y) * nx;
                        for (int x = x0; x <= x1; x++)
                        {
                            float dx = x * cell - p.x;
                            float d2 = dx * dx + dyz;
                            if (d2 >= r2) continue;
                            float t = 1f - d2 * invR2;
                            field[row + x] += t * t * t;
                        }
                    }
                }
            }
        }

        private int Node(int x, int y, int z) => (z * ny + y) * nx + x;

        // ---------------------------------------------------------------- 2. surface nets

        // Corner k of a cell is at offset (k & 1, (k >> 1) & 1, (k >> 2) & 1). The 12 edges join corners that differ by one bit.
        private static readonly int[] EdgeA = { 0, 2, 4, 6, 0, 1, 4, 5, 0, 1, 2, 3 };
        private static readonly int[] EdgeB = { 1, 3, 5, 7, 2, 3, 6, 7, 4, 5, 6, 7 };

        private readonly float[] corner = new float[8];

        private void ExtractVertices()
        {
            for (int z = 0; z < nz - 1; z++)
            for (int y = 0; y < ny - 1; y++)
            for (int x = 0; x < nx - 1; x++)
            {
                int c = Node(x, y, z);
                cellVertex[c] = -1;
                int mask = 0;
                for (int k = 0; k < 8; k++)
                {
                    float f = field[Node(x + (k & 1), y + ((k >> 1) & 1), z + ((k >> 2) & 1))];
                    corner[k] = f;
                    if (f > iso) mask |= 1 << k;
                }
                if (mask == 0 || mask == 255)
                    continue;
                Vector3 sum = VMath.Zero;
                int crossings = 0;
                for (int e = 0; e < 12; e++)
                {
                    int a = EdgeA[e], b = EdgeB[e];
                    float fa = corner[a], fb = corner[b];
                    if ((fa > iso) == (fb > iso)) continue;
                    float t = (iso - fa) / (fb - fa);
                    Vector3 pa = new Vector3(a & 1, (a >> 1) & 1, (a >> 2) & 1);
                    Vector3 pb = new Vector3(b & 1, (b >> 1) & 1, (b >> 2) & 1);
                    sum += pa + (pb - pa) * t;
                    crossings++;
                }
                cellVertex[c] = vertices.Count;
                vertices.Add(origin + (new Vector3(x, y, z) + sum / crossings) * cell);
            }
        }

        /// <summary>
        /// One quad per grid edge with a sign change, joining the vertices of the 4 cells around the edge. The winding
        /// makes Cross(b - a, c - a) point out of the fluid (Unity's front face).
        /// </summary>
        private void ExtractQuads()
        {
            for (int z = 0; z < nz; z++)
            for (int y = 0; y < ny; y++)
            for (int x = 0; x < nx; x++)
            {
                bool inside = field[Node(x, y, z)] > iso;
                // Edge along +x: cells (x, y-1..y, z-1..z).
                if (x < nx - 1 && y > 0 && z > 0 && y < ny - 1 && z < nz - 1 && inside != field[Node(x + 1, y, z)] > iso)
                    Quad(Node(x, y - 1, z - 1), Node(x, y, z - 1), Node(x, y, z), Node(x, y - 1, z), inside);
                // Edge along +y: cells (x-1..x, y, z-1..z).
                if (y < ny - 1 && x > 0 && z > 0 && x < nx - 1 && z < nz - 1 && inside != field[Node(x, y + 1, z)] > iso)
                    Quad(Node(x - 1, y, z - 1), Node(x - 1, y, z), Node(x, y, z), Node(x, y, z - 1), inside);
                // Edge along +z: cells (x-1..x, y-1..y, z).
                if (z < nz - 1 && x > 0 && y > 0 && x < nx - 1 && y < ny - 1 && inside != field[Node(x, y, z + 1)] > iso)
                    Quad(Node(x - 1, y - 1, z), Node(x, y - 1, z), Node(x, y, z), Node(x - 1, y, z), inside);
            }
        }

        private void Quad(int ca, int cb, int cc, int cd, bool insideAtStart)
        {
            int a = cellVertex[ca], b = cellVertex[cb], c = cellVertex[cc], d = cellVertex[cd];
            if (a < 0 || b < 0 || c < 0 || d < 0)
                return;
            if (insideAtStart)
            {
                triangles.Add(a); triangles.Add(b); triangles.Add(c);
                triangles.Add(a); triangles.Add(c); triangles.Add(d);
            }
            else
            {
                triangles.Add(a); triangles.Add(c); triangles.Add(b);
                triangles.Add(a); triangles.Add(d); triangles.Add(c);
            }
        }

        // ---------------------------------------------------------------- 3. smoothing and normals

        /// <summary>Laplacian smoothing over the triangle edges (each edge counted from both triangles sharing it).</summary>
        private void Smooth()
        {
            int nv = vertices.Count;
            if (smoothingIterations <= 0 || nv == 0) return;
            if (smoothBuffer.Length < nv)
            {
                smoothBuffer = new Vector3[nv];
                neighborCount = new int[nv];
            }
            for (int it = 0; it < smoothingIterations; it++)
            {
                Array.Clear(smoothBuffer, 0, nv);
                Array.Clear(neighborCount, 0, nv);
                for (int t = 0; t < triangles.Count; t += 3)
                {
                    int a = triangles[t], b = triangles[t + 1], c = triangles[t + 2];
                    Vector3 pa = vertices[a], pb = vertices[b], pc = vertices[c];
                    smoothBuffer[a] += pb + pc; neighborCount[a] += 2;
                    smoothBuffer[b] += pa + pc; neighborCount[b] += 2;
                    smoothBuffer[c] += pa + pb; neighborCount[c] += 2;
                }
                for (int v = 0; v < nv; v++)
                    if (neighborCount[v] > 0)
                        vertices[v] = VMath.Lerp(vertices[v], smoothBuffer[v] / neighborCount[v], 0.5f);
            }
        }

        private void ComputeNormals()
        {
            int nv = vertices.Count;
            for (int v = 0; v < nv; v++)
                normals.Add(VMath.Zero);
            for (int t = 0; t < triangles.Count; t += 3)
            {
                int a = triangles[t], b = triangles[t + 1], c = triangles[t + 2];
                Vector3 n = VMath.Cross(vertices[b] - vertices[a], vertices[c] - vertices[a]);
                normals[a] += n;
                normals[b] += n;
                normals[c] += n;
            }
            for (int v = 0; v < nv; v++)
                normals[v] = VMath.NormalizeOr(normals[v], VMath.Up);
        }

        /// <summary>Volume enclosed by the mesh (divergence theorem), for tests: positive when the faces point outward.</summary>
        public float EnclosedVolume()
        {
            float v = 0f;
            for (int t = 0; t < triangles.Count; t += 3)
                v += VMath.Dot(vertices[triangles[t]], VMath.Cross(vertices[triangles[t + 1]], vertices[triangles[t + 2]])) / 6f;
            return v;
        }
    }
}
