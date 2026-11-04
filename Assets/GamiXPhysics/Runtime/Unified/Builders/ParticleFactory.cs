using System;
using System.Collections.Generic;
using GamiX.Physics.Geometry;
using UnityEngine;

namespace GamiX.Physics.Unified
{
    /// <summary>
    /// Builds the shared particle object types: fluid and granular blocks, and rigid bodies (with their sparse
    /// signed distance field, section 5.1). Simulation-specific builders live with their simulation.
    /// </summary>
    public static class ParticleFactory
    {
        // ================================================================ fluids and granular material

        /// <summary>Block of fluid particles on a lattice of spacing = fluid rest distance.</summary>
        public static List<int> CreateFluidBlock(UnifiedSolver solver, Vector3 center, Vector3 size, float particleMass = 1f,
                                                 Vector3 velocity = default, int group = -1, float jitter = 0.01f)
        {
            if (group < 0) group = solver.NewGroup();
            int phase = Phase.Make(group, Phase.Fluid);
            return FillBox(solver, center, size, solver.parameters.FluidRestDistance, particleMass, velocity, phase, jitter);
        }

        /// <summary>
        /// Block of granular particles (sand, section 6): solid particles that collide with each other and rely on
        /// the position-level static/kinetic friction to form piles.
        /// </summary>
        public static List<int> CreateGranularBlock(UnifiedSolver solver, Vector3 center, Vector3 size, float particleMass = 1f,
                                                    Vector3 velocity = default, float jitter = 0.05f)
        {
            int phase = Phase.Make(solver.NewGroup(), Phase.SelfCollide);
            return FillBox(solver, center, size, solver.parameters.SolidRestDistance * 1.01f, particleMass, velocity, phase, jitter);
        }

        private static List<int> FillBox(UnifiedSolver solver, Vector3 center, Vector3 size, float spacing, float mass,
                                         Vector3 velocity, int phase, float jitter)
        {
            var result = new List<int>();
            int nx = Mathf.Max(1, Mathf.FloorToInt(size.x / spacing));
            int ny = Mathf.Max(1, Mathf.FloorToInt(size.y / spacing));
            int nz = Mathf.Max(1, Mathf.FloorToInt(size.z / spacing));
            Vector3 start = center - new Vector3(nx - 1, ny - 1, nz - 1) * (0.5f * spacing);
            var rng = new System.Random(12345);
            float w = mass > 0f ? 1f / mass : 0f;
            for (int x = 0; x < nx; x++)
            for (int y = 0; y < ny; y++)
            for (int z = 0; z < nz; z++)
            {
                Vector3 j = new Vector3((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f)
                            * (jitter * spacing);
                Vector3 p = start + new Vector3(x, y, z) * spacing + j;
                result.Add(solver.AddParticle(p, velocity, w, phase));
            }
            return result;
        }

        // ================================================================ rigid bodies (section 5)

        /// <summary>
        /// Creates a shape-matched rigid body from local particle positions, their signed distance (negative inside)
        /// and outward SDF gradients. The particles get the same new phase (no self collision), as in section 5.
        /// </summary>
        public static RigidGroup CreateRigid(UnifiedSolver solver, IList<Vector3> localPoints, IList<float> sdf,
                                             IList<Vector3> sdfGradients, Vector3 position, Quat rotation,
                                             float particleMass, Vector3 velocity = default, float stiffness = 1f)
        {
            int n = localPoints.Count;
            if (n == 0)
                throw new ArgumentException("A rigid body needs at least one particle.");
            int group = solver.NewGroup();
            int phase = Phase.Make(group);

            Vector3 com = VMath.Zero;
            for (int k = 0; k < n; k++)
                com += localPoints[k];
            com /= n;

            var rg = new RigidGroup
            {
                indices = new int[n],
                restOffsets = new Vector3[n],
                sdf = new float[n],
                sdfGradient = new Vector3[n],
                stiffness = stiffness,
                rotation = rotation.Normalized,
                localRestCenter = com,
                totalMass = particleMass * n,
                phase = phase
            };
            int rigidIndex = solver.Rigids.Count;
            float w = particleMass > 0f ? 1f / particleMass : 0f;
            for (int k = 0; k < n; k++)
            {
                Vector3 r = localPoints[k] - com;
                Vector3 world = position + rotation.Rotate(r);
                int i = solver.AddParticle(world, velocity, w, phase);
                rg.indices[k] = i;
                rg.restOffsets[k] = r;
                rg.sdf[k] = sdf[k];
                rg.sdfGradient[k] = sdfGradients[k];
                solver.rigidOf[i] = rigidIndex;
                solver.rigidSlot[i] = k;
            }
            rg.center = position;
            solver.Rigids.Add(rg);
            return rg;
        }

        /// <summary>
        /// Rigid box filled with particles. The analytic SDF is measured to a surface half a radius outside the outer
        /// layer, so the outer layer is "boundary" (|phi| &lt; r) and deeper layers are "interior" (section 5.1).
        /// </summary>
        public static RigidGroup CreateRigidBox(UnifiedSolver solver, Vector3 position, Quat rotation, Vector3 size,
                                                float particleMass = 1f, Vector3 velocity = default, float stiffness = 1f)
        {
            float s = solver.parameters.SolidRestDistance;
            float r = solver.parameters.radius;
            int nx = Mathf.Max(1, Mathf.RoundToInt(size.x / s));
            int ny = Mathf.Max(1, Mathf.RoundToInt(size.y / s));
            int nz = Mathf.Max(1, Mathf.RoundToInt(size.z / s));
            Vector3 half = new Vector3(nx - 1, ny - 1, nz - 1) * (0.5f * s);
            Vector3 surface = half + new Vector3(0.5f * r, 0.5f * r, 0.5f * r);

            var pts = new List<Vector3>();
            var phi = new List<float>();
            var grad = new List<Vector3>();
            for (int x = 0; x < nx; x++)
            for (int y = 0; y < ny; y++)
            for (int z = 0; z < nz; z++)
            {
                Vector3 p = new Vector3(x, y, z) * s - half;
                Vector3 q = VMath.Abs(p) - surface; // all components negative (inside)
                Vector3 n;
                float d;
                if (q.x >= q.y && q.x >= q.z) { d = q.x; n = new Vector3(VMath.Sign(p.x), 0f, 0f); }
                else if (q.y >= q.z) { d = q.y; n = new Vector3(0f, VMath.Sign(p.y), 0f); }
                else { d = q.z; n = new Vector3(0f, 0f, VMath.Sign(p.z)); }
                pts.Add(p);
                phi.Add(d);
                grad.Add(n);
            }
            return CreateRigid(solver, pts, phi, grad, position, rotation, particleMass, velocity, stiffness);
        }

        /// <summary>Rigid ball filled with particles, SDF phi = |p| - R.</summary>
        public static RigidGroup CreateRigidSphere(UnifiedSolver solver, Vector3 position, float radius,
                                                   float particleMass = 1f, Vector3 velocity = default, float stiffness = 1f)
        {
            float s = solver.parameters.SolidRestDistance;
            float r = solver.parameters.radius;
            float inner = Mathf.Max(0f, radius - r);
            int m = Mathf.CeilToInt(inner / s);
            var pts = new List<Vector3>();
            var phi = new List<float>();
            var grad = new List<Vector3>();
            float surface = inner + 0.5f * r;
            for (int x = -m; x <= m; x++)
            for (int y = -m; y <= m; y++)
            for (int z = -m; z <= m; z++)
            {
                Vector3 p = new Vector3(x, y, z) * s;
                float len = VMath.Length(p);
                if (len > inner + 1e-4f) continue;
                pts.Add(p);
                phi.Add(len - surface);
                grad.Add(len > 1e-6f ? p / len : VMath.Up);
            }
            if (pts.Count == 0)
            {
                pts.Add(VMath.Zero);
                phi.Add(-0.5f * r);
                grad.Add(VMath.Up);
            }
            return CreateRigid(solver, pts, phi, grad, position, Quat.Identity, particleMass, velocity, stiffness);
        }

        /// <summary>
        /// Rigid body from a closed triangle mesh (section 5): solid voxelization (a voxel is inside when a ray cast
        /// from its center crosses the surface an odd number of times, using the chapter 3 ray-triangle test), then a
        /// distance transform gives the SDF value and gradient of every interior particle.
        /// Mesh vertices are in local space and are scaled by <paramref name="scale"/>.
        /// </summary>
        public static RigidGroup CreateRigidFromMesh(UnifiedSolver solver, Vector3[] vertices, int[] triangles, Vector3 scale,
                                                     Vector3 position, Quat rotation, float particleMass = 1f,
                                                     Vector3 velocity = default, float stiffness = 1f)
        {
            Voxelizer.Voxelize(vertices, triangles, scale, solver.parameters.SolidRestDistance, solver.parameters.radius,
                               out List<Vector3> pts, out List<float> phi, out List<Vector3> grad);
            if (pts.Count == 0)
                throw new ArgumentException("The mesh produced no interior voxel (is it closed and large enough?).");
            return CreateRigid(solver, pts, phi, grad, position, rotation, particleMass, velocity, stiffness);
        }

        // ================================================================ attachments

        /// <summary>Joins a particle to the closest particle of a rigid group with a distance constraint (rope attachments).</summary>
        public static void Attach(UnifiedSolver solver, int particle, RigidGroup rigid, float stiffness = 1f)
        {
            int best = rigid.indices[0];
            float bestD = float.MaxValue;
            foreach (int i in rigid.indices)
            {
                float d = VMath.DistanceSq(solver.positions[i], solver.positions[particle]);
                if (d < bestD) { bestD = d; best = i; }
            }
            solver.AddDistanceConstraint(particle, best, stiffness);
        }

        // ================================================================ geometry helpers

        /// <summary>Unit icosphere with outward (counter-clockwise seen from outside) triangles.</summary>
        public static void Icosphere(int subdivisions, out List<Vector3> vertices, out List<int> triangles)
        {
            float t = (1f + Mathf.Sqrt(5f)) * 0.5f;
            vertices = new List<Vector3>
            {
                new Vector3(-1, t, 0), new Vector3(1, t, 0), new Vector3(-1, -t, 0), new Vector3(1, -t, 0),
                new Vector3(0, -1, t), new Vector3(0, 1, t), new Vector3(0, -1, -t), new Vector3(0, 1, -t),
                new Vector3(t, 0, -1), new Vector3(t, 0, 1), new Vector3(-t, 0, -1), new Vector3(-t, 0, 1)
            };
            for (int i = 0; i < vertices.Count; i++)
                vertices[i] = VMath.Normalize(vertices[i]);
            triangles = new List<int>
            {
                0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11,
                1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9,
                4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1
            };
            for (int s = 0; s < subdivisions; s++)
            {
                var cache = new Dictionary<long, int>();
                var next = new List<int>(triangles.Count * 4);
                List<Vector3> verts = vertices;
                int Mid(int a, int b)
                {
                    long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                    if (cache.TryGetValue(key, out int m)) return m;
                    verts.Add(VMath.Normalize((verts[a] + verts[b]) * 0.5f));
                    cache[key] = verts.Count - 1;
                    return verts.Count - 1;
                }
                for (int k = 0; k < triangles.Count; k += 3)
                {
                    int a = triangles[k], b = triangles[k + 1], c = triangles[k + 2];
                    int ab = Mid(a, b), bc = Mid(b, c), ca = Mid(c, a);
                    next.AddRange(new[] { a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca });
                }
                triangles = next;
            }
            // Make every triangle face outward: its normal must point away from the center (positive volume).
            for (int k = 0; k < triangles.Count; k += 3)
            {
                Vector3 a = vertices[triangles[k]], b = vertices[triangles[k + 1]], c = vertices[triangles[k + 2]];
                if (VMath.Dot(VMath.Cross(b - a, c - a), a + b + c) < 0f)
                {
                    int tmp = triangles[k + 1];
                    triangles[k + 1] = triangles[k + 2];
                    triangles[k + 2] = tmp;
                }
            }
        }
    }
}
