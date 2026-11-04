using System.Collections.Generic;
using GamiX.Physics.Geometry;
using UnityEngine;

namespace GamiX.Physics.Unified
{
    /// <summary>
    /// Solid voxelization of a closed triangle mesh and sparse signed distance field (paper, sections 5 and 5.1).
    /// </summary>
    public static class Voxelizer
    {
        /// <summary>
        /// Places particles at the occupied voxel centers of the mesh interior and computes for each one
        /// phi (signed distance, negative inside) and the outward unit gradient of phi.
        /// </summary>
        /// <param name="spacing">Voxel size (usually the solid rest distance 2r).</param>
        /// <param name="radius">Particle radius, used to place the surface half a radius outside the outer layer.</param>
        public static void Voxelize(Vector3[] vertices, int[] triangles, Vector3 scale, float spacing, float radius,
                                    out List<Vector3> points, out List<float> phi, out List<Vector3> gradients)
        {
            points = new List<Vector3>();
            phi = new List<float>();
            gradients = new List<Vector3>();

            int vCount = vertices.Length;
            var v = new Vector3[vCount];
            Aabb bounds = Aabb.Empty;
            for (int i = 0; i < vCount; i++)
            {
                v[i] = VMath.Mul(vertices[i], scale);
                bounds.Encapsulate(v[i]);
            }

            Vector3 size = bounds.Size;
            int nx = Mathf.Max(1, Mathf.CeilToInt(size.x / spacing));
            int ny = Mathf.Max(1, Mathf.CeilToInt(size.y / spacing));
            int nz = Mathf.Max(1, Mathf.CeilToInt(size.z / spacing));
            Vector3 origin = bounds.Center - new Vector3(nx - 1, ny - 1, nz - 1) * (0.5f * spacing);

            // Inside test: parity of the crossings of a ray cast along +X (chapter 3 ray-triangle intersection).
            // A slightly tilted direction avoids hitting shared edges exactly.
            bool[,,] inside = new bool[nx + 2, ny + 2, nz + 2];
            Vector3 dir = VMath.Normalize(new Vector3(1f, 0.0013f, 0.0021f));
            var tri = new Triangle();
            for (int x = 0; x < nx; x++)
            for (int y = 0; y < ny; y++)
            for (int z = 0; z < nz; z++)
            {
                Vector3 c = origin + new Vector3(x, y, z) * spacing;
                var ray = new Ray3D(c, dir);
                int crossings = 0;
                for (int t = 0; t + 2 < triangles.Length; t += 3)
                {
                    tri.a = v[triangles[t]];
                    tri.b = v[triangles[t + 1]];
                    tri.c = v[triangles[t + 2]];
                    if (Raycast.RayTriangle(ray, tri, out _))
                        crossings++;
                }
                inside[x + 1, y + 1, z + 1] = (crossings & 1) == 1;
            }

            // Exterior voxels touching the interior: they approximate the surface for the distance transform.
            var surface = new List<Vector3>();
            for (int x = 0; x < nx + 2; x++)
            for (int y = 0; y < ny + 2; y++)
            for (int z = 0; z < nz + 2; z++)
            {
                if (inside[x, y, z]) continue;
                bool touches = false;
                for (int k = 0; k < 6 && !touches; k++)
                {
                    int ax = x + (k == 0 ? 1 : k == 1 ? -1 : 0);
                    int ay = y + (k == 2 ? 1 : k == 3 ? -1 : 0);
                    int az = z + (k == 4 ? 1 : k == 5 ? -1 : 0);
                    if (ax >= 0 && ay >= 0 && az >= 0 && ax < nx + 2 && ay < ny + 2 && az < nz + 2 && inside[ax, ay, az])
                        touches = true;
                }
                if (touches)
                    surface.Add(origin + new Vector3(x - 1, y - 1, z - 1) * spacing);
            }

            for (int x = 0; x < nx; x++)
            for (int y = 0; y < ny; y++)
            for (int z = 0; z < nz; z++)
            {
                if (!inside[x + 1, y + 1, z + 1]) continue;
                Vector3 c = origin + new Vector3(x, y, z) * spacing;
                float best = float.MaxValue;
                Vector3 nearest = c + VMath.Up * spacing;
                for (int s = 0; s < surface.Count; s++)
                {
                    float d = VMath.DistanceSq(c, surface[s]);
                    if (d < best)
                    {
                        best = d;
                        nearest = surface[s];
                    }
                }
                // Distance to the surface, which lies between this voxel and the nearest exterior voxel.
                float dist = Mathf.Sqrt(best) - spacing + 0.5f * radius;
                points.Add(c);
                phi.Add(-Mathf.Max(dist, 0.25f * radius));
                gradients.Add(VMath.NormalizeOr(nearest - c, VMath.Up));
            }
        }
    }
}
