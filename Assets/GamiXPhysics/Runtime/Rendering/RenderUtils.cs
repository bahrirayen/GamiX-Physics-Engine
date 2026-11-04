using System.Collections.Generic;
using UnityEngine;

namespace GamiX.Physics.Rendering
{
    /// <summary>Materials and procedural meshes for the renderers (display only, no physics here).</summary>
    public static class RenderUtils
    {
        private static readonly Dictionary<int, Material> MaterialCache = new Dictionary<int, Material>();
        private static Mesh sphereMesh;

        /// <summary>Lit material of the active pipeline (URP Lit, or Standard as fallback), with GPU instancing.</summary>
        public static Material CreateLitMaterial(Color color)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null) shader = Shader.Find("Standard");
            var mat = new Material(shader) { enableInstancing = true };
            SetColor(mat, color);
            return mat;
        }

        public static void SetColor(Material mat, Color color)
        {
            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", color);
            if (mat.HasProperty("_Color")) mat.SetColor("_Color", color);
        }

        /// <summary>Shared instancing material for a color.</summary>
        public static Material GetSharedMaterial(Color color)
        {
            int key = ((int)(color.r * 255f) << 24) | ((int)(color.g * 255f) << 16) | ((int)(color.b * 255f) << 8) | (int)(color.a * 255f);
            if (!MaterialCache.TryGetValue(key, out Material mat) || mat == null)
            {
                mat = CreateLitMaterial(color);
                MaterialCache[key] = mat;
            }
            return mat;
        }

        /// <summary>Low-poly unit-diameter sphere for instanced particles.</summary>
        public static Mesh SphereMesh
        {
            get
            {
                if (sphereMesh == null)
                {
                    Unified.ParticleFactory.Icosphere(1, out List<Vector3> verts, out List<int> tris);
                    var v = new Vector3[verts.Count];
                    for (int i = 0; i < v.Length; i++) v[i] = verts[i] * 0.5f;
                    sphereMesh = new Mesh { name = "GamiX Particle Sphere", vertices = v, triangles = tris.ToArray() };
                    var normals = new Vector3[v.Length];
                    for (int i = 0; i < v.Length; i++) normals[i] = verts[i];
                    sphereMesh.normals = normals;
                    sphereMesh.bounds = new Bounds(Vector3.zero, Vector3.one);
                }
                return sphereMesh;
            }
        }

        /// <summary>Area-weighted vertex normals computed with our own cross products.</summary>
        public static void ComputeNormals(Vector3[] vertices, int[] triangles, Vector3[] normals)
        {
            for (int i = 0; i < normals.Length; i++)
                normals[i] = VMath.Zero;
            for (int t = 0; t + 2 < triangles.Length; t += 3)
            {
                int a = triangles[t], b = triangles[t + 1], c = triangles[t + 2];
                Vector3 n = VMath.Cross(vertices[b] - vertices[a], vertices[c] - vertices[a]);
                normals[a] += n;
                normals[b] += n;
                normals[c] += n;
            }
            for (int i = 0; i < normals.Length; i++)
                normals[i] = VMath.NormalizeOr(normals[i], VMath.Up);
        }

        /// <summary>Instance matrix for a sphere of the given diameter at position p (built by hand).</summary>
        public static Matrix4x4 ScaleTranslation(Vector3 p, float diameter)
        {
            Matrix4x4 m = default;
            m.m00 = diameter; m.m11 = diameter; m.m22 = diameter; m.m33 = 1f;
            m.m03 = p.x; m.m13 = p.y; m.m23 = p.z;
            return m;
        }
    }
}
