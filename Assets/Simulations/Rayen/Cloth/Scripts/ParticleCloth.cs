using GamiX.Physics;
using GamiX.Physics.Components;
using GamiX.Physics.Rendering;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Cloth
{
    public enum ClothPinMode
    {
        None,
        TopEdge,
        TopCorners,
        LeftEdge,
        /// <summary>Pinned particles follow this transform (flag attached to a moving pole).</summary>
        LeftEdgeFollowTransform
    }

    /// <summary>
    /// Rectangular cloth in the transform's local XZ plane (paper section 8): stretch, shear and bending distance
    /// constraints, optional tethers, aerodynamics (add a ClothAerodynamics next to the world) and optional tearing.
    /// A mesh is generated and deformed every frame (vertex normals computed by hand).
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public sealed class ParticleCloth : ParticleBody
    {
        public Vector2 dimensions = new Vector2(2f, 2f);
        [Tooltip("Particles along X and Z. Keep the spacing close to 2 x particle radius for self collision.")]
        public Vector2Int resolution = new Vector2Int(25, 25);
        public float particleMass = 0.05f;
        [Range(0f, 1f)] public float stretchStiffness = 1f;
        [Range(0f, 1f)] public float shearStiffness = 0.8f;
        [Range(0f, 1f)] public float bendStiffness = 0.3f;
        public ClothPinMode pinMode = ClothPinMode.TopEdge;
        public bool useTethers = true;
        [Tooltip("> 1 enables tearing: a constraint breaks when stretched beyond this ratio.")]
        public float tearRatio = 0f;
        public bool aerodynamics = true;

        public ClothHandle Cloth { get; private set; }

        private Mesh mesh;
        private Vector3[] vertices, normals;
        private int[] localTriangles;
        private Vector3[] pinnedLocal;

        private bool IsPinned(int u, int v)
        {
            int last = resolution.y - 1;
            switch (pinMode)
            {
                case ClothPinMode.TopEdge: return v == last;
                case ClothPinMode.TopCorners: return v == last && (u == 0 || u == resolution.x - 1);
                case ClothPinMode.LeftEdge:
                case ClothPinMode.LeftEdgeFollowTransform: return u == 0;
                default: return false;
            }
        }

        protected override void Create(UnifiedSolver solver)
        {
            renderParticles = false;
            Vector3 origin = transform.TransformPoint(new Vector3(-0.5f * dimensions.x, 0f, -0.5f * dimensions.y));
            Vector3 axisU = transform.TransformVector(new Vector3(dimensions.x, 0f, 0f));
            Vector3 axisV = transform.TransformVector(new Vector3(0f, 0f, dimensions.y));
            Cloth = ClothBuilder.CreateCloth(solver, origin, axisU, axisV, resolution.x, resolution.y, particleMass,
                                             stretchStiffness, shearStiffness, bendStiffness, IsPinned, tearRatio);
            Cloth.aerodynamics = aerodynamics;
            if (useTethers)
                ClothBuilder.AddClothTethers(solver, Cloth);
            Particles.AddRange(Cloth.particles);

            // Local mesh indices (cloth triangles refer to solver indices).
            int n = Cloth.particles.Length;
            var map = new System.Collections.Generic.Dictionary<int, int>(n);
            for (int k = 0; k < n; k++) map[Cloth.particles[k]] = k;
            localTriangles = new int[Cloth.triangles.Length * 2];
            for (int t = 0; t < Cloth.triangles.Length; t += 3)
            {
                int a = map[Cloth.triangles[t]], b = map[Cloth.triangles[t + 1]], c = map[Cloth.triangles[t + 2]];
                // Front and back faces so the cloth is visible from both sides.
                localTriangles[2 * t] = a; localTriangles[2 * t + 1] = b; localTriangles[2 * t + 2] = c;
                localTriangles[2 * t + 3] = a; localTriangles[2 * t + 4] = c; localTriangles[2 * t + 5] = b;
            }
            vertices = new Vector3[n * 2];
            normals = new Vector3[n * 2];
            var uv = new Vector2[n * 2];
            for (int v = 0; v < resolution.y; v++)
            for (int u = 0; u < resolution.x; u++)
            {
                int k = v * resolution.x + u;
                uv[k] = uv[k + n] = new Vector2((float)u / (resolution.x - 1), (float)v / (resolution.y - 1));
            }
            // Back faces use a duplicated vertex set with flipped normals.
            for (int t = 0; t < localTriangles.Length; t += 6)
            {
                localTriangles[t + 3] += n; localTriangles[t + 4] += n; localTriangles[t + 5] += n;
            }
            mesh = new Mesh { name = "GamiX Cloth" };
            mesh.MarkDynamic();
            mesh.vertices = vertices;
            mesh.uv = uv;
            mesh.triangles = localTriangles;
            GetComponent<MeshFilter>().sharedMesh = mesh;
            MeshRenderer mr = GetComponent<MeshRenderer>();
            if (mr.sharedMaterial == null)
                mr.sharedMaterial = RenderUtils.CreateLitMaterial(color);

            if (pinMode == ClothPinMode.LeftEdgeFollowTransform)
            {
                pinnedLocal = new Vector3[n];
                for (int k = 0; k < n; k++)
                    pinnedLocal[k] = transform.InverseTransformPoint(solver.positions[Cloth.particles[k]]);
            }
        }

        public override void PreStep(UnifiedSolver solver, float h)
        {
            if (Cloth == null || pinnedLocal == null) return;
            for (int k = 0; k < Cloth.particles.Length; k++)
            {
                int i = Cloth.particles[k];
                if (solver.invMass[i] == 0f)
                    solver.MovePinnedParticle(i, transform.TransformPoint(pinnedLocal[k]));
            }
        }

        public override void SyncVisual(UnifiedSolver solver)
        {
            if (Cloth == null) return;
            int n = Cloth.particles.Length;
            // The mesh lives in the GameObject's local space.
            for (int k = 0; k < n; k++)
            {
                Vector3 local = transform.InverseTransformPoint(solver.positions[Cloth.particles[k]]);
                vertices[k] = local;
                vertices[k + n] = local;
            }
            RenderUtils.ComputeNormals(vertices, localTriangles, normals);
            mesh.vertices = vertices;
            mesh.normals = normals;
            mesh.RecalculateBounds();
        }
    }
}
