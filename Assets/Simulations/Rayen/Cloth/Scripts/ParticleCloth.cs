using System.Collections.Generic;
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
    /// constraints, optional tethers, aerodynamics (add a ClothAerodynamics next to the world) and tearing.
    /// With <see cref="realTearing"/>, torn edges split the mesh (<see cref="ClothTearing"/>) so the cloth really opens.
    /// A double-sided mesh is generated and deformed every frame (vertex normals computed by hand).
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
        [Tooltip("Split the mesh along torn edges (otherwise only the constraints break).")]
        public bool realTearing = true;
        [Tooltip("Can be cut with the ClothCutter blade.")]
        public bool cuttable = true;
        public bool aerodynamics = true;
        [Tooltip("Optional texture (UVs follow the grid and stay correct across tears).")]
        public Texture2D texture;
        [Range(0f, 1f)] public float smoothness = 0.15f;

        public ClothHandle Cloth { get; private set; }
        public ClothTearing Tearing { get; private set; }
        public int FirstConstraint { get; private set; }
        public int ConstraintCount { get; private set; }

        private Mesh mesh;
        private Vector3[] vertices, normals;
        private Vector2[] meshUv;
        private int[] meshTriangles;
        private readonly List<int> triangleBuffer = new List<int>();
        private int meshVersion = -1;
        private readonly Dictionary<int, Vector3> pinnedLocal = new Dictionary<int, Vector3>();

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
            FirstConstraint = solver.DistanceConstraints.Count;
            // With real tearing, ClothTearing decides which edges tear (crack propagation), not the solver.
            Cloth = ClothBuilder.CreateCloth(solver, origin, axisU, axisV, resolution.x, resolution.y, particleMass,
                                             stretchStiffness, shearStiffness, bendStiffness, IsPinned, realTearing ? 0f : tearRatio);
            ConstraintCount = solver.DistanceConstraints.Count - FirstConstraint;
            Cloth.aerodynamics = aerodynamics;
            if (useTethers)
                ClothBuilder.AddClothTethers(solver, Cloth);
            Particles.AddRange(Cloth.particles);

            Tearing = new ClothTearing(solver, Cloth, FirstConstraint, ConstraintCount) { tearRatio = tearRatio };
            Tearing.ParticleSplit += OnParticleSplit;

            if (pinMode == ClothPinMode.LeftEdgeFollowTransform)
            {
                foreach (int i in Cloth.particles)
                    if (solver.invMass[i] == 0f)
                        pinnedLocal[i] = transform.InverseTransformPoint(solver.positions[i]);
            }

            mesh = new Mesh { name = "GamiX Cloth" };
            mesh.MarkDynamic();
            GetComponent<MeshFilter>().sharedMesh = mesh;
            MeshRenderer mr = GetComponent<MeshRenderer>();
            if (mr.sharedMaterial == null)
            {
                Material mat = RenderUtils.CreateLitMaterial(texture != null ? Color.white : color);
                if (texture != null && mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", texture);
                if (texture != null && mat.HasProperty("_MainTex")) mat.SetTexture("_MainTex", texture);
                if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", smoothness);
                mr.sharedMaterial = mat;
            }
            RebuildMesh(solver);
        }

        private void OnParticleSplit(int copy, int source)
        {
            Particles.Add(copy);
            if (pinnedLocal.TryGetValue(source, out Vector3 local))
                pinnedLocal[copy] = local;
        }

        /// <summary>Tears the cloth along a blade path (see <see cref="ClothTearing.Cut"/>).</summary>
        public int Cut(Vector3 from, Vector3 to) => Tearing != null ? Tearing.Cut(from, to, 0.75f * Tearing.Spacing) : 0;

        /// <summary>Largest stretch |x_i - x_j| / L - 1 over the intact stretch constraints of this cloth.</summary>
        public float MaxStretch(UnifiedSolver solver)
        {
            float max = 0f;
            for (int c = FirstConstraint; c < FirstConstraint + ConstraintCount; c++)
            {
                DistanceConstraint d = solver.DistanceConstraints[c];
                if (d.broken || d.kind != 0) continue;
                max = Mathf.Max(max, VMath.Distance(solver.positions[d.i], solver.positions[d.j]) / d.restLength - 1f);
            }
            return max;
        }

        public override void PreStep(UnifiedSolver solver, float h)
        {
            if (Cloth == null) return;
            if (realTearing)
                Tearing.Update();
            foreach (KeyValuePair<int, Vector3> kv in pinnedLocal)
            {
                if (solver.invMass[kv.Key] == 0f)
                    solver.MovePinnedParticle(kv.Key, transform.TransformPoint(kv.Value));
            }
        }

        public override void SyncVisual(UnifiedSolver solver)
        {
            if (Cloth == null) return;
            if (meshVersion != Tearing.Version)
                RebuildMesh(solver);
            int n = Tearing.MeshVertexCount;
            // The mesh lives in the GameObject's local space; the second half of the vertices is the back face.
            for (int k = 0; k < n; k++)
            {
                Vector3 local = transform.InverseTransformPoint(solver.positions[Tearing.MeshParticle(k)]);
                vertices[k] = local;
                vertices[k + n] = local;
            }
            RenderUtils.ComputeNormals(vertices, meshTriangles, normals);
            mesh.vertices = vertices;
            mesh.normals = normals;
            mesh.RecalculateBounds();
        }

        private void RebuildMesh(UnifiedSolver solver)
        {
            meshVersion = Tearing.Version;
            int n = Tearing.MeshVertexCount;
            vertices = new Vector3[2 * n];
            normals = new Vector3[2 * n];
            meshUv = new Vector2[2 * n];
            for (int k = 0; k < n; k++)
            {
                vertices[k] = vertices[k + n] = transform.InverseTransformPoint(solver.positions[Tearing.MeshParticle(k)]);
                meshUv[k] = meshUv[k + n] = Tearing.MeshUv(k);
            }
            Tearing.GetMeshTriangles(triangleBuffer);
            int t3 = triangleBuffer.Count;
            meshTriangles = new int[2 * t3];
            for (int t = 0; t < t3; t += 3)
            {
                int a = triangleBuffer[t], b = triangleBuffer[t + 1], c = triangleBuffer[t + 2];
                // Front face, and the back face on the duplicated vertices with the opposite winding.
                meshTriangles[2 * t] = a; meshTriangles[2 * t + 1] = b; meshTriangles[2 * t + 2] = c;
                meshTriangles[2 * t + 3] = a + n; meshTriangles[2 * t + 4] = c + n; meshTriangles[2 * t + 5] = b + n;
            }
            mesh.Clear();
            mesh.vertices = vertices;
            mesh.uv = meshUv;
            mesh.triangles = meshTriangles;
        }
    }
}
