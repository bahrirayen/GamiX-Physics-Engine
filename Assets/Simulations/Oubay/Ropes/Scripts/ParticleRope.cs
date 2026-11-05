using GamiX.Physics;
using GamiX.Physics.Components;
using GamiX.Physics.Rendering;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Ropes
{
    /// <summary>
    /// Rope: chain of particles with stretch and bending constraints (paper section 8), rendered as a tube extruded
    /// along the particles ("we extrude tessellated cylinders along their path").
    /// Each end can follow a transform (kinematic pin) or be attached to a particle rigid body.
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public sealed class ParticleRope : ParticleBody
    {
        public Transform startPoint;
        public Transform endPoint;
        [Tooltip("Used when the corresponding Transform is empty (local to this object).")]
        public Vector3 startLocal = Vector3.zero;
        public Vector3 endLocal = new Vector3(2f, 0f, 0f);
        public float particleMass = 0.05f;
        [Range(0f, 1f)] public float stretchStiffness = 1f;
        [Range(0f, 1f)] public float bendStiffness = 0.2f;
        public bool pinStart = true;
        public bool pinEnd;
        [Tooltip("Optional: attach the start / end to these particle rigid bodies instead of pinning.")]
        public ParticleRigidBody attachStartTo;
        public ParticleRigidBody attachEndTo;
        public float tubeRadius = 0.032f;
        [Range(3, 16)] public int tubeSides = 12;

        public RopeHandle Rope { get; private set; }

        private Mesh mesh;
        private Vector3[] vertices, normals;
        private Vector4[] tangents;
        private int[] triangles;
        private Vector2[] uvs;
        private Material ropeMaterial;

        private const float TextureTwistLength = 0.24f;

        private Vector3 StartWorld => startPoint != null ? startPoint.position : transform.TransformPoint(startLocal);
        private Vector3 EndWorld => endPoint != null ? endPoint.position : transform.TransformPoint(endLocal);

        protected override void Create(UnifiedSolver solver)
        {
            renderParticles = false;
            Rope = RopeBuilder.CreateRope(solver, StartWorld, EndWorld, particleMass, stretchStiffness, bendStiffness,
                                           pinStart && attachStartTo == null, pinEnd && attachEndTo == null);
            Particles.AddRange(Rope.particles);
            int first = Rope.particles[0], last = Rope.particles[Rope.particles.Length - 1];
            // Make sure the rigid bodies exist (build order between components is not guaranteed).
            if (attachStartTo != null) attachStartTo.Build(solver);
            if (attachEndTo != null) attachEndTo.Build(solver);
            if (attachStartTo != null && attachStartTo.Group != null)
                ParticleFactory.Attach(solver, first, attachStartTo.Group);
            if (attachEndTo != null && attachEndTo.Group != null)
                ParticleFactory.Attach(solver, last, attachEndTo.Group);
            BuildMesh(solver);
        }

        public override void PreStep(UnifiedSolver solver, float h)
        {
            if (Rope == null) return;
            int first = Rope.particles[0], last = Rope.particles[Rope.particles.Length - 1];
            if (solver.invMass[first] == 0f) solver.MovePinnedParticle(first, StartWorld);
            if (solver.invMass[last] == 0f) solver.MovePinnedParticle(last, EndWorld);
        }

        private void BuildMesh(UnifiedSolver solver)
        {
            int n = Rope.particles.Length, s = tubeSides;
            int ringSize = s + 1; // Duplicate the seam vertex so the texture wraps cleanly.
            vertices = new Vector3[n * ringSize];
            normals = new Vector3[n * ringSize];
            tangents = new Vector4[n * ringSize];
            uvs = new Vector2[n * ringSize];
            triangles = new int[(n - 1) * s * 6];
            int t = 0;
            for (int k = 0; k + 1 < n; k++)
            for (int j = 0; j < s; j++)
            {
                int a = k * ringSize + j, b = a + 1, c = (k + 1) * ringSize + j, d = c + 1;
                triangles[t++] = a; triangles[t++] = c; triangles[t++] = b;
                triangles[t++] = b; triangles[t++] = c; triangles[t++] = d;
            }
            float distance = 0f;
            for (int k = 0; k < n; k++)
            {
                if (k > 0)
                    distance += VMath.Distance(solver.positions[Rope.particles[k - 1]], solver.positions[Rope.particles[k]]);
                for (int j = 0; j <= s; j++)
                    uvs[k * ringSize + j] = new Vector2(distance / TextureTwistLength, (float)j / s);
            }
            mesh = new Mesh { name = "GamiX Rope" };
            mesh.MarkDynamic();
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uvs;
            GetComponent<MeshFilter>().sharedMesh = mesh;
            MeshRenderer mr = GetComponent<MeshRenderer>();
            ropeMaterial = RenderUtils.CreateLitMaterial(color);
            ropeMaterial.name = "GamiX Manila Rope";
            Texture2D albedo = Resources.Load<Texture2D>("Ropes/ManilaRopeAlbedo");
            if (albedo != null)
            {
                if (ropeMaterial.HasProperty("_BaseMap")) ropeMaterial.SetTexture("_BaseMap", albedo);
                if (ropeMaterial.HasProperty("_MainTex")) ropeMaterial.SetTexture("_MainTex", albedo);
            }
            Texture2D normalMap = Resources.Load<Texture2D>("Ropes/ManilaRopeNormal");
            if (normalMap != null && ropeMaterial.HasProperty("_BumpMap"))
            {
                ropeMaterial.SetTexture("_BumpMap", normalMap);
                ropeMaterial.SetFloat("_BumpScale", 0.35f);
                ropeMaterial.EnableKeyword("_NORMALMAP");
            }
            if (ropeMaterial.HasProperty("_Metallic")) ropeMaterial.SetFloat("_Metallic", 0f);
            if (ropeMaterial.HasProperty("_Smoothness")) ropeMaterial.SetFloat("_Smoothness", 0.22f);
            mr.sharedMaterial = ropeMaterial;
        }

        public override void SyncVisual(UnifiedSolver solver)
        {
            if (Rope == null) return;
            int n = Rope.particles.Length, s = tubeSides, ringSize = tubeSides + 1;
            // Parallel transport frame along the polyline (no twisting).
            Vector3 prevTangent = VMath.Normalize(solver.positions[Rope.particles[1]] - solver.positions[Rope.particles[0]]);
            VMath.OrthonormalBasis(prevTangent, out Vector3 side, out _);
            for (int k = 0; k < n; k++)
            {
                Vector3 p = solver.positions[Rope.particles[k]];
                Vector3 next = solver.positions[Rope.particles[Mathf.Min(k + 1, n - 1)]];
                Vector3 prev = solver.positions[Rope.particles[Mathf.Max(k - 1, 0)]];
                Vector3 tangent = VMath.NormalizeOr(next - prev, prevTangent);
                // Rotate the side vector by the rotation taking prevTangent to tangent.
                Vector3 axis = VMath.Cross(prevTangent, tangent);
                float sinA = VMath.Length(axis);
                if (sinA > 1e-6f)
                {
                    float angle = Mathf.Atan2(sinA, VMath.Dot(prevTangent, tangent));
                    side = Quat.FromAxisAngle(axis / sinA, angle).Rotate(side);
                }
                side = VMath.Normalize(side - tangent * VMath.Dot(side, tangent));
                Vector3 up = VMath.Cross(tangent, side);
                prevTangent = tangent;
                for (int j = 0; j <= s; j++)
                {
                    float a = 2f * Mathf.PI * j / s;
                    Vector3 dir = side * Mathf.Cos(a) + up * Mathf.Sin(a);
                    int index = k * ringSize + j;
                    vertices[index] = transform.InverseTransformPoint(p + dir * tubeRadius);
                    normals[index] = transform.InverseTransformDirection(dir);
                    Vector3 localTangent = VMath.NormalizeOr(transform.InverseTransformDirection(tangent), Vector3.forward);
                    tangents[index] = new Vector4(localTangent.x, localTangent.y, localTangent.z, -1f);
                }
            }
            mesh.vertices = vertices;
            mesh.normals = normals;
            mesh.tangents = tangents;
            mesh.RecalculateBounds();
        }
    }
}
