using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Physics.Components
{
    public enum ParticleRigidShape
    {
        Box,
        Sphere,
        /// <summary>Voxelizes the MeshFilter's mesh (must be closed).</summary>
        Mesh
    }

    /// <summary>
    /// Rigid body made of particles and kept rigid by shape matching (paper section 5). The GameObject's transform is
    /// used as the display: after each step it is set to the pose found by shape matching.
    /// </summary>
    public sealed class ParticleRigidBody : ParticleBody
    {
        public ParticleRigidShape shape = ParticleRigidShape.Box;
        [Tooltip("Box: full size. Sphere: x = diameter (1 matches Unity's sphere). Mesh: ignored (uses the transform scale).")]
        public Vector3 size = Vector3.one;
        public float particleMass = 1f;
        public Vector3 initialVelocity;
        [Range(0f, 1f)] public float stiffness = 1f;
        [Tooltip("Plastic deformation (section 5.3): threshold distance and creep in [0, 1]. 0 disables it.")]
        public float plasticThreshold;
        [Range(0f, 1f)] public float plasticCreep;

        public RigidGroup Group { get; private set; }

        protected override void Create(UnifiedSolver solver)
        {
            Quat q = Quat.FromUnity(transform.rotation);
            Vector3 scale = transform.lossyScale;
            switch (shape)
            {
                case ParticleRigidShape.Sphere:
                    Group = ParticleFactory.CreateRigidSphere(solver, transform.position, 0.5f * size.x * scale.x, particleMass, initialVelocity, stiffness);
                    break;
                case ParticleRigidShape.Mesh:
                    MeshFilter mf = GetComponent<MeshFilter>();
                    if (mf == null || mf.sharedMesh == null)
                    {
                        Debug.LogError($"{name}: Mesh shape needs a MeshFilter with a mesh.", this);
                        return;
                    }
                    Mesh mesh = mf.sharedMesh;
                    Group = ParticleFactory.CreateRigidFromMesh(solver, mesh.vertices, mesh.triangles, scale,
                                                                VMath.Zero, q, particleMass, initialVelocity, stiffness);
                    // The mesh was sampled around its own origin: place it at the transform.
                    Vector3 offset = transform.position + q.Rotate(Group.localRestCenter) - Group.center;
                    foreach (int i in Group.indices)
                    {
                        solver.positions[i] += offset;
                        solver.predicted[i] += offset;
                        solver.restPositions[i] += offset;
                    }
                    Group.center += offset;
                    break;
                default:
                    Group = ParticleFactory.CreateRigidBox(solver, transform.position, q, VMath.Mul(size, scale), particleMass, initialVelocity, stiffness);
                    break;
            }
            Group.plasticThreshold = plasticThreshold;
            Group.plasticCreep = plasticCreep;
            Particles.AddRange(Group.indices);
        }

        public override void SyncVisual(UnifiedSolver solver)
        {
            if (Group == null) return;
            // Display pose: x_world = c + Q (x_local - restCenter)  =>  position = c - Q restCenter (scaled space).
            Vector3 restCenter = shape == ParticleRigidShape.Mesh ? Group.localRestCenter : VMath.Zero;
            transform.SetPositionAndRotation(Group.center - Group.rotation.Rotate(restCenter), Group.rotation.ToUnity());
        }
    }
}
