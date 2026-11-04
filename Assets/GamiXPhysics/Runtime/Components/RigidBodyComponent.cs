using GamiX.Physics.RigidBodies;
using UnityEngine;

namespace GamiX.Physics.Components
{
    public enum RigidBodyMode
    {
        Dynamic,
        Static,
        /// <summary>Moved by the transform (animation/script), pushes dynamic bodies, infinite mass.</summary>
        Kinematic
    }

    public enum DisplayMode
    {
        /// <summary>Copy the simulated pose into the Transform (display only).</summary>
        Transform,
        /// <summary>
        /// Apply our own 4x4 matrix to every mesh vertex (chapter 1, "une meme transformation a chacun des vertex";
        /// chapter 2 slide 12, CustomCube.ApplyTransformation). The Transform stays at the origin.
        /// </summary>
        ManualVertices
    }

    /// <summary>
    /// Rigid body of the <see cref="RigidWorld"/>. The inertia tensor comes from the shape geometry and the mass
    /// (chapter 2 part 2, slide 33). Size is in local units multiplied by the transform scale, so a default Unity cube
    /// (size 1) or sphere (diameter 1) matches its mesh.
    /// </summary>
    public sealed class RigidBodyComponent : MonoBehaviour
    {
        public RigidShapeType shape = RigidShapeType.Box;
        public RigidBodyMode mode = RigidBodyMode.Dynamic;
        [Tooltip("Box: full size. Sphere: x = diameter. Plane: normal = transform up.")]
        public Vector3 size = Vector3.one;
        [Tooltip("Mass in kg. If density > 0, the mass is density x volume instead.")]
        public float mass = 1f;
        public float density;
        [Range(0f, 1f)] public float restitution = 0.3f;
        [Range(0f, 2f)] public float friction = 0.5f;
        public float linearDamping = 0.01f;
        public float angularDamping = 0.02f;
        public bool useGravity = true;
        public Vector3 initialVelocity;
        [Tooltip("Initial angular velocity in rad/s (world space).")]
        public Vector3 initialAngularVelocity;
        public DisplayMode display = DisplayMode.Transform;

        public RigidBody Body { get; private set; }
        public RigidWorld World { get; private set; }

        private Mesh displayMesh;
        private Vector3[] localVertices, worldVertices, localNormals, worldNormals;
        private Vector3 displayScale;

        private void Start()
        {
            World = RigidWorld.Find(this);
            if (World == null)
            {
                Debug.LogError($"{name}: no RigidWorld in the scene.", this);
                return;
            }
            Vector3 s = VMath.Mul(size, transform.lossyScale);
            RigidShape rs;
            switch (shape)
            {
                case RigidShapeType.Sphere: rs = new SphereShape(0.5f * s.x); break;
                case RigidShapeType.Plane: rs = new PlaneShape(VMath.Up); break;
                default: rs = new BoxShape(s * 0.5f); break;
            }
            float m = density > 0f ? density * rs.Volume : mass;
            Quat q = Quat.FromUnity(transform.rotation);
            Body = mode == RigidBodyMode.Dynamic && shape != RigidShapeType.Plane
                ? new RigidBody(rs, m, transform.position, q)
                : RigidBody.CreateStatic(rs, transform.position, q);
            Body.isKinematic = mode == RigidBodyMode.Kinematic;
            Body.restitution = restitution;
            Body.friction = friction;
            Body.linearDamping = linearDamping;
            Body.angularDamping = angularDamping;
            Body.useGravity = useGravity;
            Body.userData = this;
            if (Body.IsDynamic)
            {
                Body.Velocity = initialVelocity;
                Body.AngularVelocity = initialAngularVelocity;
            }
            World.World.AddBody(Body);
            World.Register(this);

            if (display == DisplayMode.ManualVertices && Body.IsDynamic)
                SetupManualVertices();
        }

        private void OnDestroy()
        {
            if (World != null && Body != null)
            {
                World.World.RemoveBody(Body);
                World.Unregister(this);
            }
        }

        private void SetupManualVertices()
        {
            MeshFilter mf = GetComponent<MeshFilter>();
            if (mf == null || mf.sharedMesh == null) return;
            displayMesh = Instantiate(mf.sharedMesh);
            displayMesh.MarkDynamic();
            localVertices = displayMesh.vertices;
            localNormals = displayMesh.normals;
            worldVertices = new Vector3[localVertices.Length];
            worldNormals = new Vector3[localNormals.Length];
            displayScale = transform.lossyScale;
            mf.sharedMesh = displayMesh;
            transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            transform.localScale = Vector3.one;
        }

        /// <summary>Kinematic bodies follow the transform; dynamic bodies drive it.</summary>
        internal void SyncVisual()
        {
            if (Body == null) return;
            if (Body.isKinematic)
            {
                float h = Time.deltaTime > 0f ? Time.deltaTime : 1f / 60f;
                Body.kinematicVelocity = (transform.position - Body.position) / h;
                Body.position = transform.position;
                Body.orientation = Quat.FromUnity(transform.rotation);
                Body.UpdateDerived();
                return;
            }
            if (!Body.IsDynamic) return;

            if (displayMesh != null)
            {
                // Same transformation applied to every vertex: v' = T R S v (homogeneous coordinates).
                Mat4 m = Mat4.TRS(Body.position, Body.orientation, displayScale);
                for (int i = 0; i < localVertices.Length; i++)
                    worldVertices[i] = m.MultiplyPoint(localVertices[i]);
                for (int i = 0; i < localNormals.Length; i++)
                    worldNormals[i] = Body.orientation.Rotate(localNormals[i]);
                displayMesh.vertices = worldVertices;
                displayMesh.normals = worldNormals;
                displayMesh.RecalculateBounds();
            }
            else
            {
                transform.SetPositionAndRotation(Body.position, Body.orientation.ToUnity());
            }
        }

        // ---- convenience API for gameplay scripts ----
        public void AddForce(Vector3 f) => Body?.AddForce(f);
        public void AddForceAtPosition(Vector3 f, Vector3 worldPoint) => Body?.AddForceAtPoint(f, worldPoint);
        public void AddTorque(Vector3 t) => Body?.AddTorque(t);
        public void ApplyImpulse(Vector3 j, Vector3 worldPoint) { Body?.Wake(); Body?.ApplyImpulseAtPoint(j, worldPoint); }
    }
}
