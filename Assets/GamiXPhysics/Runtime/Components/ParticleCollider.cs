using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Physics.Components
{
    public enum ColliderShape
    {
        Plane,
        Box,
        Sphere,
        Capsule
    }

    /// <summary>
    /// Static or animated collision shape for particles (floors, walls, containers, obstacles). The pose is read from
    /// the transform every simulation frame, so animating the transform moves the shape (with friction).
    /// Sizes are given in local units and multiplied by the transform scale.
    /// </summary>
    public sealed class ParticleCollider : MonoBehaviour
    {
        public ColliderShape shape = ColliderShape.Box;
        [Tooltip("Box: full size. Sphere: x = radius. Capsule: x = radius, y = total height.")]
        public Vector3 size = Vector3.one;
        [Tooltip("Container: particles are kept inside the shape (e.g. a fluid tank).")]
        public bool inverted;
        [Tooltip("Friction multiplier (negative = use the world's friction).")]
        public float friction = -1f;
        public bool drawGizmo = true;

        public KinematicShape Shape { get; private set; }
        private UnifiedWorld world;

        private void OnEnable()
        {
            world = UnifiedWorld.Find(this);
            if (world != null)
                world.Register(this);
            if (Shape != null)
                Shape.enabled = true;
        }

        private void OnDisable()
        {
            if (Shape != null)
                Shape.enabled = false;
        }

        internal void Build(UnifiedSolver solver)
        {
            if (Shape != null) return;
            Vector3 s = VMath.Mul(size, transform.lossyScale);
            switch (shape)
            {
                case ColliderShape.Plane:
                    Shape = new KinematicPlane();
                    break;
                case ColliderShape.Sphere:
                    Shape = new KinematicSphere { radius = s.x };
                    break;
                case ColliderShape.Capsule:
                    Shape = new KinematicCapsule { radius = s.x, halfHeight = Mathf.Max(0f, 0.5f * s.y - s.x) };
                    break;
                default:
                    Shape = new KinematicBox { halfExtents = s * 0.5f };
                    break;
            }
            Shape.inverted = inverted;
            Shape.friction = friction;
            Shape.position = transform.position;
            Shape.rotation = Quat.FromUnity(transform.rotation);
            solver.AddShape(Shape);
        }

        internal void SyncPose(float h)
        {
            if (Shape == null) return;
            Shape.SetPose(transform.position, Quat.FromUnity(transform.rotation), h);
        }

        private void OnDrawGizmos()
        {
            if (!drawGizmo) return;
            Gizmos.color = inverted ? new Color(0.2f, 0.8f, 1f, 0.6f) : new Color(1f, 0.6f, 0.2f, 0.6f);
            Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
            Vector3 s = VMath.Mul(size, transform.lossyScale);
            switch (shape)
            {
                case ColliderShape.Plane:
                    Gizmos.DrawWireCube(Vector3.zero, new Vector3(10f, 0f, 10f));
                    break;
                case ColliderShape.Sphere:
                    Gizmos.DrawWireSphere(Vector3.zero, s.x);
                    break;
                case ColliderShape.Capsule:
                    float hh = Mathf.Max(0f, 0.5f * s.y - s.x);
                    Gizmos.DrawWireSphere(new Vector3(0f, hh, 0f), s.x);
                    Gizmos.DrawWireSphere(new Vector3(0f, -hh, 0f), s.x);
                    break;
                default:
                    Gizmos.DrawWireCube(Vector3.zero, s);
                    break;
            }
        }
    }
}
