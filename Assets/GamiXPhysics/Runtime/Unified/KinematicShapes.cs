using GamiX.Physics.Geometry;
using UnityEngine;

namespace GamiX.Physics.Unified
{
    /// <summary>
    /// Large static or animated collision shapes (walls, floors, containers). The paper notes that particles are a
    /// poor primitive for large flat surfaces and uses analytic shapes / SDFs for them (section 9).
    /// Each shape exposes a signed distance (negative inside the solid) and its outward gradient.
    /// </summary>
    public abstract class KinematicShape
    {
        public Vector3 position;
        public Quat rotation = Quat.Identity;

        /// <summary>When true the shape is a container: particles are kept inside it (e.g. a fluid tank).</summary>
        public bool inverted;
        public float friction = -1f; // < 0: use the solver's friction.
        public bool enabled = true;

        internal Vector3 previousPosition;
        internal Quat previousRotation = Quat.Identity;
        internal Vector3 linearVelocity;
        internal Vector3 angularVelocity;

        /// <summary>Signed distance from p to the solid surface in local space (negative inside).</summary>
        protected abstract float LocalDistance(Vector3 localPoint, out Vector3 localNormal);

        /// <summary>Local bounds used for culling (ignored when inverted).</summary>
        protected abstract Aabb LocalBounds { get; }

        public float SignedDistance(Vector3 worldPoint, out Vector3 normal)
        {
            Vector3 local = rotation.InverseRotate(worldPoint - position);
            float d = LocalDistance(local, out Vector3 n);
            if (inverted)
            {
                d = -d;
                n = -n;
            }
            normal = rotation.Rotate(n);
            return d;
        }

        /// <summary>World AABB of the solid (infinite when inverted or unbounded).</summary>
        public Aabb WorldBounds
        {
            get
            {
                if (inverted)
                    return new Aabb(new Vector3(-1e9f, -1e9f, -1e9f), new Vector3(1e9f, 1e9f, 1e9f));
                Aabb lb = LocalBounds;
                return new Obb(position + rotation.Rotate(lb.Center), rotation, lb.Extents).ComputeAabb();
            }
        }

        /// <summary>Moves the shape. The velocity is deduced from the previous pose, for friction with moving shapes.</summary>
        public void SetPose(Vector3 newPosition, Quat newRotation, float dt)
        {
            previousPosition = position;
            previousRotation = rotation;
            position = newPosition;
            rotation = newRotation;
            if (dt > 0f)
            {
                linearVelocity = (position - previousPosition) / dt;
                Quat dq = rotation * previousRotation.Conjugate;
                if (dq.w < 0f) dq = dq * -1f;
                dq.ToAxisAngle(out Vector3 axis, out float angle);
                angularVelocity = axis * (angle / dt);
            }
        }

        /// <summary>Velocity of the shape surface at a world point: v + w x (p - c).</summary>
        public Vector3 PointVelocity(Vector3 worldPoint)
        {
            return linearVelocity + VMath.Cross(angularVelocity, worldPoint - position);
        }
    }

    /// <summary>Half-space below the plane through position with normal rotation * up.</summary>
    public sealed class KinematicPlane : KinematicShape
    {
        public Vector3 localNormal = VMath.Up;

        protected override float LocalDistance(Vector3 p, out Vector3 n)
        {
            n = localNormal;
            return VMath.Dot(p, localNormal);
        }

        protected override Aabb LocalBounds => new Aabb(new Vector3(-1e9f, -1e9f, -1e9f), new Vector3(1e9f, 1e9f, 1e9f));
    }

    public sealed class KinematicSphere : KinematicShape
    {
        public float radius = 0.5f;

        protected override float LocalDistance(Vector3 p, out Vector3 n)
        {
            float len = VMath.Length(p);
            n = len > 1e-9f ? p / len : VMath.Up;
            return len - radius;
        }

        protected override Aabb LocalBounds => Aabb.FromCenterExtents(VMath.Zero, new Vector3(radius, radius, radius));
    }

    public sealed class KinematicBox : KinematicShape
    {
        public Vector3 halfExtents = new Vector3(0.5f, 0.5f, 0.5f);

        /// <summary>Exact signed distance of a box (outside: distance to the box; inside: minus distance to the closest face).</summary>
        protected override float LocalDistance(Vector3 p, out Vector3 n)
        {
            Vector3 q = VMath.Abs(p) - halfExtents;
            if (q.x > 0f || q.y > 0f || q.z > 0f)
            {
                Vector3 outside = VMath.Max(q, VMath.Zero);
                float d = VMath.Length(outside);
                Vector3 dir = new Vector3(
                    outside.x * VMath.Sign(p.x), outside.y * VMath.Sign(p.y), outside.z * VMath.Sign(p.z));
                n = d > 1e-9f ? dir / d : VMath.Up;
                return d;
            }
            // Inside: closest face.
            if (q.x >= q.y && q.x >= q.z)
            {
                n = new Vector3(VMath.Sign(p.x), 0f, 0f);
                return q.x;
            }
            if (q.y >= q.z)
            {
                n = new Vector3(0f, VMath.Sign(p.y), 0f);
                return q.y;
            }
            n = new Vector3(0f, 0f, VMath.Sign(p.z));
            return q.z;
        }

        protected override Aabb LocalBounds => Aabb.FromCenterExtents(VMath.Zero, halfExtents);
    }

    /// <summary>Capsule along the local Y axis: segment [-halfHeight, +halfHeight] swept by radius.</summary>
    public sealed class KinematicCapsule : KinematicShape
    {
        public float radius = 0.25f;
        public float halfHeight = 0.5f;

        protected override float LocalDistance(Vector3 p, out Vector3 n)
        {
            float y = VMath.Clamp(p.y, -halfHeight, halfHeight);
            Vector3 d = p - new Vector3(0f, y, 0f);
            float len = VMath.Length(d);
            n = len > 1e-9f ? d / len : VMath.Right;
            return len - radius;
        }

        protected override Aabb LocalBounds => Aabb.FromCenterExtents(VMath.Zero, new Vector3(radius, halfHeight + radius, radius));
    }
}
