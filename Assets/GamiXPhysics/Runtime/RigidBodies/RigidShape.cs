using GamiX.Physics.Geometry;
using UnityEngine;

namespace GamiX.Physics.RigidBodies
{
    public enum RigidShapeType
    {
        Sphere,
        Box,
        Plane
    }

    /// <summary>Collision geometry attached to a rigid body. Knows its volume and its inertia tensor.</summary>
    public abstract class RigidShape
    {
        public abstract RigidShapeType Type { get; }

        /// <summary>Volume, used to derive the mass from a density.</summary>
        public abstract float Volume { get; }

        /// <summary>Body-space inertia tensor at the center of mass for the given mass (slide 33).</summary>
        public abstract Mat3 ComputeInertia(float mass);

        /// <summary>World AABB for the broad phase.</summary>
        public abstract Aabb ComputeAabb(Vector3 position, Quat rotation);
    }

    public sealed class SphereShape : RigidShape
    {
        public float radius;
        public bool hollow;

        public SphereShape(float radius, bool hollow = false)
        {
            this.radius = radius;
            this.hollow = hollow;
        }

        public override RigidShapeType Type => RigidShapeType.Sphere;
        public override float Volume => 4f / 3f * Mathf.PI * radius * radius * radius;

        public override Mat3 ComputeInertia(float mass)
        {
            return hollow ? InertiaTensor.HollowSphere(mass, radius) : InertiaTensor.SolidSphere(mass, radius);
        }

        public override Aabb ComputeAabb(Vector3 position, Quat rotation)
        {
            return Aabb.FromCenterExtents(position, new Vector3(radius, radius, radius));
        }
    }

    public sealed class BoxShape : RigidShape
    {
        public Vector3 halfExtents;

        public BoxShape(Vector3 halfExtents)
        {
            this.halfExtents = halfExtents;
        }

        public override RigidShapeType Type => RigidShapeType.Box;
        public override float Volume => 8f * halfExtents.x * halfExtents.y * halfExtents.z;

        public override Mat3 ComputeInertia(float mass)
        {
            return InertiaTensor.Box(mass, halfExtents * 2f);
        }

        public override Aabb ComputeAabb(Vector3 position, Quat rotation)
        {
            return new Obb(position, rotation, halfExtents).ComputeAabb();
        }

        public Obb ToObb(Vector3 position, Quat rotation) => new Obb(position, rotation, halfExtents);
    }

    /// <summary>Infinite plane through the body position, normal = rotation * localNormal. Static bodies only.</summary>
    public sealed class PlaneShape : RigidShape
    {
        public Vector3 localNormal;

        public PlaneShape(Vector3 localNormal)
        {
            this.localNormal = VMath.Normalize(localNormal);
        }

        public PlaneShape() : this(VMath.Up)
        {
        }

        public override RigidShapeType Type => RigidShapeType.Plane;
        public override float Volume => 0f;
        public override Mat3 ComputeInertia(float mass) => Mat3.Zero;

        public override Aabb ComputeAabb(Vector3 position, Quat rotation)
        {
            const float big = 1e6f;
            return new Aabb(new Vector3(-big, -big, -big), new Vector3(big, big, big));
        }

        public Plane3D ToPlane(Vector3 position, Quat rotation)
        {
            return Plane3D.FromPointNormal(position, rotation.Rotate(localNormal));
        }
    }
}
