using System;
using UnityEngine;

namespace GamiX.Physics.Geometry
{
    /// <summary>Half-line P(t) = S + t V, t >= 0 (chapter 3, slide 11). Direction is kept unit length.</summary>
    [Serializable]
    public struct Ray3D
    {
        public Vector3 origin;
        public Vector3 direction;

        public Ray3D(Vector3 origin, Vector3 direction)
        {
            this.origin = origin;
            this.direction = VMath.Normalize(direction);
        }

        public Vector3 GetPoint(float t) => origin + direction * t;
    }

    /// <summary>Infinite line P(t) = S + t V, t in R (chapter 3, slide 9).</summary>
    [Serializable]
    public struct Line
    {
        public Vector3 point;
        public Vector3 direction;

        public Line(Vector3 point, Vector3 direction)
        {
            this.point = point;
            this.direction = direction;
        }

        public static Line FromPoints(Vector3 a, Vector3 b) => new Line(a, b - a);

        public Vector3 GetPoint(float t) => point + direction * t;
    }

    /// <summary>Segment P(t) = P1 + t (P2 - P1), t in [0, 1] (chapter 3, slide 10).</summary>
    [Serializable]
    public struct Segment
    {
        public Vector3 a;
        public Vector3 b;

        public Segment(Vector3 a, Vector3 b)
        {
            this.a = a;
            this.b = b;
        }

        public Vector3 GetPoint(float t) => a + (b - a) * t;
        public float Length => VMath.Length(b - a);
    }

    /// <summary>
    /// Plane n.Q + D = 0 stored as L = &lt;n, D&gt; with D = -n.P (chapter 3, slides 22-25). n is unit length.
    /// </summary>
    [Serializable]
    public struct Plane3D
    {
        public Vector3 normal;
        public float d;

        public Plane3D(Vector3 normal, float d)
        {
            float len = VMath.Length(normal);
            this.normal = normal / len;
            this.d = d / len;
        }

        public static Plane3D FromPointNormal(Vector3 point, Vector3 normal)
        {
            Vector3 n = VMath.Normalize(normal);
            return new Plane3D { normal = n, d = -VMath.Dot(n, point) };
        }

        public static Plane3D FromTriangle(Vector3 a, Vector3 b, Vector3 c)
        {
            return FromPointNormal(a, VMath.Cross(b - a, c - a));
        }

        /// <summary>Signed distance n.P + D (positive on the side the normal points to), slide 26.</summary>
        public float SignedDistance(Vector3 p) => VMath.Dot(normal, p) + d;

        public Vector3 ClosestPoint(Vector3 p) => p - normal * SignedDistance(p);

        /// <summary>Any point on the plane (the foot of the origin).</summary>
        public Vector3 PointOnPlane => -normal * d;
    }

    [Serializable]
    public struct Sphere
    {
        public Vector3 center;
        public float radius;

        public Sphere(Vector3 center, float radius)
        {
            this.center = center;
            this.radius = radius;
        }

        public bool Contains(Vector3 p) => VMath.DistanceSq(p, center) <= radius * radius;
        public float Volume => 4f / 3f * Mathf.PI * radius * radius * radius;
    }

    /// <summary>Axis-aligned bounding box: 6 values, min and max of each dimension (chapter 4, slide 36).</summary>
    [Serializable]
    public struct Aabb
    {
        public Vector3 min;
        public Vector3 max;

        public Aabb(Vector3 min, Vector3 max)
        {
            this.min = min;
            this.max = max;
        }

        public static Aabb FromCenterExtents(Vector3 center, Vector3 halfExtents)
        {
            return new Aabb(center - halfExtents, center + halfExtents);
        }

        public static Aabb Empty => new Aabb(
            new Vector3(float.MaxValue, float.MaxValue, float.MaxValue),
            new Vector3(float.MinValue, float.MinValue, float.MinValue));

        public Vector3 Center => (min + max) * 0.5f;
        public Vector3 Extents => (max - min) * 0.5f;
        public Vector3 Size => max - min;

        public void Encapsulate(Vector3 p)
        {
            min = VMath.Min(min, p);
            max = VMath.Max(max, p);
        }

        public void Encapsulate(Aabb other)
        {
            min = VMath.Min(min, other.min);
            max = VMath.Max(max, other.max);
        }

        public Aabb Expanded(float margin)
        {
            Vector3 m = new Vector3(margin, margin, margin);
            return new Aabb(min - m, max + m);
        }

        public bool Contains(Vector3 p)
        {
            return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z;
        }

        /// <summary>Overlap of the projections on the three axes.</summary>
        public bool Overlaps(Aabb o)
        {
            return min.x <= o.max.x && max.x >= o.min.x &&
                   min.y <= o.max.y && max.y >= o.min.y &&
                   min.z <= o.max.z && max.z >= o.min.z;
        }

        public Vector3 ClosestPoint(Vector3 p) => VMath.Clamp(p, min, max);

        public static Aabb FromPoints(Vector3[] points)
        {
            Aabb box = Empty;
            for (int i = 0; i < points.Length; i++)
                box.Encapsulate(points[i]);
            return box;
        }
    }

    /// <summary>
    /// Oriented bounding box: center (3), orientation (4 as quaternion, i.e. 9 as matrix) and half extents (3),
    /// chapter 4 slide 42.
    /// </summary>
    [Serializable]
    public struct Obb
    {
        public Vector3 center;
        public Quat rotation;
        public Vector3 halfExtents;

        public Obb(Vector3 center, Quat rotation, Vector3 halfExtents)
        {
            this.center = center;
            this.rotation = rotation;
            this.halfExtents = halfExtents;
        }

        public Vector3 AxisX => rotation.Rotate(VMath.Right);
        public Vector3 AxisY => rotation.Rotate(VMath.Up);
        public Vector3 AxisZ => rotation.Rotate(VMath.Forward);

        public Vector3 Axis(int i)
        {
            return i == 0 ? AxisX : (i == 1 ? AxisY : AxisZ);
        }

        public float Extent(int i)
        {
            return i == 0 ? halfExtents.x : (i == 1 ? halfExtents.y : halfExtents.z);
        }

        public Vector3 ToLocal(Vector3 worldPoint) => rotation.InverseRotate(worldPoint - center);
        public Vector3 ToWorld(Vector3 localPoint) => center + rotation.Rotate(localPoint);

        public Vector3 ClosestPoint(Vector3 p)
        {
            Vector3 local = VMath.Clamp(ToLocal(p), -halfExtents, halfExtents);
            return ToWorld(local);
        }

        /// <summary>Projection radius on axis L: r = sum e_i |u_i . L| (chapter 4, slide 43).</summary>
        public float ProjectionRadius(Vector3 axis)
        {
            return halfExtents.x * Mathf.Abs(VMath.Dot(AxisX, axis)) +
                   halfExtents.y * Mathf.Abs(VMath.Dot(AxisY, axis)) +
                   halfExtents.z * Mathf.Abs(VMath.Dot(AxisZ, axis));
        }

        public Aabb ComputeAabb()
        {
            Mat3 r = rotation.ToMat3();
            Vector3 e = new Vector3(
                Mathf.Abs(r.m00) * halfExtents.x + Mathf.Abs(r.m01) * halfExtents.y + Mathf.Abs(r.m02) * halfExtents.z,
                Mathf.Abs(r.m10) * halfExtents.x + Mathf.Abs(r.m11) * halfExtents.y + Mathf.Abs(r.m12) * halfExtents.z,
                Mathf.Abs(r.m20) * halfExtents.x + Mathf.Abs(r.m21) * halfExtents.y + Mathf.Abs(r.m22) * halfExtents.z);
            return Aabb.FromCenterExtents(center, e);
        }

        public void GetCorners(Vector3[] corners)
        {
            int k = 0;
            for (int ix = -1; ix <= 1; ix += 2)
            for (int iy = -1; iy <= 1; iy += 2)
            for (int iz = -1; iz <= 1; iz += 2)
                corners[k++] = ToWorld(new Vector3(ix * halfExtents.x, iy * halfExtents.y, iz * halfExtents.z));
        }
    }

    [Serializable]
    public struct Triangle
    {
        public Vector3 a, b, c;

        public Triangle(Vector3 a, Vector3 b, Vector3 c)
        {
            this.a = a; this.b = b; this.c = c;
        }

        /// <summary>Unit normal, right-hand rule (a, b, c).</summary>
        public Vector3 Normal => VMath.TriangleNormal(a, b, c);
        public float Area => 0.5f * VMath.Length(VMath.Cross(b - a, c - a));
        public Plane3D Plane3D => Plane3D.FromTriangle(a, b, c);
    }

    /// <summary>Capsule = segment [a, b] swept by a sphere of the given radius.</summary>
    [Serializable]
    public struct Capsule
    {
        public Vector3 a, b;
        public float radius;

        public Capsule(Vector3 a, Vector3 b, float radius)
        {
            this.a = a; this.b = b; this.radius = radius;
        }
    }

    /// <summary>Finite solid cylinder with axis [a, b].</summary>
    [Serializable]
    public struct Cylinder
    {
        public Vector3 a, b;
        public float radius;

        public Cylinder(Vector3 a, Vector3 b, float radius)
        {
            this.a = a; this.b = b; this.radius = radius;
        }
    }

    /// <summary>Result of a ray query: parameter t, hit point and surface normal.</summary>
    public struct RayHit
    {
        public float t;
        public Vector3 point;
        public Vector3 normal;
        /// <summary>Barycentric coordinates (w0, w1, w2) for triangle hits.</summary>
        public Vector3 barycentric;
        /// <summary>Optional index of the object hit (set by multi-object queries).</summary>
        public int index;
    }
}
