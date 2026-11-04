using UnityEngine;

namespace GamiX.Physics
{
    /// <summary>
    /// Vector helpers written by hand. UnityEngine.Vector3 is only used as an (x, y, z) container:
    /// dot, cross, length and normalization are implemented here instead of using Unity's built-ins.
    /// </summary>
    public static class VMath
    {
        public const float Epsilon = 1e-6f;

        public static readonly Vector3 Zero = new Vector3(0f, 0f, 0f);
        public static readonly Vector3 One = new Vector3(1f, 1f, 1f);
        public static readonly Vector3 Up = new Vector3(0f, 1f, 0f);
        public static readonly Vector3 Right = new Vector3(1f, 0f, 0f);
        public static readonly Vector3 Forward = new Vector3(0f, 0f, 1f);

        /// <summary>Scalar product a.b (chapter 1, "produit scalaire").</summary>
        public static float Dot(Vector3 a, Vector3 b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        /// <summary>Cross product a x b (chapter 1, "produit vectoriel").</summary>
        public static Vector3 Cross(Vector3 a, Vector3 b)
        {
            return new Vector3(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
        }

        public static float LengthSq(Vector3 v)
        {
            return v.x * v.x + v.y * v.y + v.z * v.z;
        }

        public static float Length(Vector3 v)
        {
            return Mathf.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        }

        public static float Distance(Vector3 a, Vector3 b)
        {
            return Length(a - b);
        }

        public static float DistanceSq(Vector3 a, Vector3 b)
        {
            return LengthSq(a - b);
        }

        /// <summary>Unit vector, or zero when the vector is (almost) null.</summary>
        public static Vector3 Normalize(Vector3 v)
        {
            float len = Length(v);
            return len > Epsilon ? v / len : Zero;
        }

        public static Vector3 Normalize(Vector3 v, out float length)
        {
            length = Length(v);
            return length > Epsilon ? v / length : Zero;
        }

        /// <summary>Unit vector, or <paramref name="fallback"/> when the vector is (almost) null.</summary>
        public static Vector3 NormalizeOr(Vector3 v, Vector3 fallback)
        {
            float len = Length(v);
            return len > Epsilon ? v / len : fallback;
        }

        /// <summary>Projection of v onto the direction "onto" (onto does not need to be unit).</summary>
        public static Vector3 Project(Vector3 v, Vector3 onto)
        {
            float d = Dot(onto, onto);
            return d > Epsilon * Epsilon ? onto * (Dot(v, onto) / d) : Zero;
        }

        /// <summary>Removes the component of v along the unit normal n.</summary>
        public static Vector3 ProjectOnPlane(Vector3 v, Vector3 unitNormal)
        {
            return v - unitNormal * Dot(v, unitNormal);
        }

        /// <summary>Mirror reflection v' = v - 2 (v.n) n (chapter 4, wall collision).</summary>
        public static Vector3 Reflect(Vector3 v, Vector3 unitNormal)
        {
            return v - unitNormal * (2f * Dot(v, unitNormal));
        }

        public static Vector3 Mul(Vector3 a, Vector3 b)
        {
            return new Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
        }

        public static Vector3 Abs(Vector3 v)
        {
            return new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
        }

        public static Vector3 Min(Vector3 a, Vector3 b)
        {
            return new Vector3(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z);
        }

        public static Vector3 Max(Vector3 a, Vector3 b)
        {
            return new Vector3(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z);
        }

        public static float MaxComponent(Vector3 v)
        {
            float m = v.x > v.y ? v.x : v.y;
            return m > v.z ? m : v.z;
        }

        public static float Clamp(float x, float min, float max)
        {
            return x < min ? min : (x > max ? max : x);
        }

        public static float Clamp01(float x)
        {
            return x < 0f ? 0f : (x > 1f ? 1f : x);
        }

        public static Vector3 Clamp(Vector3 v, Vector3 min, Vector3 max)
        {
            return new Vector3(Clamp(v.x, min.x, max.x), Clamp(v.y, min.y, max.y), Clamp(v.z, min.z, max.z));
        }

        public static Vector3 Lerp(Vector3 a, Vector3 b, float t)
        {
            return a + (b - a) * t;
        }

        public static float Sign(float x)
        {
            return x < 0f ? -1f : 1f;
        }

        /// <summary>Builds two unit vectors t1, t2 so that (n, t1, t2) is an orthonormal basis.</summary>
        public static void OrthonormalBasis(Vector3 n, out Vector3 t1, out Vector3 t2)
        {
            // Pick the world axis least aligned with n to avoid a degenerate cross product.
            Vector3 helper = Mathf.Abs(n.x) < 0.57735f ? Right : (Mathf.Abs(n.y) < 0.57735f ? Up : Forward);
            t1 = Normalize(Cross(n, helper));
            t2 = Cross(n, t1);
        }

        /// <summary>Unit normal of triangle (a, b, c) using the right-hand rule (chapter 1, slide 10).</summary>
        public static Vector3 TriangleNormal(Vector3 a, Vector3 b, Vector3 c)
        {
            return Normalize(Cross(b - a, c - a));
        }

        /// <summary>Back-face test of chapter 1: the face is visible when the angle between the normal and the
        /// view vector (vertex to camera) is below 90 degrees, i.e. when their dot product is positive.</summary>
        public static bool IsFaceVisible(Vector3 vertex, Vector3 normal, Vector3 cameraPosition)
        {
            return Dot(cameraPosition - vertex, normal) > 0f;
        }

        public static bool IsFinite(Vector3 v)
        {
            return !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) ||
                     float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));
        }
    }
}
