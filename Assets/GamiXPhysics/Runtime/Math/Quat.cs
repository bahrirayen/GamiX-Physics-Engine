using System;
using UnityEngine;

namespace GamiX.Physics
{
    /// <summary>
    /// Unit quaternion q = (w, x, y, z) = (a, b, c, d) in the notation of chapter 1 (slides 53-61).
    /// Written from scratch: Unity's Quaternion is only produced at the very end, to hand a rotation to the renderer.
    /// </summary>
    [Serializable]
    public struct Quat
    {
        public float w, x, y, z;

        public Quat(float w, float x, float y, float z)
        {
            this.w = w; this.x = x; this.y = y; this.z = z;
        }

        public static Quat Identity => new Quat(1f, 0f, 0f, 0f);

        public Vector3 Vector => new Vector3(x, y, z);

        /// <summary>q = (cos(theta/2), v sin(theta/2)) with v unit (slide 55). Angle in radians.</summary>
        public static Quat FromAxisAngle(Vector3 axis, float angleRad)
        {
            Vector3 v = VMath.Normalize(axis);
            float half = 0.5f * angleRad;
            float s = Mathf.Sin(half);
            return new Quat(Mathf.Cos(half), v.x * s, v.y * s, v.z * s);
        }

        public static Quat FromAxisAngleDeg(Vector3 axis, float angleDeg)
        {
            return FromAxisAngle(axis, angleDeg * Mathf.Deg2Rad);
        }

        /// <summary>Exponential map: rotation of |r| radians around r/|r|.</summary>
        public static Quat FromRotationVector(Vector3 r)
        {
            float angle = VMath.Length(r);
            if (angle < 1e-9f)
                return new Quat(1f, 0.5f * r.x, 0.5f * r.y, 0.5f * r.z).Normalized;
            return FromAxisAngle(r / angle, angle);
        }

        /// <summary>Reverse rules of slide 55: theta = 2 acos(a), v = (b, c, d) / sqrt(1 - a^2).</summary>
        public void ToAxisAngle(out Vector3 axis, out float angleRad)
        {
            Quat q = Normalized;
            float a = VMath.Clamp(q.w, -1f, 1f);
            angleRad = 2f * Mathf.Acos(a);
            float s = Mathf.Sqrt(1f - a * a);
            axis = s < 1e-6f ? VMath.Right : new Vector3(q.x / s, q.y / s, q.z / s);
        }

        /// <summary>Hamilton product (slide 53, 4x4 matrix form).</summary>
        public static Quat operator *(Quat q1, Quat q2)
        {
            return new Quat(
                q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
                q1.x * q2.w + q1.w * q2.x - q1.z * q2.y + q1.y * q2.z,
                q1.y * q2.w + q1.z * q2.x + q1.w * q2.y - q1.x * q2.z,
                q1.z * q2.w - q1.y * q2.x + q1.x * q2.y + q1.w * q2.z);
        }

        public static Quat operator *(Quat q, float s)
        {
            return new Quat(q.w * s, q.x * s, q.y * s, q.z * s);
        }

        public static Quat operator +(Quat a, Quat b)
        {
            return new Quat(a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z);
        }

        public float NormSq => w * w + x * x + y * y + z * z;

        /// <summary>||q|| = sqrt(a^2 + b^2 + c^2 + d^2) (slide 53).</summary>
        public float Norm => Mathf.Sqrt(NormSq);

        public Quat Normalized
        {
            get
            {
                float n = Norm;
                return n > 1e-12f ? new Quat(w / n, x / n, y / n, z / n) : Identity;
            }
        }

        /// <summary>Conjugate (a, -b, -c, -d): the inverse of a unit quaternion (slide 57).</summary>
        public Quat Conjugate => new Quat(w, -x, -y, -z);

        public Quat Inverse
        {
            get
            {
                float n2 = NormSq;
                return n2 > 1e-12f ? new Quat(w / n2, -x / n2, -y / n2, -z / n2) : Identity;
            }
        }

        public static float Dot(Quat a, Quat b)
        {
            return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        }

        /// <summary>
        /// Rotates a vector by this unit quaternion. Algebraically equal to q (0, v) q^-1 (slide 58),
        /// expanded to avoid the two full Hamilton products.
        /// </summary>
        public Vector3 Rotate(Vector3 v)
        {
            Vector3 u = new Vector3(x, y, z);
            Vector3 t = VMath.Cross(u, v) * 2f;
            return v + t * w + VMath.Cross(u, t);
        }

        /// <summary>Literal implementation of slide 58: P' = q . P . q^-1 with P = (0, x, y, z).</summary>
        public Vector3 RotateHamilton(Vector3 v)
        {
            Quat p = new Quat(0f, v.x, v.y, v.z);
            Quat r = this * p * Conjugate;
            return new Vector3(r.x, r.y, r.z);
        }

        /// <summary>Rotates by the inverse rotation (world to local for a unit quaternion).</summary>
        public Vector3 InverseRotate(Vector3 v)
        {
            return Conjugate.Rotate(v);
        }

        /// <summary>Equivalent rotation matrix (slide 59).</summary>
        public Mat3 ToMat3()
        {
            float a = w, b = x, c = y, d = z;
            return new Mat3(
                2f * (a * a + b * b) - 1f, 2f * (b * c - a * d), 2f * (b * d + a * c),
                2f * (b * c + a * d), 2f * (a * a + c * c) - 1f, 2f * (c * d - a * b),
                2f * (b * d - a * c), 2f * (c * d + a * b), 2f * (a * a + d * d) - 1f);
        }

        /// <summary>Rotation matrix to quaternion (Shepperd's method, numerically stable).</summary>
        public static Quat FromMat3(Mat3 m)
        {
            float trace = m.m00 + m.m11 + m.m22;
            Quat q;
            if (trace > 0f)
            {
                float s = Mathf.Sqrt(trace + 1f) * 2f;
                q = new Quat(0.25f * s, (m.m21 - m.m12) / s, (m.m02 - m.m20) / s, (m.m10 - m.m01) / s);
            }
            else if (m.m00 > m.m11 && m.m00 > m.m22)
            {
                float s = Mathf.Sqrt(1f + m.m00 - m.m11 - m.m22) * 2f;
                q = new Quat((m.m21 - m.m12) / s, 0.25f * s, (m.m01 + m.m10) / s, (m.m02 + m.m20) / s);
            }
            else if (m.m11 > m.m22)
            {
                float s = Mathf.Sqrt(1f + m.m11 - m.m00 - m.m22) * 2f;
                q = new Quat((m.m02 - m.m20) / s, (m.m01 + m.m10) / s, 0.25f * s, (m.m12 + m.m21) / s);
            }
            else
            {
                float s = Mathf.Sqrt(1f + m.m22 - m.m00 - m.m11) * 2f;
                q = new Quat((m.m10 - m.m01) / s, (m.m02 + m.m20) / s, (m.m12 + m.m21) / s, 0.25f * s);
            }
            return q.Normalized;
        }

        /// <summary>
        /// Spherical linear interpolation (slide 61): q(t) = sin((1-t)theta)/sin(theta) q1 + sin(t theta)/sin(theta) q2,
        /// with cos(theta) = q1.q2. Takes the shortest path and falls back to linear interpolation for tiny angles.
        /// </summary>
        public static Quat Slerp(Quat q1, Quat q2, float t)
        {
            float dot = Dot(q1, q2);
            if (dot < 0f)
            {
                q2 = q2 * -1f;
                dot = -dot;
            }
            float k1, k2;
            if (dot > 0.9995f)
            {
                k1 = 1f - t;
                k2 = t;
            }
            else
            {
                float theta = Mathf.Acos(dot);
                float sinTheta = Mathf.Sin(theta);
                k1 = Mathf.Sin(theta * (1f - t)) / sinTheta;
                k2 = Mathf.Sin(theta * t) / sinTheta;
            }
            return (q1 * k1 + q2 * k2).Normalized;
        }

        /// <summary>First order integration of dq/dt = 1/2 (0, w) q, then renormalization.</summary>
        public static Quat Integrate(Quat q, Vector3 omega, float dt)
        {
            Quat spin = new Quat(0f, omega.x, omega.y, omega.z) * q;
            return (q + spin * (0.5f * dt)).Normalized;
        }

        /// <summary>Exact integration for a constant angular velocity over dt (exponential map).</summary>
        public static Quat IntegrateExact(Quat q, Vector3 omega, float dt)
        {
            return (FromRotationVector(omega * dt) * q).Normalized;
        }

        /// <summary>Time derivative dq/dt = 1/2 (0, w) q, used inside ODE solvers.</summary>
        public static Quat Derivative(Quat q, Vector3 omega)
        {
            return new Quat(0f, omega.x, omega.y, omega.z) * q * 0.5f;
        }

        /// <summary>Converts to Unity's quaternion for rendering only (Unity stores w last).</summary>
        public Quaternion ToUnity()
        {
            return new Quaternion(x, y, z, w);
        }

        public static Quat FromUnity(Quaternion q)
        {
            return new Quat(q.w, q.x, q.y, q.z);
        }

        public override string ToString()
        {
            return $"({w:F4}, {x:F4}, {y:F4}, {z:F4})";
        }
    }
}
