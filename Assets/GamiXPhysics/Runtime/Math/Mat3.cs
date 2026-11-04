using System;
using UnityEngine;

namespace GamiX.Physics
{
    /// <summary>
    /// 3x3 matrix (row-major: mRC = row R, column C). Used for rotation matrices, inertia tensors
    /// (chapter 2 part 2) and the covariance matrix of rigid shape matching (FleX paper, eq. 16).
    /// </summary>
    [Serializable]
    public struct Mat3
    {
        public float m00, m01, m02;
        public float m10, m11, m12;
        public float m20, m21, m22;

        public Mat3(float m00, float m01, float m02,
                    float m10, float m11, float m12,
                    float m20, float m21, float m22)
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
        }

        public static Mat3 Identity => new Mat3(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f);
        public static Mat3 Zero => new Mat3(0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f);

        public static Mat3 Diagonal(float a, float b, float c)
        {
            return new Mat3(a, 0f, 0f, 0f, b, 0f, 0f, 0f, c);
        }

        public static Mat3 Diagonal(Vector3 d)
        {
            return Diagonal(d.x, d.y, d.z);
        }

        public static Mat3 FromColumns(Vector3 c0, Vector3 c1, Vector3 c2)
        {
            return new Mat3(c0.x, c1.x, c2.x,
                            c0.y, c1.y, c2.y,
                            c0.z, c1.z, c2.z);
        }

        public static Mat3 FromRows(Vector3 r0, Vector3 r1, Vector3 r2)
        {
            return new Mat3(r0.x, r0.y, r0.z,
                            r1.x, r1.y, r1.z,
                            r2.x, r2.y, r2.z);
        }

        public Vector3 GetColumn(int i)
        {
            switch (i)
            {
                case 0: return new Vector3(m00, m10, m20);
                case 1: return new Vector3(m01, m11, m21);
                case 2: return new Vector3(m02, m12, m22);
                default: throw new ArgumentOutOfRangeException(nameof(i));
            }
        }

        public Vector3 GetRow(int i)
        {
            switch (i)
            {
                case 0: return new Vector3(m00, m01, m02);
                case 1: return new Vector3(m10, m11, m12);
                case 2: return new Vector3(m20, m21, m22);
                default: throw new ArgumentOutOfRangeException(nameof(i));
            }
        }

        public float this[int row, int col]
        {
            get
            {
                switch (row * 3 + col)
                {
                    case 0: return m00; case 1: return m01; case 2: return m02;
                    case 3: return m10; case 4: return m11; case 5: return m12;
                    case 6: return m20; case 7: return m21; case 8: return m22;
                    default: throw new ArgumentOutOfRangeException();
                }
            }
            set
            {
                switch (row * 3 + col)
                {
                    case 0: m00 = value; break; case 1: m01 = value; break; case 2: m02 = value; break;
                    case 3: m10 = value; break; case 4: m11 = value; break; case 5: m12 = value; break;
                    case 6: m20 = value; break; case 7: m21 = value; break; case 8: m22 = value; break;
                    default: throw new ArgumentOutOfRangeException();
                }
            }
        }

        public static Mat3 operator *(Mat3 a, Mat3 b)
        {
            return new Mat3(
                a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20,
                a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21,
                a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22,

                a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20,
                a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21,
                a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22,

                a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20,
                a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21,
                a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22);
        }

        public static Vector3 operator *(Mat3 m, Vector3 v)
        {
            return new Vector3(
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z);
        }

        public static Mat3 operator *(Mat3 m, float s)
        {
            return new Mat3(m.m00 * s, m.m01 * s, m.m02 * s,
                            m.m10 * s, m.m11 * s, m.m12 * s,
                            m.m20 * s, m.m21 * s, m.m22 * s);
        }

        public static Mat3 operator *(float s, Mat3 m)
        {
            return m * s;
        }

        public static Mat3 operator +(Mat3 a, Mat3 b)
        {
            return new Mat3(a.m00 + b.m00, a.m01 + b.m01, a.m02 + b.m02,
                            a.m10 + b.m10, a.m11 + b.m11, a.m12 + b.m12,
                            a.m20 + b.m20, a.m21 + b.m21, a.m22 + b.m22);
        }

        public static Mat3 operator -(Mat3 a, Mat3 b)
        {
            return new Mat3(a.m00 - b.m00, a.m01 - b.m01, a.m02 - b.m02,
                            a.m10 - b.m10, a.m11 - b.m11, a.m12 - b.m12,
                            a.m20 - b.m20, a.m21 - b.m21, a.m22 - b.m22);
        }

        public Mat3 Transposed => new Mat3(m00, m10, m20, m01, m11, m21, m02, m12, m22);

        /// <summary>M^T v without building the transpose.</summary>
        public Vector3 TransposeMultiply(Vector3 v)
        {
            return new Vector3(
                m00 * v.x + m10 * v.y + m20 * v.z,
                m01 * v.x + m11 * v.y + m21 * v.z,
                m02 * v.x + m12 * v.y + m22 * v.z);
        }

        public float Trace => m00 + m11 + m22;

        public float Determinant =>
            m00 * (m11 * m22 - m12 * m21) -
            m01 * (m10 * m22 - m12 * m20) +
            m02 * (m10 * m21 - m11 * m20);

        /// <summary>Inverse by the adjugate (cofactor) matrix. Returns false when the matrix is singular.</summary>
        public bool TryInverse(out Mat3 inverse)
        {
            float det = Determinant;
            if (Mathf.Abs(det) < 1e-12f)
            {
                inverse = Zero;
                return false;
            }
            float inv = 1f / det;
            inverse = new Mat3(
                (m11 * m22 - m12 * m21) * inv,
                (m02 * m21 - m01 * m22) * inv,
                (m01 * m12 - m02 * m11) * inv,

                (m12 * m20 - m10 * m22) * inv,
                (m00 * m22 - m02 * m20) * inv,
                (m02 * m10 - m00 * m12) * inv,

                (m10 * m21 - m11 * m20) * inv,
                (m01 * m20 - m00 * m21) * inv,
                (m00 * m11 - m01 * m10) * inv);
            return true;
        }

        /// <summary>Inverse, or the zero matrix when singular (useful for infinite inertia).</summary>
        public Mat3 Inverse
        {
            get
            {
                TryInverse(out Mat3 inv);
                return inv;
            }
        }

        /// <summary>Solves M x = b (chapter 3: intersection of 3 planes, line-line distance).</summary>
        public bool Solve(Vector3 b, out Vector3 x)
        {
            if (TryInverse(out Mat3 inv))
            {
                x = inv * b;
                return true;
            }
            x = VMath.Zero;
            return false;
        }

        /// <summary>
        /// "Star" (skew-symmetric) matrix of w, so that Skew(w) * v = w x v.
        /// Chapter 2 part 2, slide 41: dR/dt = w* R.
        /// </summary>
        public static Mat3 Skew(Vector3 w)
        {
            return new Mat3(0f, -w.z, w.y,
                            w.z, 0f, -w.x,
                            -w.y, w.x, 0f);
        }

        /// <summary>Outer product a b^T.</summary>
        public static Mat3 Outer(Vector3 a, Vector3 b)
        {
            return new Mat3(a.x * b.x, a.x * b.y, a.x * b.z,
                            a.y * b.x, a.y * b.y, a.y * b.z,
                            a.z * b.x, a.z * b.y, a.z * b.z);
        }

        /// <summary>Adds s * a b^T to this matrix (accumulation without temporaries).</summary>
        public void AddOuter(Vector3 a, Vector3 b, float s)
        {
            m00 += s * a.x * b.x; m01 += s * a.x * b.y; m02 += s * a.x * b.z;
            m10 += s * a.y * b.x; m11 += s * a.y * b.y; m12 += s * a.y * b.z;
            m20 += s * a.z * b.x; m21 += s * a.z * b.y; m22 += s * a.z * b.z;
        }

        /// <summary>R M R^T: expresses a body-space tensor in world space (I(t) = R Ibody R^T).</summary>
        public static Mat3 Similarity(Mat3 r, Mat3 m)
        {
            return r * m * r.Transposed;
        }

        /// <summary>
        /// Gram-Schmidt re-orthonormalization of the columns of a rotation matrix
        /// (chapter 2 part 2, slide 36): T' = T/|T|, N' = N - T'(N.T') normalized, B' = T' x N'.
        /// </summary>
        public static Mat3 Orthonormalize(Mat3 r)
        {
            Vector3 t = VMath.Normalize(r.GetColumn(0));
            Vector3 n = r.GetColumn(1);
            n = VMath.Normalize(n - t * VMath.Dot(n, t));
            Vector3 b = VMath.Cross(t, n);
            return FromColumns(t, n, b);
        }

        /// <summary>
        /// Extracts the rotation part of A (polar decomposition A = Q S) with the iterative method of
        /// Mueller et al. 2016 ("A Robust Method to Extract the Rotational Part of Deformations").
        /// <paramref name="q"/> is used as warm start and receives the result. Used by rigid shape matching (eq. 15-16).
        /// </summary>
        public static void ExtractRotation(Mat3 a, ref Quat q, int maxIterations = 10)
        {
            Vector3 a0 = a.GetColumn(0), a1 = a.GetColumn(1), a2 = a.GetColumn(2);
            for (int iter = 0; iter < maxIterations; iter++)
            {
                Mat3 r = q.ToMat3();
                Vector3 r0 = r.GetColumn(0), r1 = r.GetColumn(1), r2 = r.GetColumn(2);
                Vector3 omega = VMath.Cross(r0, a0) + VMath.Cross(r1, a1) + VMath.Cross(r2, a2);
                float denom = Mathf.Abs(VMath.Dot(r0, a0) + VMath.Dot(r1, a1) + VMath.Dot(r2, a2)) + 1e-9f;
                omega /= denom;
                float w = VMath.Length(omega);
                if (w < 1e-9f)
                    break;
                q = (Quat.FromAxisAngle(omega / w, w) * q).Normalized;
            }
        }

        public override string ToString()
        {
            return $"[{m00:F4} {m01:F4} {m02:F4}; {m10:F4} {m11:F4} {m12:F4}; {m20:F4} {m21:F4} {m22:F4}]";
        }
    }
}
