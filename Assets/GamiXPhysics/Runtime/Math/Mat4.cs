using System;
using UnityEngine;

namespace GamiX.Physics
{
    /// <summary>
    /// 4x4 homogeneous transformation matrix (chapter 1: "coordonnees homogenes 3D").
    /// Used to move mesh vertices ourselves, the same way the course's CustomCube.ApplyTransformation does.
    /// </summary>
    [Serializable]
    public struct Mat4
    {
        public float m00, m01, m02, m03;
        public float m10, m11, m12, m13;
        public float m20, m21, m22, m23;
        public float m30, m31, m32, m33;

        public static Mat4 Identity
        {
            get
            {
                Mat4 m = default;
                m.m00 = m.m11 = m.m22 = m.m33 = 1f;
                return m;
            }
        }

        /// <summary>Translation matrix (chapter 2 part 1, slide 12: m03, m13, m23 hold the translation).</summary>
        public static Mat4 Translation(Vector3 t)
        {
            Mat4 m = Identity;
            m.m03 = t.x; m.m13 = t.y; m.m23 = t.z;
            return m;
        }

        public static Mat4 Scale(Vector3 s)
        {
            Mat4 m = Identity;
            m.m00 = s.x; m.m11 = s.y; m.m22 = s.z;
            return m;
        }

        public static Mat4 Rotation(Mat3 r)
        {
            Mat4 m = Identity;
            m.m00 = r.m00; m.m01 = r.m01; m.m02 = r.m02;
            m.m10 = r.m10; m.m11 = r.m11; m.m12 = r.m12;
            m.m20 = r.m20; m.m21 = r.m21; m.m22 = r.m22;
            return m;
        }

        public static Mat4 Rotation(Quat q)
        {
            return Rotation(q.ToMat3());
        }

        /// <summary>Translation * Rotation * Scale, built directly.</summary>
        public static Mat4 TRS(Vector3 t, Quat q, Vector3 s)
        {
            Mat3 r = q.ToMat3();
            Mat4 m = Identity;
            m.m00 = r.m00 * s.x; m.m01 = r.m01 * s.y; m.m02 = r.m02 * s.z; m.m03 = t.x;
            m.m10 = r.m10 * s.x; m.m11 = r.m11 * s.y; m.m12 = r.m12 * s.z; m.m13 = t.y;
            m.m20 = r.m20 * s.x; m.m21 = r.m21 * s.y; m.m22 = r.m22 * s.z; m.m23 = t.z;
            return m;
        }

        public static Mat4 operator *(Mat4 a, Mat4 b)
        {
            Mat4 r;
            r.m00 = a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20 + a.m03 * b.m30;
            r.m01 = a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21 + a.m03 * b.m31;
            r.m02 = a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22 + a.m03 * b.m32;
            r.m03 = a.m00 * b.m03 + a.m01 * b.m13 + a.m02 * b.m23 + a.m03 * b.m33;

            r.m10 = a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20 + a.m13 * b.m30;
            r.m11 = a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31;
            r.m12 = a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32;
            r.m13 = a.m10 * b.m03 + a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33;

            r.m20 = a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20 + a.m23 * b.m30;
            r.m21 = a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31;
            r.m22 = a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22 + a.m23 * b.m32;
            r.m23 = a.m20 * b.m03 + a.m21 * b.m13 + a.m22 * b.m23 + a.m23 * b.m33;

            r.m30 = a.m30 * b.m00 + a.m31 * b.m10 + a.m32 * b.m20 + a.m33 * b.m30;
            r.m31 = a.m30 * b.m01 + a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31;
            r.m32 = a.m30 * b.m02 + a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32;
            r.m33 = a.m30 * b.m03 + a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33;
            return r;
        }

        /// <summary>Transforms a point (w = 1), with perspective divide when w != 1.</summary>
        public Vector3 MultiplyPoint(Vector3 p)
        {
            float x = m00 * p.x + m01 * p.y + m02 * p.z + m03;
            float y = m10 * p.x + m11 * p.y + m12 * p.z + m13;
            float z = m20 * p.x + m21 * p.y + m22 * p.z + m23;
            float w = m30 * p.x + m31 * p.y + m32 * p.z + m33;
            if (Mathf.Abs(w - 1f) > 1e-7f && Mathf.Abs(w) > 1e-12f)
            {
                float inv = 1f / w;
                return new Vector3(x * inv, y * inv, z * inv);
            }
            return new Vector3(x, y, z);
        }

        /// <summary>Transforms a direction (w = 0): translation is ignored.</summary>
        public Vector3 MultiplyVector(Vector3 v)
        {
            return new Vector3(
                m00 * v.x + m01 * v.y + m02 * v.z,
                m10 * v.x + m11 * v.y + m12 * v.z,
                m20 * v.x + m21 * v.y + m22 * v.z);
        }

        public Mat3 UpperLeft => new Mat3(m00, m01, m02, m10, m11, m12, m20, m21, m22);

        public Vector3 TranslationPart => new Vector3(m03, m13, m23);

        /// <summary>Inverse of an affine matrix [A t; 0 1] = [A^-1, -A^-1 t; 0 1].</summary>
        public Mat4 InverseAffine()
        {
            Mat3 inv = UpperLeft.Inverse;
            Vector3 t = -(inv * TranslationPart);
            Mat4 m = Rotation(inv);
            m.m03 = t.x; m.m13 = t.y; m.m23 = t.z;
            return m;
        }

        /// <summary>Copy into Unity's matrix type, only to submit instanced draws to the renderer.</summary>
        public Matrix4x4 ToUnity()
        {
            Matrix4x4 m;
            m.m00 = m00; m.m01 = m01; m.m02 = m02; m.m03 = m03;
            m.m10 = m10; m.m11 = m11; m.m12 = m12; m.m13 = m13;
            m.m20 = m20; m.m21 = m21; m.m22 = m22; m.m23 = m23;
            m.m30 = m30; m.m31 = m31; m.m32 = m32; m.m33 = m33;
            return m;
        }
    }
}
