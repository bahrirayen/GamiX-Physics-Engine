using System.Collections.Generic;
using UnityEngine;

namespace GamiX.Physics.RigidBodies
{
    /// <summary>
    /// Body-space inertia tensors I_body of homogeneous solids, expressed at their center of mass
    /// (chapter 2 part 2, slide 33). They are diagonal in the object's own axes.
    /// </summary>
    public static class InertiaTensor
    {
        /// <summary>Rectangular box of full sides a (x), b (y), c (z): m/12 diag(b^2+c^2, a^2+c^2, a^2+b^2).</summary>
        public static Mat3 Box(float mass, Vector3 size)
        {
            float a2 = size.x * size.x, b2 = size.y * size.y, c2 = size.z * size.z;
            float k = mass / 12f;
            return Mat3.Diagonal(k * (b2 + c2), k * (a2 + c2), k * (a2 + b2));
        }

        /// <summary>Solid sphere: 2/5 m R^2 on the diagonal.</summary>
        public static Mat3 SolidSphere(float mass, float radius)
        {
            float i = 0.4f * mass * radius * radius;
            return Mat3.Diagonal(i, i, i);
        }

        /// <summary>Hollow sphere (thin shell): 2/3 m R^2 on the diagonal.</summary>
        public static Mat3 HollowSphere(float mass, float radius)
        {
            float i = 2f / 3f * mass * radius * radius;
            return Mat3.Diagonal(i, i, i);
        }

        /// <summary>
        /// Solid cylinder of radius R and length l around the given axis (0 = x, 1 = y, 2 = z):
        /// 1/2 m R^2 along the axis, 1/4 m R^2 + 1/12 m l^2 across it.
        /// </summary>
        public static Mat3 SolidCylinder(float mass, float radius, float length, int axis = 1)
        {
            float along = 0.5f * mass * radius * radius;
            float across = 0.25f * mass * radius * radius + mass * length * length / 12f;
            return AxisDiagonal(along, across, axis);
        }

        /// <summary>Hollow cylinder (thin tube): m R^2 along the axis, 1/2 m R^2 + 1/12 m l^2 across it.</summary>
        public static Mat3 HollowCylinder(float mass, float radius, float length, int axis = 1)
        {
            float along = mass * radius * radius;
            float across = 0.5f * mass * radius * radius + mass * length * length / 12f;
            return AxisDiagonal(along, across, axis);
        }

        /// <summary>
        /// Capsule (cylinder of length l + two hemispheres of radius R) along the given axis.
        /// Mass is split by volume; hemispheres are moved with the parallel axis theorem.
        /// </summary>
        public static Mat3 Capsule(float mass, float radius, float cylinderLength, int axis = 1)
        {
            float vCyl = Mathf.PI * radius * radius * cylinderLength;
            float vSph = 4f / 3f * Mathf.PI * radius * radius * radius;
            float mCyl = mass * vCyl / (vCyl + vSph);
            float mHemi = 0.5f * (mass - mCyl);

            float cylAlong = 0.5f * mCyl * radius * radius;
            float cylAcross = 0.25f * mCyl * radius * radius + mCyl * cylinderLength * cylinderLength / 12f;

            // Hemisphere about its own center of mass (3R/8 from the flat face), then shifted.
            float hemiAlong = 0.4f * mHemi * radius * radius;
            float hemiAcrossCm = 0.4f * mHemi * radius * radius - mHemi * (3f * radius / 8f) * (3f * radius / 8f);
            float dist = 0.5f * cylinderLength + 3f * radius / 8f;
            float hemiAcross = hemiAcrossCm + mHemi * dist * dist;

            return AxisDiagonal(cylAlong + 2f * hemiAlong, cylAcross + 2f * hemiAcross, axis);
        }

        private static Mat3 AxisDiagonal(float along, float across, int axis)
        {
            switch (axis)
            {
                case 0: return Mat3.Diagonal(along, across, across);
                case 2: return Mat3.Diagonal(across, across, along);
                default: return Mat3.Diagonal(across, along, across);
            }
        }

        /// <summary>
        /// Inertia tensor of a set of point masses around their center of mass (chapter 2 part 2, slide 31):
        /// I = sum m_i [ (y^2+z^2) -xy -xz ; -yx (x^2+z^2) -yz ; -zx -zy (x^2+y^2) ] with r'_i = r_i - c.
        /// Works for any shape made of particles (e.g. voxelized meshes).
        /// </summary>
        public static Mat3 FromPointMasses(IReadOnlyList<Vector3> points, IReadOnlyList<float> masses,
                                           out Vector3 centerOfMass, out float totalMass)
        {
            totalMass = 0f;
            centerOfMass = VMath.Zero;
            for (int i = 0; i < points.Count; i++)
            {
                totalMass += masses[i];
                centerOfMass += points[i] * masses[i];
            }
            if (totalMass > 0f)
                centerOfMass /= totalMass;

            Mat3 inertia = Mat3.Zero;
            for (int i = 0; i < points.Count; i++)
            {
                Vector3 r = points[i] - centerOfMass;
                float m = masses[i];
                inertia.m00 += m * (r.y * r.y + r.z * r.z);
                inertia.m11 += m * (r.x * r.x + r.z * r.z);
                inertia.m22 += m * (r.x * r.x + r.y * r.y);
                inertia.m01 -= m * r.x * r.y;
                inertia.m02 -= m * r.x * r.z;
                inertia.m12 -= m * r.y * r.z;
            }
            inertia.m10 = inertia.m01;
            inertia.m20 = inertia.m02;
            inertia.m21 = inertia.m12;
            return inertia;
        }

        /// <summary>Parallel axis (Huygens-Steiner) theorem: I_P = I_cm + m (|d|^2 E - d d^T).</summary>
        public static Mat3 ParallelAxis(Mat3 inertiaAtCenter, float mass, Vector3 offset)
        {
            float d2 = VMath.Dot(offset, offset);
            return inertiaAtCenter + (Mat3.Diagonal(d2, d2, d2) - Mat3.Outer(offset, offset)) * mass;
        }
    }
}
