using System;
using GamiX.Physics.Geometry;
using UnityEngine;

namespace GamiX.Physics.Collision
{
    /// <summary>
    /// When does a collision happen inside a time step? (chapter 4, slides 9-18).
    /// Knowing t0 &lt; tc &lt; t0 + dt, tc is found by interpolation, by bisection or analytically (continuous detection).
    /// </summary>
    public static class TimeOfImpact
    {
        /// <summary>
        /// Linear interpolation of the signed distance s = y - r between the start (s0 &gt; 0) and the end (s1 &lt; 0)
        /// of the step (chapter 4, application 1): tc = dt s0 / (s0 - s1).
        /// </summary>
        public static float Interpolate(float s0, float s1, float dt)
        {
            if (s0 <= 0f)
                return 0f;
            float denom = s0 - s1;
            return denom > 1e-12f ? dt * s0 / denom : 0f;
        }

        /// <summary>
        /// Bisection on the time of impact (chapter 4, slide 11): <paramref name="signedDistanceAt"/> simulates from t0 to
        /// t0 + tau and returns the signed distance. If interpenetration is found at t0 + dt/2, tc is in the first half,
        /// otherwise in the second half, and so on until the tolerance is reached.
        /// </summary>
        public static float Bisection(Func<float, float> signedDistanceAt, float dt, float tolerance = 1e-4f, int maxIterations = 32)
        {
            float lo = 0f, hi = dt;
            for (int i = 0; i < maxIterations && hi - lo > tolerance; i++)
            {
                float mid = 0.5f * (lo + hi);
                if (signedDistanceAt(mid) < 0f)
                    hi = mid;
                else
                    lo = mid;
            }
            return lo; // Last time known without interpenetration.
        }

        /// <summary>
        /// Continuous detection (CCD) of a sphere moving in a straight line against a plane
        /// (chapter 4, application 2). Returns false if no contact in [0, dt].
        /// </summary>
        public static bool SweptSpherePlane(Vector3 center, float radius, Vector3 velocity, Plane3D plane, float dt, out float toi)
        {
            toi = 0f;
            float s0 = plane.SignedDistance(center) - radius;
            if (s0 <= 0f)
                return true; // Already touching.
            float vn = VMath.Dot(velocity, plane.normal);
            if (vn >= 0f)
                return false; // Moving away.
            float t = -s0 / vn;
            if (t > dt)
                return false;
            toi = t;
            return true;
        }

        /// <summary>CCD between two moving spheres: first root of |dp + dv t| = rA + rB in [0, dt].</summary>
        public static bool SweptSphereSphere(Vector3 pa, Vector3 va, float ra, Vector3 pb, Vector3 vb, float rb, float dt, out float toi)
        {
            toi = 0f;
            Vector3 dp = pb - pa, dv = vb - va;
            float r = ra + rb;
            float c = VMath.Dot(dp, dp) - r * r;
            if (c <= 0f)
                return true;
            float a = VMath.Dot(dv, dv);
            float b = 2f * VMath.Dot(dp, dv);
            if (b >= 0f)
                return false;
            if (Numerics.SolveQuadratic(a, b, c, out float t0, out _) == 0)
                return false;
            if (t0 < 0f || t0 > dt)
                return false;
            toi = t0;
            return true;
        }

        /// <summary>
        /// Number of sub-steps so that each displacement stays below the obstacle thickness + 2r
        /// (chapter 4, application 2: n = ceil(v dt / (e + 2r))).
        /// </summary>
        public static int SubSteps(float speed, float dt, float thickness, float radius)
        {
            float e = thickness + 2f * radius;
            return e > 0f ? Mathf.Max(1, Mathf.CeilToInt(speed * dt / e)) : 1;
        }
    }
}
