using UnityEngine;

namespace GamiX.Physics.Geometry
{
    /// <summary>Bounding volume construction (chapter 4, slides 21-23 and 29-42).</summary>
    public static class BoundingVolumes
    {
        /// <summary>Method 1 (slide 23): center = middle of the AABB of the points, radius = max |p_i - c|.</summary>
        public static Sphere SphereFromAabb(Vector3[] points)
        {
            Aabb box = Aabb.FromPoints(points);
            return SphereAround(points, box.Center);
        }

        /// <summary>Method 2 (slide 23): center = average of the points.</summary>
        public static Sphere SphereFromCentroid(Vector3[] points)
        {
            Vector3 c = VMath.Zero;
            for (int i = 0; i < points.Length; i++)
                c += points[i];
            c /= Mathf.Max(1, points.Length);
            return SphereAround(points, c);
        }

        /// <summary>
        /// Method 3, Ritter 1990 (slide 23): start from two far-apart points, then grow the sphere for each point
        /// outside it. Typically 5-20% bigger than the optimal sphere.
        /// </summary>
        public static Sphere SphereRitter(Vector3[] points)
        {
            if (points.Length == 0)
                return new Sphere(VMath.Zero, 0f);
            Vector3 x = points[0];
            Vector3 y = FarthestFrom(points, x);
            Vector3 z = FarthestFrom(points, y);
            Vector3 c = (y + z) * 0.5f;
            float r = 0.5f * VMath.Length(z - y);
            for (int i = 0; i < points.Length; i++)
            {
                float d = VMath.Length(points[i] - c);
                if (d > r)
                {
                    float newR = 0.5f * (r + d);
                    c += (points[i] - c) * ((newR - r) / d);
                    r = newR;
                }
            }
            return new Sphere(c, r);
        }

        private static Vector3 FarthestFrom(Vector3[] points, Vector3 p)
        {
            Vector3 best = p;
            float bestD = -1f;
            for (int i = 0; i < points.Length; i++)
            {
                float d = VMath.DistanceSq(points[i], p);
                if (d > bestD)
                {
                    bestD = d;
                    best = points[i];
                }
            }
            return best;
        }

        private static Sphere SphereAround(Vector3[] points, Vector3 c)
        {
            float r2 = 0f;
            for (int i = 0; i < points.Length; i++)
            {
                float d = VMath.DistanceSq(points[i], c);
                if (d > r2)
                    r2 = d;
            }
            return new Sphere(c, Mathf.Sqrt(r2));
        }

        /// <summary>World bounding sphere of a moving object: c_world = R c_local + position, r unchanged (slide 23).</summary>
        public static Sphere TransformSphere(Sphere local, Vector3 position, Quat rotation, float maxScale = 1f)
        {
            return new Sphere(position + rotation.Rotate(local.center * maxScale), local.radius * maxScale);
        }
    }

    /// <summary>Boolean overlap tests between bounding volumes (chapter 4).</summary>
    public static class Overlap
    {
        /// <summary>|cB - cA|^2 &lt; (rA + rB)^2: compares squares, no square root (slide 31).</summary>
        public static bool SphereSphere(Sphere a, Sphere b)
        {
            float r = a.radius + b.radius;
            return VMath.DistanceSq(a.center, b.center) < r * r;
        }

        public static bool AabbAabb(Aabb a, Aabb b) => a.Overlaps(b);

        /// <summary>Closest point q = clamp(c, min, max), collision when |c - q|^2 &lt;= r^2 (slide 37).</summary>
        public static bool SphereAabb(Sphere s, Aabb box)
        {
            return VMath.DistanceSq(s.center, box.ClosestPoint(s.center)) <= s.radius * s.radius;
        }

        /// <summary>Sphere-OBB: express the center in the box frame, then the same test (slide 37).</summary>
        public static bool SphereObb(Sphere s, Obb box)
        {
            return VMath.DistanceSq(s.center, box.ClosestPoint(s.center)) <= s.radius * s.radius;
        }

        /// <summary>
        /// Separating axis theorem for two OBBs (slide 43): 3 + 3 face axes and 9 edge cross products.
        /// L is a separating axis when |T.L| &gt; rA + rB. Near-zero cross products (parallel edges) are skipped.
        /// </summary>
        public static bool ObbObb(Obb a, Obb b)
        {
            Vector3 t = b.center - a.center;
            for (int i = 0; i < 3; i++)
            {
                if (Separates(a.Axis(i), t, a, b)) return false;
                if (Separates(b.Axis(i), t, a, b)) return false;
            }
            for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                Vector3 l = VMath.Cross(a.Axis(i), b.Axis(j));
                if (VMath.LengthSq(l) < 1e-6f)
                    continue;
                if (Separates(l, t, a, b)) return false;
            }
            return true;
        }

        private static bool Separates(Vector3 l, Vector3 t, Obb a, Obb b)
        {
            return Mathf.Abs(VMath.Dot(t, l)) > a.ProjectionRadius(l) + b.ProjectionRadius(l);
        }
    }
}
