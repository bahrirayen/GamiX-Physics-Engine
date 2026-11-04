using UnityEngine;

namespace GamiX.Physics.Geometry
{
    /// <summary>Distance and closest-point queries (chapter 3).</summary>
    public static class Distance
    {
        /// <summary>
        /// Distance from Q to the line (S, V): d = sqrt(|Q - S|^2 - ((Q - S) . V/|V|)^2) (chapter 3, slide 12).
        /// </summary>
        public static float PointLine(Vector3 q, Line line)
        {
            Vector3 sq = q - line.point;
            float len = VMath.Length(line.direction);
            if (len < VMath.Epsilon)
                return VMath.Length(sq);
            float proj = VMath.Dot(sq, line.direction) / len;
            float d2 = VMath.LengthSq(sq) - proj * proj;
            return Mathf.Sqrt(d2 > 0f ? d2 : 0f);
        }

        /// <summary>Parameter t of the orthogonal projection of q on the line.</summary>
        public static float ProjectOnLine(Vector3 q, Line line)
        {
            float vv = VMath.Dot(line.direction, line.direction);
            return vv > VMath.Epsilon * VMath.Epsilon ? VMath.Dot(q - line.point, line.direction) / vv : 0f;
        }

        public static Vector3 ClosestPointOnSegment(Vector3 q, Segment s, out float t)
        {
            Vector3 ab = s.b - s.a;
            float denom = VMath.Dot(ab, ab);
            t = denom > VMath.Epsilon * VMath.Epsilon ? VMath.Clamp01(VMath.Dot(q - s.a, ab) / denom) : 0f;
            return s.a + ab * t;
        }

        public static float PointSegment(Vector3 q, Segment s)
        {
            return VMath.Length(q - ClosestPointOnSegment(q, s, out _));
        }

        /// <summary>Distance |A x1 + B y1 + C z1 + D| / sqrt(A^2 + B^2 + C^2) (chapter 3, slide 26).</summary>
        public static float PointPlane(Vector3 p, Plane3D plane)
        {
            return Mathf.Abs(plane.SignedDistance(p));
        }

        /// <summary>Same formula with a non-normalized plane given as (n, D).</summary>
        public static float PointPlane(Vector3 p, Vector3 n, float d)
        {
            return Mathf.Abs(VMath.Dot(n, p) + d) / VMath.Length(n);
        }

        /// <summary>
        /// Minimal distance between two lines P1(t1) = S1 + t1 V1 and P2(t2) = S2 + t2 V2 (chapter 3, slides 16-20).
        /// Setting df/dt1 = df/dt2 = 0 gives the 2x2 system
        /// [V1.V1  -V1.V2] [t1]   [-(S1-S2).V1]
        /// [-V1.V2  V2.V2] [t2] = [ (S1-S2).V2]
        /// Returns false when the lines are parallel (the matrix is singular).
        /// </summary>
        public static bool LineLine(Line l1, Line l2, out float t1, out float t2, out float distance)
        {
            Vector3 v1 = l1.direction, v2 = l2.direction;
            Vector3 s12 = l1.point - l2.point;
            float v11 = VMath.Dot(v1, v1);
            float v22 = VMath.Dot(v2, v2);
            float v12 = VMath.Dot(v1, v2);
            bool ok = Numerics.Solve2x2(v11, -v12, -v12, v22, -VMath.Dot(s12, v1), VMath.Dot(s12, v2), out t1, out t2);
            if (!ok)
            {
                // Parallel lines: every point is at the same distance.
                t1 = 0f;
                t2 = ProjectOnLine(l1.point, l2);
            }
            distance = VMath.Length(l1.GetPoint(t1) - l2.GetPoint(t2));
            return ok;
        }

        /// <summary>
        /// Closest points between two segments (the line-line solution clamped to [0, 1]).
        /// Used for edge-edge box contacts and capsule tests.
        /// </summary>
        public static float SegmentSegment(Segment s1, Segment s2, out Vector3 c1, out Vector3 c2, out float t1, out float t2)
        {
            Vector3 d1 = s1.b - s1.a, d2 = s2.b - s2.a, r = s1.a - s2.a;
            float a = VMath.Dot(d1, d1), e = VMath.Dot(d2, d2), f = VMath.Dot(d2, r);
            const float eps = 1e-12f;

            if (a <= eps && e <= eps)
            {
                t1 = t2 = 0f;
            }
            else if (a <= eps)
            {
                t1 = 0f;
                t2 = VMath.Clamp01(f / e);
            }
            else
            {
                float c = VMath.Dot(d1, r);
                if (e <= eps)
                {
                    t2 = 0f;
                    t1 = VMath.Clamp01(-c / a);
                }
                else
                {
                    float b = VMath.Dot(d1, d2);
                    float denom = a * e - b * b;
                    t1 = denom > eps ? VMath.Clamp01((b * f - c * e) / denom) : 0f;
                    t2 = (b * t1 + f) / e;
                    if (t2 < 0f)
                    {
                        t2 = 0f;
                        t1 = VMath.Clamp01(-c / a);
                    }
                    else if (t2 > 1f)
                    {
                        t2 = 1f;
                        t1 = VMath.Clamp01((b - c) / a);
                    }
                }
            }
            c1 = s1.a + d1 * t1;
            c2 = s2.a + d2 * t2;
            return VMath.Length(c1 - c2);
        }

        /// <summary>Closest point of a triangle to p (Voronoi region method, Ericson "Real-Time Collision Detection").</summary>
        public static Vector3 ClosestPointOnTriangle(Vector3 p, Triangle tri)
        {
            Vector3 a = tri.a, b = tri.b, c = tri.c;
            Vector3 ab = b - a, ac = c - a, ap = p - a;
            float d1 = VMath.Dot(ab, ap), d2 = VMath.Dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f) return a;

            Vector3 bp = p - b;
            float d3 = VMath.Dot(ab, bp), d4 = VMath.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3) return b;

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
                return a + ab * (d1 / (d1 - d3));

            Vector3 cp = p - c;
            float d5 = VMath.Dot(ab, cp), d6 = VMath.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6) return c;

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
                return a + ac * (d2 / (d2 - d6));

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
                return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

            float denom = 1f / (va + vb + vc);
            float v = vb * denom, w = vc * denom;
            return a + ab * v + ac * w;
        }

        /// <summary>
        /// Barycentric coordinates of P in triangle (P0, P1, P2) (chapter 3, slides 34-35):
        /// R = w1 Q1 + w2 Q2 with R = P - P0, Q1 = P1 - P0, Q2 = P2 - P0, solved as a 2x2 system; w0 = 1 - w1 - w2.
        /// </summary>
        public static bool Barycentric(Vector3 p, Vector3 p0, Vector3 p1, Vector3 p2, out Vector3 w)
        {
            Vector3 r = p - p0, q1 = p1 - p0, q2 = p2 - p0;
            float q11 = VMath.Dot(q1, q1), q12 = VMath.Dot(q1, q2), q22 = VMath.Dot(q2, q2);
            bool ok = Numerics.Solve2x2(q11, q12, q12, q22, VMath.Dot(r, q1), VMath.Dot(r, q2), out float w1, out float w2);
            w = new Vector3(1f - w1 - w2, w1, w2);
            return ok;
        }

        /// <summary>
        /// Intersection point of three planes (chapter 3, slide 28): solve [n1; n2; n3] Q = [-D1; -D2; -D3].
        /// Returns false when the determinant is zero (slide 29).
        /// </summary>
        public static bool ThreePlanes(Plane3D p1, Plane3D p2, Plane3D p3, out Vector3 point)
        {
            Mat3 m = Mat3.FromRows(p1.normal, p2.normal, p3.normal);
            return m.Solve(new Vector3(-p1.d, -p2.d, -p3.d), out point);
        }
    }
}
