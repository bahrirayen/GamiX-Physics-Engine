using GamiX.Physics.Geometry;
using NUnit.Framework;
using UnityEngine;

namespace GamiX.Physics.Tests
{
    /// <summary>Math (chapter 1) and geometry (chapter 3) checked against the worked examples of the slides.</summary>
    public class MathAndGeometryTests
    {
        private const float Tol = 1e-3f;

        private static void Near(Vector3 expected, Vector3 actual, float tol = Tol)
        {
            Assert.That(VMath.Distance(expected, actual), Is.LessThan(tol), $"expected {expected} got {actual}");
        }

        // ---------------------------------------------------------------- chapter 1

        [Test]
        public void TriangleNormal_Slide10()
        {
            // V1(0,2,2), V2(0,1,4), V3(1,1,2): n = S x r = (-2,-2,-1), unit (-2/3,-2/3,-1/3).
            Vector3 v1 = new Vector3(0, 2, 2), v2 = new Vector3(0, 1, 4), v3 = new Vector3(1, 1, 2);
            Vector3 n = VMath.Cross(v2 - v3, v1 - v3);
            Near(new Vector3(-2, -2, -1), n);
            Near(new Vector3(-2f / 3f, -2f / 3f, -1f / 3f), VMath.Normalize(n));
        }

        [Test]
        public void BackFace_Slide12()
        {
            // Camera (0,0,0), vertex (10,10,40), normal (5,5,-2): V.n = -20, beta = 93.68 deg, not visible.
            Vector3 c = Vector3.zero, a = new Vector3(10, 10, 40), n = new Vector3(5, 5, -2);
            Vector3 v = c - a;
            Assert.AreEqual(-20f, VMath.Dot(v, n), Tol);
            float beta = Mathf.Acos(VMath.Dot(v, n) / (VMath.Length(v) * VMath.Length(n))) * Mathf.Rad2Deg;
            Assert.AreEqual(93.68f, beta, 0.01f);
            Assert.IsFalse(VMath.IsFaceVisible(a, n, c));
        }

        [Test]
        public void Quaternion_Slide56Table()
        {
            float s = 1f / Mathf.Sqrt(2f);
            Quat yaw = Quat.FromAxisAngleDeg(Vector3.up, 90f);
            Assert.AreEqual(s, yaw.w, Tol); Assert.AreEqual(s, yaw.y, Tol);
            Quat roll = Quat.FromAxisAngleDeg(new Vector3(0, 0, 1), 180f);
            Assert.AreEqual(0f, roll.w, Tol); Assert.AreEqual(1f, roll.z, Tol);
            // Two 90 degree yaws make a 180 degree yaw.
            Quat twice = yaw * yaw;
            Assert.AreEqual(0f, twice.w, Tol); Assert.AreEqual(1f, Mathf.Abs(twice.y), Tol);
        }

        [Test]
        public void Quaternion_RotateMatchesHamiltonAndMatrix()
        {
            Quat q = Quat.FromAxisAngleDeg(new Vector3(0.3f, -1f, 2f), 73f);
            Vector3 p = new Vector3(1.5f, -2f, 0.7f);
            Near(q.RotateHamilton(p), q.Rotate(p));
            Near(q.ToMat3() * p, q.Rotate(p));
            Near(p, q.InverseRotate(q.Rotate(p)));
            Quat back = Quat.FromMat3(q.ToMat3());
            Assert.AreEqual(1f, Mathf.Abs(Quat.Dot(q, back)), Tol);
        }

        [Test]
        public void Quaternion_SlerpMidpoint()
        {
            Quat a = Quat.Identity, b = Quat.FromAxisAngleDeg(Vector3.right, 90f);
            Quat m = Quat.Slerp(a, b, 0.5f);
            Quat expected = Quat.FromAxisAngleDeg(Vector3.right, 45f);
            Assert.AreEqual(1f, Mathf.Abs(Quat.Dot(m, expected)), 1e-4f);
        }

        [Test]
        public void Mat3_InverseAndGramSchmidt()
        {
            Mat3 m = new Mat3(2, 1, 0, 1, 3, 1, 0, 1, 4);
            Mat3 id = m * m.Inverse;
            Assert.AreEqual(1f, id.m00, Tol); Assert.AreEqual(1f, id.m11, Tol); Assert.AreEqual(0f, id.m01, Tol);
            Mat3 drifted = Quat.FromAxisAngleDeg(Vector3.up, 30f).ToMat3() + new Mat3(0.01f, 0.02f, 0, 0, 0.01f, 0.03f, 0.02f, 0, 0);
            Mat3 r = Mat3.Orthonormalize(drifted);
            Mat3 rrt = r * r.Transposed;
            Assert.AreEqual(1f, rrt.m00, Tol); Assert.AreEqual(0f, rrt.m01, Tol); Assert.AreEqual(1f, r.Determinant, Tol);
        }

        [Test]
        public void Mat3_ExtractRotationRecoversRotation()
        {
            Quat q = Quat.FromAxisAngleDeg(new Vector3(1, 2, 3), 50f);
            Mat3 a = q.ToMat3() * Mat3.Diagonal(1.3f, 0.8f, 1.1f); // rotation times stretch
            Quat r = Quat.Identity;
            Mat3.ExtractRotation(a, ref r, 50);
            Assert.AreEqual(1f, Mathf.Abs(Quat.Dot(q, r)), 1e-3f);
        }

        // ---------------------------------------------------------------- chapter 3

        [Test]
        public void PointLineDistance_Slide14()
        {
            // A(1,4,-5), B(2,-3,8), C(2,3,6): |BA|^2 = 219, BA.V = 151, V.V = 123, d = sqrt(219 - 151^2/123) = 5.7988.
            // (The slide finds 5.78 because it rounds the projection to (1.23, -1.23, 13.51) before squaring.)
            Line l = Line.FromPoints(new Vector3(1, 4, -5), new Vector3(2, 3, 6));
            Assert.AreEqual(Mathf.Sqrt(219f - 151f * 151f / 123f), Distance.PointLine(new Vector3(2, -3, 8), l), 1e-4f);
            Assert.AreEqual(5.78f, Distance.PointLine(new Vector3(2, -3, 8), l), 0.02f);
        }

        [Test]
        public void LineLineDistance_Slides19to20()
        {
            // (AB) and (CD) with A(1,0,4), B(-1,1,3), C(0,5,4), D(1,2,2): t1 = 2/3, t2 = 1, d = 4 sqrt(3) / 3.
            Line l1 = Line.FromPoints(new Vector3(1, 0, 4), new Vector3(-1, 1, 3));
            Line l2 = Line.FromPoints(new Vector3(0, 5, 4), new Vector3(1, 2, 2));
            Assert.IsTrue(Distance.LineLine(l1, l2, out float t1, out float t2, out float d));
            Assert.AreEqual(2f / 3f, t1, Tol);
            Assert.AreEqual(1f, t2, Tol);
            Assert.AreEqual(4f * Mathf.Sqrt(3f) / 3f, d, Tol);
        }

        [Test]
        public void PointPlaneDistance_Slide26()
        {
            // Plane x + 2y - z - 5 = 0, point (3,2,1): |3 + 4 - 1 - 5| / sqrt(6).
            Assert.AreEqual(1f / Mathf.Sqrt(6f), Distance.PointPlane(new Vector3(3, 2, 1), new Vector3(1, 2, -1), -5f), Tol);
        }

        [Test]
        public void RayPlane_Slide27()
        {
            Plane3D plane = Plane3D.FromPointNormal(new Vector3(0, 2, 0), Vector3.up);
            Assert.IsTrue(Raycast.RayPlane(new Ray3D(Vector3.zero, new Vector3(1, 1, 0)), plane, out RayHit hit));
            Near(new Vector3(2, 2, 0), hit.point);
            Assert.IsFalse(Raycast.RayPlane(new Ray3D(Vector3.zero, Vector3.right), plane, out _)); // n.V = 0
        }

        [Test]
        public void ThreePlanes_Slide28()
        {
            Plane3D px = new Plane3D(new Vector3(1, 0, 0), -1f), py = new Plane3D(new Vector3(0, 1, 0), -2f), pz = new Plane3D(new Vector3(0, 0, 1), -3f);
            Assert.IsTrue(Distance.ThreePlanes(px, py, pz, out Vector3 q));
            Near(new Vector3(1, 2, 3), q);
            Assert.IsFalse(Distance.ThreePlanes(px, px, pz, out _)); // det = 0
        }

        [Test]
        public void RaySphereAabbObbTriangle()
        {
            Ray3D ray = new Ray3D(new Vector3(-5, 0, 0), Vector3.right);
            Assert.IsTrue(Raycast.RaySphere(ray, new Sphere(Vector3.zero, 1f), out RayHit h));
            Assert.AreEqual(4f, h.t, Tol);
            Assert.IsTrue(Raycast.RayAabb(ray, new Aabb(-Vector3.one, Vector3.one), out h));
            Assert.AreEqual(4f, h.t, Tol);
            Near(new Vector3(-1, 0, 0), h.normal);
            Obb obb = new Obb(Vector3.zero, Quat.FromAxisAngleDeg(Vector3.up, 45f), Vector3.one);
            Assert.IsTrue(Raycast.RayObb(ray, obb, out h));
            Assert.AreEqual(5f - Mathf.Sqrt(2f), h.t, Tol);
            Triangle tri = new Triangle(new Vector3(0, -1, -1), new Vector3(0, 1, -1), new Vector3(0, 0, 1));
            Assert.IsTrue(Raycast.RayTriangle(ray, tri, out h));
            Assert.AreEqual(5f, h.t, Tol);
            Assert.AreEqual(1f, h.barycentric.x + h.barycentric.y + h.barycentric.z, Tol);
            Assert.IsFalse(Raycast.RayTriangle(new Ray3D(new Vector3(-5, 3, 0), Vector3.right), tri, out _));
        }

        [Test]
        public void RayCylinder()
        {
            Cylinder cyl = new Cylinder(new Vector3(0, -1, 0), new Vector3(0, 1, 0), 0.5f);
            Assert.IsTrue(Raycast.RayCylinder(new Ray3D(new Vector3(-3, 0, 0), Vector3.right), cyl, out RayHit h));
            Assert.AreEqual(2.5f, h.t, Tol);
            Assert.IsTrue(Raycast.RayCylinder(new Ray3D(new Vector3(0, 5, 0), Vector3.down), cyl, out h));
            Assert.AreEqual(4f, h.t, Tol); // top cap
        }

        [Test]
        public void PolynomialRoots_Slides31to32()
        {
            Assert.AreEqual(2, Numerics.SolveQuadratic(1, -3, 2, out float r0, out float r1));
            Assert.AreEqual(1f, r0, Tol); Assert.AreEqual(2f, r1, Tol);
            float[] roots = new float[3];
            // (t - 1)(t - 2)(t - 3) = t^3 - 6t^2 + 11t - 6
            Assert.AreEqual(3, Numerics.SolveCubic(-6, 11, -6, roots));
            Assert.AreEqual(1f, roots[0], Tol); Assert.AreEqual(2f, roots[1], Tol); Assert.AreEqual(3f, roots[2], Tol);
            // t^3 - 1 = 0 has a single real root.
            Assert.AreEqual(1, Numerics.SolveCubic(0, 0, -1, roots));
            Assert.AreEqual(1f, roots[0], Tol);
        }

        [Test]
        public void Bisection_Chapter4Slide12()
        {
            float x = Numerics.Bisection(t => t * t - 2f, 0f, 2f, 1e-6f);
            Assert.AreEqual(Mathf.Sqrt(2f), x, 1e-5f);
        }

        [Test]
        public void BoundingSpheres_Slide23()
        {
            Vector3[] pts = { new Vector3(-1, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, -1, 0), new Vector3(0, 0, 0.5f) };
            foreach (Sphere s in new[] { BoundingVolumes.SphereFromAabb(pts), BoundingVolumes.SphereFromCentroid(pts), BoundingVolumes.SphereRitter(pts) })
                foreach (Vector3 p in pts)
                    Assert.LessOrEqual(VMath.Distance(p, s.center), s.radius + 1e-4f);
        }
    }
}
