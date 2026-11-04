using UnityEngine;

namespace GamiX.Physics.Geometry
{
    /// <summary>
    /// Ray casting (chapter 3 and chapter 4 slide 64). Every query returns the smallest t >= 0 in [0, maxDistance].
    /// Primitives other than triangles are intersected in object space, then the hit is brought back to world space
    /// (chapter 3, slide 33).
    /// </summary>
    public static class Raycast
    {
        /// <summary>
        /// Ray-plane (slide 27): n.(S + t V) + D = 0  =>  t = -(D + n.S) / (n.V). No hit when n.V = 0.
        /// </summary>
        public static bool RayPlane(Ray3D ray, Plane3D plane, out RayHit hit, float maxDistance = float.MaxValue)
        {
            hit = default;
            float nv = VMath.Dot(plane.normal, ray.direction);
            if (Mathf.Abs(nv) < VMath.Epsilon)
                return false;
            float t = -(plane.d + VMath.Dot(plane.normal, ray.origin)) / nv;
            if (t < 0f || t > maxDistance)
                return false;
            hit.t = t;
            hit.point = ray.GetPoint(t);
            hit.normal = nv < 0f ? plane.normal : -plane.normal;
            return true;
        }

        /// <summary>
        /// Ray-sphere (chapter 4, slide 64): |o + t d - c|^2 = r^2, m = o - c, b = m.d, c' = m.m - r^2,
        /// Delta = b^2 - c'; first hit t = -b - sqrt(Delta) (or the exit point when the origin is inside).
        /// </summary>
        public static bool RaySphere(Ray3D ray, Sphere sphere, out RayHit hit, float maxDistance = float.MaxValue)
        {
            hit = default;
            Vector3 m = ray.origin - sphere.center;
            float b = VMath.Dot(m, ray.direction);
            float c = VMath.Dot(m, m) - sphere.radius * sphere.radius;
            if (c > 0f && b > 0f)
                return false; // Outside and pointing away.
            float delta = b * b - c;
            if (delta < 0f)
                return false;
            float sq = Mathf.Sqrt(delta);
            float t = -b - sq;
            if (t < 0f)
                t = -b + sq; // Origin inside the sphere.
            if (t < 0f || t > maxDistance)
                return false;
            hit.t = t;
            hit.point = ray.GetPoint(t);
            hit.normal = VMath.Normalize(hit.point - sphere.center);
            return true;
        }

        /// <summary>
        /// Ray-AABB with the slab method (chapter 4, slide 64): per axis t1 = (min - o) / d, t2 = (max - o) / d,
        /// t_enter = max(t1), t_exit = min(t2); hit when t_enter &lt;= t_exit.
        /// </summary>
        public static bool RayAabb(Ray3D ray, Aabb box, out RayHit hit, float maxDistance = float.MaxValue)
        {
            hit = default;
            float tEnter = 0f, tExit = maxDistance;
            int enterAxis = -1;
            float enterSign = 0f;
            for (int i = 0; i < 3; i++)
            {
                float o = ray.origin[i], d = ray.direction[i], mn = box.min[i], mx = box.max[i];
                if (Mathf.Abs(d) < 1e-12f)
                {
                    if (o < mn || o > mx)
                        return false;
                    continue;
                }
                float inv = 1f / d;
                float t1 = (mn - o) * inv, t2 = (mx - o) * inv;
                float sign = -1f;
                if (t1 > t2)
                {
                    float tmp = t1; t1 = t2; t2 = tmp;
                    sign = 1f;
                }
                if (t1 > tEnter)
                {
                    tEnter = t1;
                    enterAxis = i;
                    enterSign = sign;
                }
                if (t2 < tExit)
                    tExit = t2;
                if (tEnter > tExit)
                    return false;
            }
            hit.t = tEnter;
            hit.point = ray.GetPoint(tEnter);
            Vector3 n = VMath.Zero;
            if (enterAxis >= 0)
                n[enterAxis] = enterSign;
            hit.normal = n;
            return true;
        }

        /// <summary>Ray-OBB: the ray is expressed in the box space, then the AABB slab test is applied.</summary>
        public static bool RayObb(Ray3D ray, Obb box, out RayHit hit, float maxDistance = float.MaxValue)
        {
            Ray3D local = new Ray3D(box.ToLocal(ray.origin), box.rotation.InverseRotate(ray.direction));
            if (!RayAabb(local, new Aabb(-box.halfExtents, box.halfExtents), out hit, maxDistance))
                return false;
            hit.point = box.ToWorld(hit.point);
            hit.normal = box.rotation.Rotate(hit.normal);
            return true;
        }

        /// <summary>
        /// Ray-triangle (chapter 3, slides 34-35): intersect the triangle's plane, then compute the barycentric
        /// coordinates (w0, w1, w2) of the point; inside when all are >= 0.
        /// </summary>
        public static bool RayTriangle(Ray3D ray, Triangle tri, out RayHit hit, float maxDistance = float.MaxValue,
                                       bool cullBackFaces = false)
        {
            hit = default;
            Vector3 n = VMath.Cross(tri.b - tri.a, tri.c - tri.a);
            float nv = VMath.Dot(n, ray.direction);
            if (Mathf.Abs(nv) < 1e-12f)
                return false;
            if (cullBackFaces && nv > 0f)
                return false;
            float t = VMath.Dot(n, tri.a - ray.origin) / nv;
            if (t < 0f || t > maxDistance)
                return false;
            Vector3 p = ray.GetPoint(t);
            if (!Distance.Barycentric(p, tri.a, tri.b, tri.c, out Vector3 w))
                return false;
            const float tol = -1e-6f;
            if (w.x < tol || w.y < tol || w.z < tol)
                return false;
            hit.t = t;
            hit.point = p;
            hit.normal = VMath.Normalize(nv < 0f ? n : -n);
            hit.barycentric = w;
            return true;
        }

        /// <summary>
        /// Ray against a finite capped cylinder. Side: quadratic in t after removing the axial component
        /// (a degree-2 surface, chapter 3 slide 30). Caps: ray-plane + disc test.
        /// </summary>
        public static bool RayCylinder(Ray3D ray, Cylinder cyl, out RayHit hit, float maxDistance = float.MaxValue)
        {
            hit = default;
            hit.t = float.MaxValue;
            Vector3 axis = cyl.b - cyl.a;
            float height = VMath.Length(axis);
            if (height < VMath.Epsilon)
                return false;
            Vector3 u = axis / height;

            Vector3 m = ray.origin - cyl.a;
            Vector3 dPerp = ray.direction - u * VMath.Dot(ray.direction, u);
            Vector3 mPerp = m - u * VMath.Dot(m, u);
            float a = VMath.Dot(dPerp, dPerp);
            float b = 2f * VMath.Dot(dPerp, mPerp);
            float c = VMath.Dot(mPerp, mPerp) - cyl.radius * cyl.radius;
            bool found = false;

            int n = Numerics.SolveQuadratic(a, b, c, out float t0, out float t1);
            for (int k = 0; k < n; k++)
            {
                float t = k == 0 ? t0 : t1;
                if (t < 0f || t > maxDistance || t >= hit.t)
                    continue;
                Vector3 p = ray.GetPoint(t);
                float h = VMath.Dot(p - cyl.a, u);
                if (h < 0f || h > height)
                    continue;
                hit.t = t;
                hit.point = p;
                hit.normal = VMath.Normalize(p - (cyl.a + u * h));
                found = true;
            }

            // End caps.
            for (int cap = 0; cap < 2; cap++)
            {
                Vector3 center = cap == 0 ? cyl.a : cyl.b;
                Vector3 normal = cap == 0 ? -u : u;
                if (RayPlane(ray, Plane3D.FromPointNormal(center, normal), out RayHit ph, maxDistance) &&
                    ph.t < hit.t && VMath.DistanceSq(ph.point, center) <= cyl.radius * cyl.radius)
                {
                    hit.t = ph.t;
                    hit.point = ph.point;
                    hit.normal = normal;
                    found = true;
                }
            }
            return found;
        }

        /// <summary>Ray-capsule: closest of the infinite-cylinder side hit and the two end spheres.</summary>
        public static bool RayCapsule(Ray3D ray, Capsule cap, out RayHit hit, float maxDistance = float.MaxValue)
        {
            hit = default;
            hit.t = float.MaxValue;
            bool found = false;
            if (RayCylinder(ray, new Cylinder(cap.a, cap.b, cap.radius), out RayHit ch, maxDistance))
            {
                // Ignore flat cap hits: the hemispheres replace them.
                Vector3 u = VMath.Normalize(cap.b - cap.a);
                if (Mathf.Abs(VMath.Dot(ch.normal, u)) < 0.999f)
                {
                    hit = ch;
                    found = true;
                }
            }
            for (int s = 0; s < 2; s++)
            {
                if (RaySphere(ray, new Sphere(s == 0 ? cap.a : cap.b, cap.radius), out RayHit sh, maxDistance) && sh.t < hit.t)
                {
                    hit = sh;
                    found = true;
                }
            }
            return found;
        }

        /// <summary>Closest hit among the triangles of an indexed mesh (vertices in world space).</summary>
        public static bool RayMesh(Ray3D ray, Vector3[] vertices, int[] triangles, out RayHit hit,
                                   float maxDistance = float.MaxValue, bool cullBackFaces = false)
        {
            hit = default;
            bool found = false;
            float best = maxDistance;
            for (int i = 0; i + 2 < triangles.Length; i += 3)
            {
                Triangle tri = new Triangle(vertices[triangles[i]], vertices[triangles[i + 1]], vertices[triangles[i + 2]]);
                if (RayTriangle(ray, tri, out RayHit h, best, cullBackFaces))
                {
                    best = h.t;
                    h.index = i / 3;
                    hit = h;
                    found = true;
                }
            }
            return found;
        }
    }
}
