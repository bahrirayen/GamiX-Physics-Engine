using System.Collections.Generic;
using GamiX.Physics.Geometry;
using UnityEngine;

namespace GamiX.Physics.Collision
{
    /// <summary>One contact point (p, n, delta) of chapter 4, slide 72. The normal points from A to B.</summary>
    public struct Contact
    {
        public Vector3 point;
        public Vector3 normal;
        public float penetration;

        public Contact(Vector3 point, Vector3 normal, float penetration)
        {
            this.point = point;
            this.normal = normal;
            this.penetration = penetration;
        }
    }

    /// <summary>
    /// Narrow phase: exact tests that also produce contact data (point, normal from A to B, penetration depth).
    /// Every function appends its contacts to <c>contacts</c> and returns true on collision.
    /// </summary>
    public static class NarrowPhase
    {
        /// <summary>
        /// Sphere-sphere (chapter 4, slide 31): n = d/|d| (A to B), depth = rA + rB - |d|,
        /// point = cA + n (rA - depth/2). Degenerate case (same centers): arbitrary normal (0, 1, 0).
        /// </summary>
        public static bool SphereSphere(Sphere a, Sphere b, List<Contact> contacts)
        {
            Vector3 d = b.center - a.center;
            float rSum = a.radius + b.radius;
            float dist2 = VMath.LengthSq(d);
            if (dist2 >= rSum * rSum)
                return false;
            float dist = Mathf.Sqrt(dist2);
            Vector3 n = dist > 1e-6f ? d / dist : VMath.Up;
            float pen = rSum - dist;
            contacts.Add(new Contact(a.center + n * (a.radius - 0.5f * pen), n, pen));
            return true;
        }

        /// <summary>Sphere (A) against a plane (B). The normal points from the sphere into the plane.</summary>
        public static bool SpherePlane(Sphere s, Plane3D plane, List<Contact> contacts)
        {
            float dist = plane.SignedDistance(s.center);
            if (dist >= s.radius)
                return false;
            float pen = s.radius - dist;
            Vector3 p = s.center - plane.normal * (dist + 0.5f * pen);
            contacts.Add(new Contact(p, -plane.normal, pen));
            return true;
        }

        /// <summary>
        /// Sphere (A) against an oriented box (B) through the box's closest point (chapter 4, slide 37).
        /// When the center is inside the box, the nearest face gives the normal.
        /// </summary>
        public static bool SphereObb(Sphere s, Obb box, List<Contact> contacts)
        {
            Vector3 local = box.ToLocal(s.center);
            Vector3 e = box.halfExtents;
            Vector3 q = VMath.Clamp(local, -e, e);
            Vector3 d = local - q;
            float dist2 = VMath.LengthSq(d);
            if (dist2 > s.radius * s.radius)
                return false;

            if (dist2 > 1e-12f)
            {
                float dist = Mathf.Sqrt(dist2);
                Vector3 nLocal = -d / dist; // From the sphere toward the box.
                Vector3 n = box.rotation.Rotate(nLocal);
                contacts.Add(new Contact(box.ToWorld(q), n, s.radius - dist));
                return true;
            }

            // Center inside the box: push out through the closest face.
            float best = float.MaxValue;
            int axis = 0;
            float sign = 1f;
            for (int i = 0; i < 3; i++)
            {
                float faceDist = e[i] - Mathf.Abs(local[i]);
                if (faceDist < best)
                {
                    best = faceDist;
                    axis = i;
                    sign = local[i] >= 0f ? 1f : -1f;
                }
            }
            Vector3 faceNormalLocal = VMath.Zero;
            faceNormalLocal[axis] = sign;
            Vector3 outward = box.rotation.Rotate(faceNormalLocal);
            contacts.Add(new Contact(s.center, -outward, s.radius + best));
            return true;
        }

        private static readonly Vector3[] Corners = new Vector3[8];

        /// <summary>Oriented box (A) against a plane (B): every corner below the plane is a contact point.</summary>
        public static bool ObbPlane(Obb box, Plane3D plane, List<Contact> contacts, float margin = 0f)
        {
            box.GetCorners(Corners);
            bool hit = false;
            for (int i = 0; i < 8; i++)
            {
                float dist = plane.SignedDistance(Corners[i]);
                if (dist < margin)
                {
                    contacts.Add(new Contact(Corners[i] - plane.normal * (0.5f * dist), -plane.normal, -dist));
                    hit = true;
                }
            }
            return hit;
        }

        // Scratch buffers for polygon clipping (single-threaded use).
        private static readonly List<Vector3> ClipA = new List<Vector3>(16);
        private static readonly List<Vector3> ClipB = new List<Vector3>(16);

        /// <summary>
        /// Box-box contact manifold. The separating axis theorem (chapter 4, slide 43) is run on the 15 candidate axes;
        /// the axis of smallest overlap gives the normal and the depth. For a face axis, the incident face of the other
        /// box is clipped (Sutherland-Hodgman) against the side planes of the reference face, giving up to 8 points.
        /// For an edge-edge axis, the contact is the closest point between the two edges (chapter 3, line-line distance).
        /// </summary>
        public static bool ObbObb(Obb a, Obb b, List<Contact> contacts)
        {
            Vector3 t = b.center - a.center;
            Vector3 a0 = a.AxisX, a1 = a.AxisY, a2 = a.AxisZ;
            Vector3 b0 = b.AxisX, b1 = b.AxisY, b2 = b.AxisZ;

            float bestFace = float.MaxValue;
            int bestFaceAxis = -1;
            Vector3 bestFaceN = VMath.Zero;

            for (int i = 0; i < 6; i++)
            {
                Vector3 l = i == 0 ? a0 : i == 1 ? a1 : i == 2 ? a2 : i == 3 ? b0 : i == 4 ? b1 : b2;
                float tl = VMath.Dot(t, l);
                float overlap = a.ProjectionRadius(l) + b.ProjectionRadius(l) - Mathf.Abs(tl);
                if (overlap < 0f)
                    return false;
                if (overlap < bestFace)
                {
                    bestFace = overlap;
                    bestFaceAxis = i;
                    bestFaceN = tl >= 0f ? l : -l;
                }
            }

            float bestEdge = float.MaxValue;
            int bestEi = -1, bestEj = -1;
            Vector3 bestEdgeN = VMath.Zero;
            for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                Vector3 l = VMath.Cross(a.Axis(i), b.Axis(j));
                float len = VMath.Length(l);
                if (len < 1e-4f)
                    continue; // Parallel edges: would give a false separating axis (slide 43 warning).
                l /= len;
                float tl = VMath.Dot(t, l);
                float overlap = a.ProjectionRadius(l) + b.ProjectionRadius(l) - Mathf.Abs(tl);
                if (overlap < 0f)
                    return false;
                if (overlap < bestEdge)
                {
                    bestEdge = overlap;
                    bestEi = i;
                    bestEj = j;
                    bestEdgeN = tl >= 0f ? l : -l;
                }
            }

            // Prefer face contacts (stable stacking) unless the edge axis is clearly better.
            if (bestEi >= 0 && bestEdge * 1.05f + 1e-4f < bestFace)
            {
                Vector3 n = bestEdgeN;
                Vector3 pa = a.center, pb = b.center;
                for (int k = 0; k < 3; k++)
                {
                    if (k != bestEi)
                    {
                        Vector3 ax = a.Axis(k);
                        pa += ax * (a.Extent(k) * VMath.Sign(VMath.Dot(ax, n)));
                    }
                    if (k != bestEj)
                    {
                        Vector3 bx = b.Axis(k);
                        pb += bx * (b.Extent(k) * VMath.Sign(VMath.Dot(bx, -n)));
                    }
                }
                Vector3 ea = a.Axis(bestEi) * a.Extent(bestEi);
                Vector3 eb = b.Axis(bestEj) * b.Extent(bestEj);
                Distance.SegmentSegment(new Segment(pa - ea, pa + ea), new Segment(pb - eb, pb + eb),
                                        out Vector3 c1, out Vector3 c2, out _, out _);
                contacts.Add(new Contact((c1 + c2) * 0.5f, n, bestEdge));
                return true;
            }

            // Face contact.
            bool refIsA = bestFaceAxis < 3;
            Obb refBox = refIsA ? a : b;
            Obb incBox = refIsA ? b : a;
            Vector3 refNormal = refIsA ? bestFaceN : -bestFaceN; // Outward normal of the reference face.
            int refAxis = refIsA ? bestFaceAxis : bestFaceAxis - 3;

            Vector3 refCenter = refBox.center + refNormal * refBox.Extent(refAxis);
            int u1Index = (refAxis + 1) % 3, u2Index = (refAxis + 2) % 3;
            Vector3 u1 = refBox.Axis(u1Index), u2 = refBox.Axis(u2Index);
            float e1 = refBox.Extent(u1Index), e2 = refBox.Extent(u2Index);

            // Incident face: the face of the other box most anti-parallel to the reference normal.
            int incAxis = 0;
            float bestDot = -1f;
            for (int k = 0; k < 3; k++)
            {
                float dd = Mathf.Abs(VMath.Dot(incBox.Axis(k), refNormal));
                if (dd > bestDot)
                {
                    bestDot = dd;
                    incAxis = k;
                }
            }
            Vector3 incN = incBox.Axis(incAxis);
            if (VMath.Dot(incN, refNormal) > 0f)
                incN = -incN;
            Vector3 incCenter = incBox.center + incN * incBox.Extent(incAxis);
            int i1 = (incAxis + 1) % 3, i2 = (incAxis + 2) % 3;
            Vector3 v1 = incBox.Axis(i1) * incBox.Extent(i1);
            Vector3 v2 = incBox.Axis(i2) * incBox.Extent(i2);

            ClipA.Clear();
            ClipA.Add(incCenter + v1 + v2);
            ClipA.Add(incCenter - v1 + v2);
            ClipA.Add(incCenter - v1 - v2);
            ClipA.Add(incCenter + v1 - v2);

            float c1Offset = VMath.Dot(u1, refCenter), c2Offset = VMath.Dot(u2, refCenter);
            ClipPolygon(ClipA, ClipB, u1, c1Offset + e1);
            ClipPolygon(ClipB, ClipA, -u1, -c1Offset + e1);
            ClipPolygon(ClipA, ClipB, u2, c2Offset + e2);
            ClipPolygon(ClipB, ClipA, -u2, -c2Offset + e2);

            bool any = false;
            float refOffset = VMath.Dot(refNormal, refCenter);
            for (int k = 0; k < ClipA.Count; k++)
            {
                Vector3 p = ClipA[k];
                float depth = refOffset - VMath.Dot(refNormal, p);
                if (depth >= 0f)
                {
                    contacts.Add(new Contact(p + refNormal * (0.5f * depth), bestFaceN, depth));
                    any = true;
                }
            }
            if (!any)
            {
                // Numerical corner case: fall back to a single contact at the incident face center.
                contacts.Add(new Contact(incCenter, bestFaceN, bestFace));
            }
            return true;
        }

        /// <summary>Sutherland-Hodgman: keeps the part of polygon "input" where dot(n, p) &lt;= offset.</summary>
        private static void ClipPolygon(List<Vector3> input, List<Vector3> output, Vector3 n, float offset)
        {
            output.Clear();
            int count = input.Count;
            if (count == 0)
                return;
            Vector3 prev = input[count - 1];
            float prevD = VMath.Dot(n, prev) - offset;
            for (int i = 0; i < count; i++)
            {
                Vector3 cur = input[i];
                float curD = VMath.Dot(n, cur) - offset;
                if (curD <= 0f)
                {
                    if (prevD > 0f)
                        output.Add(prev + (cur - prev) * (prevD / (prevD - curD)));
                    output.Add(cur);
                }
                else if (prevD <= 0f)
                {
                    output.Add(prev + (cur - prev) * (prevD / (prevD - curD)));
                }
                prev = cur;
                prevD = curD;
            }
        }
    }
}
