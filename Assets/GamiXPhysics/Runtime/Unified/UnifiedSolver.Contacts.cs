using System;
using UnityEngine;

namespace GamiX.Physics.Unified
{
    public sealed partial class UnifiedSolver
    {
        // Contacts are stored per particle ("particle-centric", paper section 9, Algorithm 2): each particle owns the
        // list of its contact partners and only ever writes its own correction, so the solve runs in parallel.
        // A pair (i, j) appears in both lists and is always evaluated in the same canonical order (lower index first),
        // so both particles see exactly the same correction, as in the serial version.
        private const int MaxContactsPerParticle = 48;
        private const int MaxShapeContactsPerParticle = 6;

        private int[] contactPartners = new int[0];
        private float[] contactDistances = new float[0];
        private int[] contactCounts = new int[0];
        private int particleContactTotal;

        private int[] shapeContactShape = new int[0];
        private float[] shapeContactNormalVelocity = new float[0];
        private Vector3[] shapeContactNormal = new Vector3[0];
        private int[] shapeContactCounts = new int[0];
        private int shapeContactTotal;
        private Geometry.Aabb[] shapeBounds = new Geometry.Aabb[0];

        // Arguments of the parallel bodies (constant during one parallel call).
        private Vector3[] contactX;
        private bool contactStabilization;
        private float contactStepH;

        private Action<int, int> buildParticleContactsRange, buildShapeContactsRange;
        private Action<int, int> solveParticleContactsRange, solveShapeContactsRange;

        private void CacheContactDelegates()
        {
            buildParticleContactsRange = BuildParticleContactsRange;
            buildShapeContactsRange = BuildShapeContactsRange;
            solveParticleContactsRange = SolveParticleContactsRange;
            solveShapeContactsRange = SolveShapeContactsRange;
        }

        public int ParticleContactCount => particleContactTotal / 2;
        public int ShapeContactCount => shapeContactTotal;

        /// <summary>Radius used against kinematic shapes (fluid particles use half their rest spacing).</summary>
        private float ShapeRadius(int i)
        {
            return Phase.IsFluid(phases[i]) ? 0.5f * parameters.FluidRestDistance : parameters.radius;
        }

        /// <summary>Phase rules: which particle pairs get a contact constraint. Symmetric in i and j.</summary>
        private bool CanCollide(int i, int j)
        {
            int pi = phases[i], pj = phases[j];
            bool fi = (pi & Phase.Fluid) != 0, fj = (pj & Phase.Fluid) != 0;
            if (fi && fj)
                return false; // Fluid-fluid interactions are handled by the density constraint.
            if (invMass[i] == 0f && invMass[j] == 0f)
                return false;
            if (Phase.Group(pi) == Phase.Group(pj))
            {
                if ((pi & Phase.SelfCollide) == 0)
                    return false;
                if ((pi & Phase.SelfCollideFilter) != 0 &&
                    VMath.DistanceSq(restPositions[i], restPositions[j]) < parameters.SolidRestDistance * parameters.SolidRestDistance * 1.02f)
                    return false;
            }
            return true;
        }

        private float ContactDistance(int i, int j)
        {
            // Fluid-solid contacts use the fluid rest distance (section 7.1).
            if (Phase.IsFluid(phases[i]) || Phase.IsFluid(phases[j]))
                return parameters.FluidRestDistance;
            return parameters.SolidRestDistance;
        }

        private void EnsureContactCapacity()
        {
            if (contactCounts.Length < Count)
            {
                int cap = positions.Length;
                contactCounts = new int[cap];
                contactPartners = new int[cap * MaxContactsPerParticle];
                contactDistances = new float[cap * MaxContactsPerParticle];
                shapeContactCounts = new int[cap];
                shapeContactShape = new int[cap * MaxShapeContactsPerParticle];
                shapeContactNormalVelocity = new float[cap * MaxShapeContactsPerParticle];
                shapeContactNormal = new Vector3[cap * MaxShapeContactsPerParticle];
            }
        }

        // ================================================================ build

        private void BuildParticleContacts()
        {
            EnsureContactCapacity();
            ParallelForRange(Count, buildParticleContactsRange);
            int total = 0;
            for (int i = 0; i < Count; i++)
                total += contactCounts[i];
            particleContactTotal = total;
        }

        private void BuildParticleContactsRange(int begin, int end)
        {
            float margin = 2f * parameters.radius * parameters.collisionMargin;
            for (int i = begin; i < end; i++)
            {
                int count = 0;
                if (active[i])
                {
                    Vector3 xi = predicted[i];
                    int n = neighborCounts[i];
                    int nb = i * maxNeighbors;
                    int cb = i * MaxContactsPerParticle;
                    for (int k = 0; k < n && count < MaxContactsPerParticle; k++)
                    {
                        int j = neighbors[nb + k];
                        if (!CanCollide(i, j))
                            continue;
                        float dist = ContactDistance(i, j);
                        float r = dist + margin;
                        Vector3 xj = predicted[j];
                        float dx = xi.x - xj.x, dy = xi.y - xj.y, dz = xi.z - xj.z;
                        if (dx * dx + dy * dy + dz * dz < r * r)
                        {
                            contactPartners[cb + count] = j;
                            contactDistances[cb + count] = dist;
                            count++;
                        }
                    }
                }
                contactCounts[i] = count;
            }
        }

        private void BuildShapeContacts(float h)
        {
            EnsureContactCapacity();
            if (shapeBounds.Length < Shapes.Count)
                shapeBounds = new Geometry.Aabb[Shapes.Count];
            for (int s = 0; s < Shapes.Count; s++)
                shapeBounds[s] = Shapes[s].WorldBounds;
            contactStepH = h;
            if (Shapes.Count == 0)
            {
                Array.Clear(shapeContactCounts, 0, Count);
                shapeContactTotal = 0;
                return;
            }
            ParallelForRange(Count, buildShapeContactsRange);
            int total = 0;
            for (int i = 0; i < Count; i++)
                total += shapeContactCounts[i];
            shapeContactTotal = total;
        }

        private void BuildShapeContactsRange(int begin, int end)
        {
            float margin = 2f * parameters.radius * parameters.collisionMargin;
            float h = contactStepH;
            int shapeCount = Shapes.Count;
            for (int i = begin; i < end; i++)
            {
                int count = 0;
                if (active[i] && invMass[i] != 0f)
                {
                    Vector3 x = predicted[i];
                    Vector3 v = velocities[i];
                    float r = ShapeRadius(i) + margin + Mathf.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z) * h;
                    int cb = i * MaxShapeContactsPerParticle;
                    for (int s = 0; s < shapeCount && count < MaxShapeContactsPerParticle; s++)
                    {
                        KinematicShape shape = Shapes[s];
                        if (!shape.enabled) continue;
                        if (!shape.inverted)
                        {
                            Geometry.Aabb b = shapeBounds[s];
                            if (x.x < b.min.x - r || x.x > b.max.x + r || x.y < b.min.y - r || x.y > b.max.y + r ||
                                x.z < b.min.z - r || x.z > b.max.z + r)
                                continue;
                        }
                        float d = shape.SignedDistance(x, out Vector3 n);
                        if (d < r)
                        {
                            shapeContactShape[cb + count] = s;
                            shapeContactNormal[cb + count] = n;
                            shapeContactNormalVelocity[cb + count] = VMath.Dot(v - shape.PointVelocity(x), n);
                            count++;
                        }
                    }
                }
                shapeContactCounts[i] = count;
            }
        }

        // ================================================================ shape contacts

        /// <summary>
        /// Contacts with kinematic shapes: C = phi(x) - r &gt;= 0, projected along the shape normal, followed by the
        /// position-level friction of eq. 23-24 relative to the (possibly moving) shape.
        /// </summary>
        private void SolveShapeContacts(Vector3[] x, bool stabilization)
        {
            if (shapeContactTotal == 0)
                return;
            contactX = x;
            contactStabilization = stabilization;
            ParallelForRange(Count, solveShapeContactsRange);
        }

        private void SolveShapeContactsRange(int begin, int end)
        {
            SolverParams p = parameters;
            Vector3[] x = contactX;
            bool stabilization = contactStabilization;
            float h = dt;
            for (int i = begin; i < end; i++)
            {
                int count = shapeContactCounts[i];
                if (count == 0) continue;
                int cb = i * MaxShapeContactsPerParticle;
                float radius = ShapeRadius(i);
                bool fluid = Phase.IsFluid(phases[i]);
                Vector3 sum = VMath.Zero;
                int applied = 0;
                for (int k = 0; k < count; k++)
                {
                    KinematicShape shape = Shapes[shapeContactShape[cb + k]];
                    Vector3 xi = x[i];
                    float d = shape.SignedDistance(xi, out Vector3 n);
                    float depth = radius - d;
                    if (depth <= 0f)
                        continue;

                    Vector3 delta = n * depth;
                    if (!stabilization)
                    {
                        float mu = shape.friction >= 0f ? shape.friction : 1f;
                        if (fluid) mu *= p.fluidFriction;
                        float muS = p.staticFriction * mu, muK = p.dynamicFriction * mu;
                        if (muS > 0f || muK > 0f)
                        {
                            // Relative tangential displacement during this step (eq. 23), the shape counts as particle j.
                            Vector3 disp = (xi + delta - positions[i]) - shape.PointVelocity(xi) * h;
                            Vector3 tangential = disp - n * VMath.Dot(disp, n);
                            float tLen = VMath.Length(tangential);
                            if (tLen > 0f)
                            {
                                // Eq. 24: static friction removes all tangential motion, kinetic limits it by mu_k * d.
                                if (tLen < muS * depth)
                                    delta -= tangential;
                                else
                                    delta -= tangential * Mathf.Min(muK * depth / tLen, 1f);
                            }
                        }
                    }
                    sum += delta;
                    applied++;
                }
                if (applied > 0)
                {
                    deltas[i] += sum;
                    deltaCounts[i] += applied;
                }
            }
        }

        // ================================================================ particle contacts

        /// <summary>
        /// Particle-particle contacts (sections 5.1 and 6.1).
        /// - Default: non-penetration C = |x_ij| - r &gt;= 0 (eq. 22) along x_ij.
        /// - Two interior rigid particles of different bodies: the SDF gradient of the particle closest to its surface
        ///   gives the minimum translation direction and depth d = min(|phi_i|, |phi_j|) (eq. 17-19). This prevents
        ///   bodies from interlocking after deep penetration.
        /// - A rigid boundary particle (|phi| &lt; r) acts as a one-sided sphere: the collision direction is reflected
        ///   about the SDF gradient when the other particle lies behind the surface (eq. 20).
        /// Then friction at the position level (eq. 23-25): static when the tangential motion is below mu_s d,
        /// Coulomb-limited by mu_k d otherwise. The correction is shared according to inverse masses.
        /// </summary>
        private void SolveParticleContacts(Vector3[] x, bool stabilization)
        {
            if (particleContactTotal == 0)
                return;
            contactX = x;
            contactStabilization = stabilization;
            ParallelForRange(Count, solveParticleContactsRange);
        }

        private void SolveParticleContactsRange(int begin, int end)
        {
            Vector3[] x = contactX;
            bool stabilization = contactStabilization;
            for (int i = begin; i < end; i++)
            {
                int count = contactCounts[i];
                if (count == 0) continue;
                int cb = i * MaxContactsPerParticle;
                Vector3 sum = VMath.Zero;
                int applied = 0;
                for (int k = 0; k < count; k++)
                {
                    int j = contactPartners[cb + k];
                    // Canonical order: always evaluate the pair from its lower index.
                    bool iIsA = i < j;
                    int a = iIsA ? i : j, b = iIsA ? j : i;
                    if (!PairCorrection(a, b, contactDistances[cb + k], x, stabilization, out Vector3 da, out Vector3 db))
                        continue;
                    sum += iIsA ? da : db;
                    applied++;
                }
                if (applied > 0)
                {
                    deltas[i] += sum;
                    deltaCounts[i] += applied;
                }
            }
        }

        /// <summary>Correction of the contact (a, b) for both particles. Pure function of the current positions.</summary>
        private bool PairCorrection(int a, int b, float dist, Vector3[] x, bool stabilization, out Vector3 da, out Vector3 db)
        {
            da = db = VMath.Zero;
            float wa = contactInvMass[a], wb = contactInvMass[b];
            float w = wa + wb;
            if (w <= 0f)
                return false;

            Vector3 xa = x[a], xb = x[b];
            float ex = xa.x - xb.x, ey = xa.y - xb.y, ez = xa.z - xb.z;
            float len2 = ex * ex + ey * ey + ez * ez;
            if (len2 >= dist * dist)
                return false;
            float len = Mathf.Sqrt(len2);
            Vector3 xab = new Vector3(ex, ey, ez);
            float r = parameters.radius;

            // dirA: direction in which particle a is pushed (b is pushed the opposite way).
            Vector3 dirA;
            float depth;
            int ga = rigidOf[a], gb = rigidOf[b];

            if (ga >= 0 && gb >= 0 && ga != gb)
            {
                RigidGroup A = Rigids[ga], B = Rigids[gb];
                float phiA = Mathf.Abs(A.sdf[rigidSlot[a]]), phiB = Mathf.Abs(B.sdf[rigidSlot[b]]);
                Vector3 gradA = A.rotation.Rotate(A.sdfGradient[rigidSlot[a]]);
                Vector3 gradB = B.rotation.Rotate(B.sdfGradient[rigidSlot[b]]);
                bool boundaryA = phiA < r, boundaryB = phiB < r;

                if (!boundaryA && !boundaryB)
                {
                    // Eq. 17: n_ab = grad(phi_a) if |phi_a| < |phi_b| else -grad(phi_b), d = min(|phi|).
                    if (phiA < phiB)
                    {
                        dirA = -gradA;
                        depth = phiA;
                    }
                    else
                    {
                        dirA = gradB;
                        depth = phiB;
                    }
                }
                else
                {
                    // Eq. 20 on the boundary particle closest to its surface; d = r - |x_ab| (one-sided sphere).
                    bool useA = boundaryA && (!boundaryB || phiA <= phiB);
                    dirA = OneSidedDirection(xab, len, useA ? gradA : gradB, useA);
                    depth = dist - len;
                }
            }
            else if (ga >= 0 && gb < 0)
            {
                RigidGroup A = Rigids[ga];
                if (Mathf.Abs(A.sdf[rigidSlot[a]]) < r)
                    dirA = OneSidedDirection(xab, len, A.rotation.Rotate(A.sdfGradient[rigidSlot[a]]), true);
                else
                    dirA = len > 1e-9f ? xab / len : VMath.Up;
                depth = dist - len;
            }
            else if (gb >= 0 && ga < 0)
            {
                RigidGroup B = Rigids[gb];
                if (Mathf.Abs(B.sdf[rigidSlot[b]]) < r)
                    dirA = OneSidedDirection(xab, len, B.rotation.Rotate(B.sdfGradient[rigidSlot[b]]), false);
                else
                    dirA = len > 1e-9f ? xab / len : VMath.Up;
                depth = dist - len;
            }
            else
            {
                dirA = len > 1e-9f ? xab / len : VMath.Up;
                depth = dist - len;
            }

            if (depth <= 0f)
                return false;

            da = dirA * (depth * wa / w);
            db = dirA * (-depth * wb / w);

            SolverParams p = parameters;
            float pairFriction = Phase.IsFluid(phases[a]) || Phase.IsFluid(phases[b]) ? p.fluidFriction : 1f;
            float muS = p.staticFriction * p.particleFriction * pairFriction;
            float muK = p.dynamicFriction * p.particleFriction * pairFriction;
            if (!stabilization && (muS > 0f || muK > 0f))
            {
                // Eq. 23: relative tangential displacement of the pair during the step.
                Vector3 rel = (xa + da - positions[a]) - (xb + db - positions[b]);
                Vector3 tangential = rel - dirA * VMath.Dot(rel, dirA);
                float tLen = VMath.Length(tangential);
                if (tLen > 0f)
                {
                    Vector3 f = tLen < muS * depth ? tangential : tangential * Mathf.Min(muK * depth / tLen, 1f);
                    // Eq. 24-25: removing the relative motion, shared by inverse mass.
                    da -= f * (wa / w);
                    db += f * (wb / w);
                }
            }
            return true;
        }

        /// <summary>
        /// One-sided collision normal of eq. 20. The boundary particle has outward SDF gradient g; the collision
        /// direction of the other particle is reflected about g when it points into the shape, so the other particle
        /// is always pushed out through the surface. Returns the push direction of the first particle of the pair.
        /// </summary>
        private static Vector3 OneSidedDirection(Vector3 xab, float len, Vector3 gradient, bool boundaryIsA)
        {
            // Direction from the boundary particle toward the other one.
            Vector3 dirOther = len > 1e-9f ? (boundaryIsA ? -xab : xab) / len : gradient;
            float dn = VMath.Dot(dirOther, gradient);
            if (dn < 0f)
                dirOther = VMath.NormalizeOr(dirOther - gradient * (2f * dn), gradient);
            // The other particle moves along dirOther, the boundary particle the opposite way.
            return boundaryIsA ? -dirOther : dirOther;
        }

        /// <summary>Optional bounce on kinematic shapes: reflects the normal velocity measured before the solve.</summary>
        private void ApplyShapeRestitution()
        {
            float e = parameters.shapeRestitution;
            if (e <= 0f || shapeContactTotal == 0)
                return;
            for (int i = 0; i < Count; i++)
            {
                int count = shapeContactCounts[i];
                int cb = i * MaxShapeContactsPerParticle;
                for (int k = 0; k < count; k++)
                {
                    float vn0 = shapeContactNormalVelocity[cb + k];
                    if (vn0 >= -0.5f) continue;
                    Vector3 n = shapeContactNormal[cb + k];
                    float vn = VMath.Dot(velocities[i], n);
                    float target = -e * vn0;
                    if (vn < target)
                        velocities[i] += n * (target - vn);
                }
            }
        }
    }
}
