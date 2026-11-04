using UnityEngine;

namespace GamiX.Physics.Unified
{
    public sealed partial class UnifiedSolver
    {
        /// <summary>
        /// Distance constraints (cloth stretch/bend, ropes): C = |x_ij| - L, gradient along x_ij (eq. 3-5 for a pair):
        /// dx_i = -w_i / (w_i + w_j) C n, dx_j = +w_j / (w_i + w_j) C n.
        /// The stiffness k is converted to k' = 1 - (1 - k)^(1/iterations) so the result does not depend on the
        /// iteration count (Mueller et al. 2007).
        /// In Gauss-Seidel mode the correction is applied immediately (sequential), otherwise it is accumulated and
        /// averaged like the other Jacobi groups.
        /// </summary>
        private void SolveDistanceConstraints(int iterationCount)
        {
            int iters = Mathf.Max(1, iterationCount);
            bool gaussSeidel = parameters.gaussSeidelDistance;
            DistanceConstraint[] items = DistanceConstraints.items;
            int count = DistanceConstraints.Count;
            for (int c = 0; c < count; c++)
            {
                ref DistanceConstraint dc = ref items[c];
                if (dc.broken) continue;
                int i = dc.i, j = dc.j;
                float wi = invMass[i], wj = invMass[j];
                float w = wi + wj;
                if (w <= 0f) continue;

                Vector3 pi = predicted[i], pj = predicted[j];
                float dx = pi.x - pj.x, dy = pi.y - pj.y, dz = pi.z - pj.z;
                float len = (float)System.Math.Sqrt(dx * dx + dy * dy + dz * dz);
                if (len < 1e-9f) continue;

                if (dc.tearRatio > 1f && len > dc.tearRatio * dc.restLength)
                {
                    dc.broken = true;
                    continue;
                }

                if (dc.effectiveIterations != iters || dc.effectiveSource != dc.stiffness)
                {
                    dc.effectiveStiffness = dc.stiffness >= 1f ? 1f : 1f - Mathf.Pow(1f - dc.stiffness, 1f / iters);
                    dc.effectiveIterations = iters;
                    dc.effectiveSource = dc.stiffness;
                }
                float s = dc.effectiveStiffness * (len - dc.restLength) / (len * w);
                float cx = dx * s, cy = dy * s, cz = dz * s;
                if (gaussSeidel)
                {
                    predicted[i] = new Vector3(pi.x - cx * wi, pi.y - cy * wi, pi.z - cz * wi);
                    predicted[j] = new Vector3(pj.x + cx * wj, pj.y + cy * wj, pj.z + cz * wj);
                }
                else
                {
                    Vector3 corr = new Vector3(cx, cy, cz);
                    deltas[i] -= corr * wi;
                    deltaCounts[i]++;
                    deltas[j] += corr * wj;
                    deltaCounts[j]++;
                }
            }
        }

        /// <summary>
        /// Rigid shape matching (section 5, eq. 15-16). For each rigid group:
        /// c = mass-weighted center of the predicted positions, A = sum m_i (x*_i - c) r_i^T, Q = rotation of the
        /// polar decomposition of A, then dx_i = (Q r_i + c) - x*_i. The least-squares best rigid transform keeps the
        /// inertial properties of the object without storing its linear or angular velocity.
        /// Plastic deformation (section 5.3) slowly moves r_i toward the deformed shape when the error is large.
        /// </summary>
        private void SolveShapeMatching()
        {
            // Groups own disjoint particles, so they can be matched in parallel.
            if (parameters.multithreaded && Rigids.Count >= 8)
                System.Threading.Tasks.Parallel.For(0, Rigids.Count, shapeMatchingBody);
            else
                for (int g = 0; g < Rigids.Count; g++)
                    SolveShapeMatchingGroup(g);
        }

        private void SolveShapeMatchingGroup(int g)
        {
            {
                RigidGroup rg = Rigids[g];
                if (!rg.active) return;
                int[] idx = rg.indices;
                int n = idx.Length;

                Vector3 c = VMath.Zero;
                float totalMass = 0f;
                for (int k = 0; k < n; k++)
                {
                    int i = idx[k];
                    float m = invMass[i] > 0f ? 1f / invMass[i] : 1e6f;
                    c += predicted[i] * m;
                    totalMass += m;
                }
                c /= totalMass;

                Mat3 A = Mat3.Zero;
                for (int k = 0; k < n; k++)
                {
                    int i = idx[k];
                    float m = invMass[i] > 0f ? 1f / invMass[i] : 1e6f;
                    A.AddOuter(predicted[i] - c, rg.restOffsets[k], m);
                }
                Quat q = rg.rotation;
                Mat3.ExtractRotation(A, ref q, 8);
                rg.rotation = q;
                rg.center = c;

                float stiffness = rg.stiffness;
                bool plastic = rg.plasticThreshold > 0f && rg.plasticCreep > 0f;
                for (int k = 0; k < n; k++)
                {
                    int i = idx[k];
                    if (invMass[i] == 0f) continue;
                    Vector3 goal = c + q.Rotate(rg.restOffsets[k]);
                    Vector3 delta = goal - predicted[i];
                    if (plastic && VMath.Length(delta) > rg.plasticThreshold)
                    {
                        // Mix the deformation back into the rest shape (plastic creep).
                        Vector3 deformedLocal = q.InverseRotate(predicted[i] - c);
                        rg.restOffsets[k] += (deformedLocal - rg.restOffsets[k]) * rg.plasticCreep;
                    }
                    deltas[i] += delta * stiffness;
                    deltaCounts[i]++;
                }
            }
        }
    }
}
