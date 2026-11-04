using System;
using System.Collections.Generic;
using GamiX.Physics;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Cloth
{
    /// <summary>Builds cloth out of particles and distance constraints (paper section 8).</summary>
    public static class ClothBuilder
    {
        /// <summary>
        /// Rectangular cloth of (resU x resV) particles in the plane (origin, axisU, axisV).
        /// Stretch constraints along the grid edges, shear constraints along the diagonals and bending constraints
        /// across edges (distance between particles two apart), as in section 8.
        /// </summary>
        public static ClothHandle CreateCloth(UnifiedSolver solver, Vector3 origin, Vector3 axisU, Vector3 axisV,
                                              int resU, int resV, float particleMass,
                                              float stretchStiffness = 1f, float shearStiffness = 0.8f, float bendStiffness = 0.3f,
                                              Func<int, int, bool> isPinned = null, float tearRatio = 0f)
        {
            int group = solver.NewGroup();
            int phase = Phase.Make(group, Phase.SelfCollide | Phase.SelfCollideFilter);
            var handle = new ClothHandle { width = resU, height = resV, phase = phase, particles = new int[resU * resV] };
            float w = particleMass > 0f ? 1f / particleMass : 0f;

            for (int v = 0; v < resV; v++)
            for (int u = 0; u < resU; u++)
            {
                Vector3 p = origin + axisU * (resU > 1 ? (float)u / (resU - 1) : 0f) + axisV * (resV > 1 ? (float)v / (resV - 1) : 0f);
                bool pinned = isPinned != null && isPinned(u, v);
                handle.particles[v * resU + u] = solver.AddParticle(p, VMath.Zero, pinned ? 0f : w, phase);
            }

            int Idx(int u, int v) => handle.particles[v * resU + u];
            for (int v = 0; v < resV; v++)
            for (int u = 0; u < resU; u++)
            {
                if (u + 1 < resU) solver.AddDistanceConstraint(Idx(u, v), Idx(u + 1, v), stretchStiffness, tearRatio, 0);
                if (v + 1 < resV) solver.AddDistanceConstraint(Idx(u, v), Idx(u, v + 1), stretchStiffness, tearRatio, 0);
                if (u + 1 < resU && v + 1 < resV)
                {
                    solver.AddDistanceConstraint(Idx(u, v), Idx(u + 1, v + 1), shearStiffness, tearRatio, 1);
                    solver.AddDistanceConstraint(Idx(u + 1, v), Idx(u, v + 1), shearStiffness, tearRatio, 1);
                }
                if (bendStiffness > 0f)
                {
                    if (u + 2 < resU) solver.AddDistanceConstraint(Idx(u, v), Idx(u + 2, v), bendStiffness, 0f, 2);
                    if (v + 2 < resV) solver.AddDistanceConstraint(Idx(u, v), Idx(u, v + 2), bendStiffness, 0f, 2);
                }
            }

            var tris = new List<int>();
            for (int v = 0; v + 1 < resV; v++)
            for (int u = 0; u + 1 < resU; u++)
            {
                tris.Add(Idx(u, v)); tris.Add(Idx(u, v + 1)); tris.Add(Idx(u + 1, v));
                tris.Add(Idx(u + 1, v)); tris.Add(Idx(u, v + 1)); tris.Add(Idx(u + 1, v + 1));
            }
            handle.triangles = tris.ToArray();
            solver.Cloths.Add(handle);
            return handle;
        }

        /// <summary>
        /// Adds tethers (long range attachments) from every free cloth particle to its closest pinned particle,
        /// with the rest distance as maximum length (reduces stretching, section 8).
        /// </summary>
        public static void AddClothTethers(UnifiedSolver solver, ClothHandle cloth, float slack = 1.05f)
        {
            var pinned = new List<int>();
            foreach (int i in cloth.particles)
                if (solver.invMass[i] == 0f)
                    pinned.Add(i);
            if (pinned.Count == 0)
                return;
            ClothTethers tethers = solver.GetOrAddExtension<ClothTethers>();
            foreach (int i in cloth.particles)
            {
                if (solver.invMass[i] == 0f) continue;
                int best = pinned[0];
                float bestD = float.MaxValue;
                foreach (int a in pinned)
                {
                    float d = VMath.DistanceSq(solver.positions[i], solver.positions[a]);
                    if (d < bestD) { bestD = d; best = a; }
                }
                tethers.Add(i, best, slack);
            }
        }
    }
}
