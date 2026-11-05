using System;
using System.Collections.Generic;
using GamiX.Physics;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Yassine
{
    /// <summary>Measurements on the solver state, used by the on-screen readouts and the tests.</summary>
    public static class FluidStats
    {
        /// <summary>Average and maximum rho / rho0 over the given fluid particles (density constraint, eq. 26).</summary>
        public static void DensityRatio(UnifiedSolver s, IEnumerable<int> particles, out float average, out float max)
        {
            average = 0f;
            max = 0f;
            float rho0 = s.RestDensity;
            if (rho0 <= 0f) return;
            int n = 0;
            foreach (int i in particles)
            {
                if (!s.active[i]) continue;
                float r = s.densities[i] / rho0;
                average += r;
                if (r > max) max = r;
                n++;
            }
            if (n > 0) average /= n;
        }

        /// <summary>Mean height of the given particles.</summary>
        public static float MeanHeight(UnifiedSolver s, IEnumerable<int> particles)
        {
            float sum = 0f;
            int n = 0;
            foreach (int i in particles)
            {
                if (!s.active[i]) continue;
                sum += s.positions[i].y;
                n++;
            }
            return n > 0 ? sum / n : 0f;
        }

        /// <summary>Largest x reached by the given particles (front of a dam break).</summary>
        public static float FrontX(UnifiedSolver s, IEnumerable<int> particles)
        {
            float x = float.NegativeInfinity;
            foreach (int i in particles)
                if (s.active[i] && s.positions[i].x > x)
                    x = s.positions[i].x;
            return x;
        }

        /// <summary>
        /// Height of the free surface: a high percentile of the particle heights (robust to a few splashing
        /// particles) plus half a rest spacing, since particle centers sit half a spacing below the surface.
        /// </summary>
        public static float SurfaceHeight(UnifiedSolver s, IEnumerable<int> particles, float percentile = 0.97f)
        {
            var heights = new List<float>();
            foreach (int i in particles)
                if (s.active[i])
                    heights.Add(s.positions[i].y);
            if (heights.Count == 0) return 0f;
            heights.Sort();
            int k = Math.Min(heights.Count - 1, (int)(percentile * (heights.Count - 1)));
            return heights[k] + 0.5f * s.parameters.FluidRestDistance;
        }

        /// <summary>
        /// Fraction of a rigid body below the given surface height, from the vertical extent of its particles
        /// (each particle is a sphere of the solver radius). For a floating body Archimedes predicts
        /// rho_body / rho_fluid.
        /// </summary>
        public static float SubmergedFraction(UnifiedSolver s, RigidGroup body, float surface)
        {
            float lo = float.PositiveInfinity, hi = float.NegativeInfinity;
            foreach (int i in body.indices)
            {
                float y = s.positions[i].y;
                if (y < lo) lo = y;
                if (y > hi) hi = y;
            }
            float r = s.parameters.radius;
            lo -= r;
            hi += r;
            return hi > lo ? VMath.Clamp01((surface - lo) / (hi - lo)) : 0f;
        }

        /// <summary>Lowest point of a rigid body (bottom of its particles).</summary>
        public static float Bottom(UnifiedSolver s, RigidGroup body)
        {
            float lo = float.PositiveInfinity;
            foreach (int i in body.indices)
                lo = Mathf.Min(lo, s.positions[i].y);
            return lo - s.parameters.radius;
        }

        /// <summary>True when every active particle has a finite position and velocity.</summary>
        public static bool AllFinite(UnifiedSolver s)
        {
            for (int i = 0; i < s.Count; i++)
            {
                if (!s.active[i]) continue;
                if (!VMath.IsFinite(s.positions[i]) || !VMath.IsFinite(s.velocities[i]))
                    return false;
            }
            return true;
        }
    }
}
