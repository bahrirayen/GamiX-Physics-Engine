using GamiX.Physics;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Ropes
{
    /// <summary>A rope: ordered chain of particles connected by stretch and bending constraints.</summary>
    public sealed class RopeHandle
    {
        public int[] particles;
        public int phase;
    }

    /// <summary>Builds ropes out of particles and distance constraints (paper section 8).</summary>
    public static class RopeBuilder
    {
        /// <summary>Rope: chain of particles with stretch constraints and bending constraints (i, i+2).</summary>
        public static RopeHandle CreateRope(UnifiedSolver solver, Vector3 start, Vector3 end, float particleMass,
                                            float stretchStiffness = 1f, float bendStiffness = 0.2f,
                                            bool pinStart = false, bool pinEnd = false, float spacingScale = 1f)
        {
            float spacing = solver.parameters.SolidRestDistance * spacingScale;
            float length = VMath.Distance(start, end);
            int n = Mathf.Max(2, Mathf.RoundToInt(length / spacing) + 1);
            int phase = Phase.Make(solver.NewGroup(), Phase.SelfCollide | Phase.SelfCollideFilter);
            var rope = new RopeHandle { particles = new int[n], phase = phase };
            float w = particleMass > 0f ? 1f / particleMass : 0f;
            for (int k = 0; k < n; k++)
            {
                Vector3 p = VMath.Lerp(start, end, (float)k / (n - 1));
                bool pinned = (k == 0 && pinStart) || (k == n - 1 && pinEnd);
                rope.particles[k] = solver.AddParticle(p, VMath.Zero, pinned ? 0f : w, phase);
            }
            for (int k = 0; k + 1 < n; k++)
                solver.AddDistanceConstraint(rope.particles[k], rope.particles[k + 1], stretchStiffness, 0f, 0);
            if (bendStiffness > 0f)
                for (int k = 0; k + 2 < n; k++)
                    solver.AddDistanceConstraint(rope.particles[k], rope.particles[k + 2], bendStiffness, 0f, 2);
            return rope;
        }
    }
}
