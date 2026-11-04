using GamiX.Physics;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Cloth
{
    /// <summary>
    /// Long range attachment ("tether", Kim et al. 2012): unilateral C = |x - anchor| - L &lt;= 0.
    /// The particle may move closer to its anchor but never farther than L. Prevents cloth over-stretching.
    /// </summary>
    public struct TetherConstraint
    {
        public int particle;
        /// <summary>Usually a pinned particle (inverse mass 0).</summary>
        public int anchor;
        public float maxLength;
    }

    /// <summary>Solver extension that projects the tethers once per solver iteration.</summary>
    public sealed class ClothTethers : SolverExtension
    {
        public readonly StructList<TetherConstraint> Tethers = new StructList<TetherConstraint>(64);

        /// <summary>Tether from a particle to an anchor, with max length = current distance x slack.</summary>
        public void Add(int particle, int anchor, float slack = 1f)
        {
            Tethers.Add(new TetherConstraint
            {
                particle = particle, anchor = anchor,
                maxLength = VMath.Distance(Solver.positions[particle], Solver.positions[anchor]) * slack
            });
        }

        /// <summary>Tethers only act when |x - anchor| &gt; L (unilateral).</summary>
        public override bool SolveConstraints(int iteration)
        {
            UnifiedSolver s = Solver;
            bool gaussSeidel = s.parameters.gaussSeidelDistance;
            bool any = false;
            for (int c = 0; c < Tethers.Count; c++)
            {
                ref TetherConstraint t = ref Tethers[c];
                int i = t.particle;
                if (s.invMass[i] == 0f) continue;
                Vector3 d = s.predicted[i] - s.predicted[t.anchor];
                float len = VMath.Length(d);
                if (len <= t.maxLength || len < 1e-9f) continue;
                Vector3 corr = d * ((len - t.maxLength) / len);
                if (gaussSeidel)
                {
                    s.predicted[i] -= corr;
                }
                else
                {
                    s.AddCorrection(i, -corr);
                    any = true;
                }
            }
            return any;
        }
    }
}
