namespace GamiX.Physics.Unified
{
    /// <summary>
    /// Extension point of the unified solver. A simulation adds its own forces, constraints and velocity effects
    /// to Algorithm 1 without editing the library: derive from this class and register it with
    /// <see cref="UnifiedSolver.AddExtension{T}"/> (or <see cref="UnifiedSolver.GetOrAddExtension{T}"/>).
    ///
    /// The solver calls the hooks once per sub-step, in this order:
    /// <list type="number">
    /// <item><see cref="ApplyForces"/>: Algorithm 1 lines 1-3, before the positions are predicted.</item>
    /// <item><see cref="SolveConstraints"/>: lines 16-22, once per solver iteration, after the built-in groups.
    /// Write corrections with <see cref="UnifiedSolver.AddCorrection"/>; they are averaged and over-relaxed (eq. 12-13).</item>
    /// <item><see cref="UpdateVelocities"/>: lines 24-26, after v = (x* - x) / h and the built-in fluid forces.</item>
    /// <item><see cref="EndSubStep"/>: after the velocities are final, before x = x*.</item>
    /// </list>
    /// </summary>
    public abstract class SolverExtension
    {
        /// <summary>The solver this extension was added to.</summary>
        public UnifiedSolver Solver { get; internal set; }

        /// <summary>Called once when the extension is added to a solver.</summary>
        public virtual void OnAdded()
        {
        }

        /// <summary>External forces (change <see cref="UnifiedSolver.velocities"/>), before prediction.</summary>
        public virtual void ApplyForces(float h)
        {
        }

        /// <summary>
        /// Projects this extension's constraints on <see cref="UnifiedSolver.predicted"/>.
        /// Return true when corrections were added (the solver then applies them).
        /// </summary>
        public virtual bool SolveConstraints(int iteration)
        {
            return false;
        }

        /// <summary>Velocity-level effects (drag, turbulence...), after the velocity update.</summary>
        public virtual void UpdateVelocities(float h)
        {
        }

        /// <summary>End of the sub-step (passive particles, statistics...).</summary>
        public virtual void EndSubStep(float h)
        {
        }
    }
}
